import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSPresetProfiles

from assessment_interfaces.msg import ItemList, ItemHolders
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, PoseStamped
from sensor_msgs.msg import LaserScan
from nav2_simple_commander.robot_navigator import BasicNavigator

from tf_transformations import euler_from_quaternion
import angles

from enum import Enum
import math
import random

LINEAR_VELOCITY  = 0.3 # Metres per second
ANGULAR_VELOCITY = 0.5 # Radians per second

TURN_LEFT = 1 # Postive angular velocity turns left
TURN_RIGHT = -1 # Negative angular velocity turns right

SCAN_THRESHOLD = 0.5 # Metres per second
 # Array indexes for sensor sectors
SCAN_FRONT = 0
SCAN_LEFT = 1
SCAN_BACK = 2
SCAN_RIGHT = 3

# Finite state machine (FSM) states
class State(Enum):
    FORWARD = 0
    TURNING = 1
    COLLECTING = 2
    RETURNING = 3

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        
        self.robot_id = self.get_namespace().strip('/')  # Each robot knows who they are, to avoid conflicts in multi-robot simulations
        # self.get_logger().info(f'{self.robot_id}')
        
        self.previous_state = None
        
        # Class variables used to store persistent values between executions of callbacks and control loop
        self.state = State.FORWARD # Current FSM state
        self.pose = Pose() # Current pose (position and orientation), relative to the odom reference frame
        self.previous_pose = Pose() # Store a snapshot of the pose for comparison against future poses
        self.yaw = 0.0 # Angle the robot is facing (rotation around the Z axis, in radians), relative to the odom reference frame
        self.previous_yaw = 0.0 # Snapshot of the angle for comparison against future angles
        self.turn_angle = 0.0 # Relative angle to turn to in the TURNING state
        self.turn_direction = TURN_LEFT # Direction to turn in the TURNING state
        self.scan_triggered = [False] * 4 # Boolean value for each of the 4 LiDAR sensor sectors. True if obstacle detected within SCAN_THRESHOLD
        self.items = ItemList()
        
        # Creates a timer that calls the control_loop method repeatedly
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Initial pose parameters
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.initial_x = self.get_parameter('x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value
        
        self.is_holding_item = False
        
        # Navigator Stuff
        self.navigator = BasicNavigator()
        self.goal = PoseStamped()
        self.goal.header.frame_id = 'map'
        self.goal.header.stamp = self.get_clock().now().to_msg()
        self.goal.pose.position.x = self.initial_x
        self.goal.pose.position.y = self.initial_y
        self.goal.pose.orientation.z = 0.0
        self.goal.pose.orientation.w = 1.0
        self.navigator.setInitialPose(self.goal)
        

        self.item_subscriber = self.create_subscription(
            ItemList,
            'items',
            self.item_callback,
            10
        )

        # Subscribes to Odometry messages published on /odom topic
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        # Subscribes to LaserScan messages on the /scan topic
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value)
        
        # Subcribes to the Item holders topic
        self.item_holder_subscriber = self.create_subscription(
            ItemHolders,
            '/item_holders',
            self.item_holder_callback,
            10
        )

        # Publishes Twist messages (linear and angular velocities) on the /cmd_vel topic
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)


    def item_callback(self, msg):
        self.items = msg
        
    def item_holder_callback(self, msg):
        self.item_holder = msg
        try:
            # So each robot gets his own itemholder data
            for data in msg.data:
                if data.robot_id == self.robot_id:
                    self.is_holding_item = data.holding_item
        except:
            pass
        

    def odom_callback(self, msg):
        self.pose = msg.pose.pose # Store the pose in a class variable

        # Uses tf_transformations package to convert orientation from quaternion to Euler angles (RPY = roll, pitch, yaw)
        (roll, pitch, yaw) = euler_from_quaternion([self.pose.orientation.x,
                                                    self.pose.orientation.y,
                                                    self.pose.orientation.z,
                                                    self.pose.orientation.w])
        
        self.yaw = yaw # Store the yaw in a class variable

    def scan_callback(self, msg):
        # Group scan ranges into 4 segments
        front_ranges = msg.ranges[331:359] + msg.ranges[0:30]
        left_ranges  = msg.ranges[31:90]
        back_ranges  = msg.ranges[91:270]
        right_ranges = msg.ranges[271:330]

        # Store True/False values for each sensor segment, based on whether the nearest detected obstacle is closer than SCAN_THRESHOLD
        self.scan_triggered[SCAN_FRONT] = min(front_ranges) < SCAN_THRESHOLD 
        self.scan_triggered[SCAN_LEFT]  = min(left_ranges)  < SCAN_THRESHOLD
        self.scan_triggered[SCAN_BACK]  = min(back_ranges)  < SCAN_THRESHOLD
        self.scan_triggered[SCAN_RIGHT] = min(right_ranges) < SCAN_THRESHOLD

    # Control loop for the FSM - called periodically by self.timer
    def control_loop(self):

        # Logs State, only if it changed (avoid cluttering the log)
        if self.state != self.previous_state:
            self.get_logger().info(f"{self.state}")
            self.previous_state = self.state
        
        
        match self.state:

            case State.FORWARD:
                # If it's holding an item, it goes to returning state
                if self.is_holding_item:
                    self.state = State.RETURNING
                    return
                
                # Obstacle avoidance
                if self.scan_triggered[SCAN_FRONT]:
                    self.previous_yaw = self.yaw
                    self.state = State.TURNING
                    self.turn_angle = 45
                    self.turn_direction = TURN_RIGHT if self.scan_triggered[SCAN_LEFT] else TURN_LEFT
                    self.get_logger().info("Obstacle detected, turning " + ("right" if self.turn_direction == TURN_RIGHT else "left"))
                    msg = Twist()  # Stop the robot before turning
                    msg.linear.x = 0.0
                    self.cmd_vel_publisher.publish(msg)
                    return

                # If there's an item in sight, go to collecting state
                if len(self.items.data) > 0:
                    self.state = State.COLLECTING
                    return

                # Move forward
                msg = Twist()
                msg.linear.x = LINEAR_VELOCITY
                self.cmd_vel_publisher.publish(msg)

                difference_x = self.pose.position.x - self.previous_pose.position.x
                difference_y = self.pose.position.y - self.previous_pose.position.y
                distance_travelled = math.sqrt(difference_x ** 2 + difference_y ** 2)

                # No need for goal distance comparison, if its moved for 1 meter and no items were detected, it means there's no item in trajectory
                if distance_travelled >= 1:
                    self.previous_yaw = self.yaw
                    self.state = State.TURNING
                    self.turn_angle = random.uniform(45, 180)
                    self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
                    self.get_logger().info("No items in sight, turning " + ("left" if self.turn_direction == TURN_LEFT else "right") + f" by {self.turn_angle:.2f} degrees")
                return



            case State.TURNING:
                
                
                msg = Twist()
                msg.linear.x = 0.0  # Stop forward movement
                msg.angular.z = self.turn_direction * ANGULAR_VELOCITY
                self.cmd_vel_publisher.publish(msg)

                yaw_difference = angles.normalize_angle(self.yaw - self.previous_yaw)
                
                # If there's an item in sight, and it's not holding one, go to collecting state
                if len(self.items.data) > 0 and self.is_holding_item == False:
                    self.state = State.COLLECTING
                    return
                
                if math.fabs(yaw_difference) >= math.radians(self.turn_angle):
                    self.previous_pose = self.pose
                    self.state = State.FORWARD
                    self.get_logger().info(f"Finished turning, driving forward now.")
                return
            

            case State.COLLECTING:
                
                # It's holding an item, no need to stay in this state now.
                if self.is_holding_item:
                    self.state = State.RETURNING
                    return

                # Obstacle avoidance
                if self.scan_triggered[SCAN_FRONT]:
                    self.previous_yaw = self.yaw
                    self.state = State.TURNING
                    self.turn_angle = 45
                    self.turn_direction = TURN_RIGHT if self.scan_triggered[SCAN_LEFT] else TURN_LEFT
                    self.get_logger().info("Obstacle detected while collecting, turning " + ("right" if self.turn_direction == TURN_RIGHT else "left"))
                    msg = Twist()  # Stop the robot before turning
                    msg.linear.x = 0.0
                    self.cmd_vel_publisher.publish(msg)
                    return

                # No items / item right in front of it | go to FORWARD state
                if len(self.items.data) == 0:
                    self.state = State.FORWARD
                    return
                else:
                    # Filter items by color
                    blue_items = [item for item in self.items.data if item.colour == 'BLUE']
                    red_items = [item for item in self.items.data if item.colour == 'RED']
                    green_items = [item for item in self.items.data if item.colour == 'GREEN']

                # Go to the closest blue item (largest diameter)
                if blue_items:
                    item = max(blue_items, key=lambda x: x.diameter)
                    msg = Twist()
                    msg.linear.x = LINEAR_VELOCITY
                    msg.angular.z = item.x / 320.0
                    self.cmd_vel_publisher.publish(msg)
                # No blue, so go green    
                elif green_items:
                    item = max(green_items, key=lambda x: x.diameter)
                    msg = Twist()
                    msg.linear.x = LINEAR_VELOCITY
                    msg.angular.z = item.x / 320.0
                    self.cmd_vel_publisher.publish(msg)
                # No green, so go red
                elif red_items:
                    item = max(red_items, key=lambda x: x.diameter)
                    msg = Twist()
                    msg.linear.x = LINEAR_VELOCITY
                    msg.angular.z = item.x / 320.0
                    self.cmd_vel_publisher.publish(msg)
                # There are no items. but we filtered, its like a priority system that gets blue first, green after, red last
                else:        
                    self.get_logger().info("No items found, transitioning back to FORWARD state")
                    self.previous_pose = self.pose
                    self.state = State.FORWARD
                return


                
            case State.RETURNING:
                # Check if the robot is still holding an item
                if not self.is_holding_item:
                    # Cancel any ongoing navigation task
                    self.navigator.cancelTask()

                    # Log the completion of the item delivery
                    self.get_logger().info("Item delivered, stopping navigation")

                    # Change state to turning for a random rotation
                    self.previous_yaw = self.yaw
                    self.state = State.TURNING
                    self.turn_angle = random.uniform(180, 260)
                    return

                # Obstacle avoidance | Mostly so it doesnt collide against other robots
                if self.scan_triggered[SCAN_FRONT]:
                    # Cancel the current navigation task
                    self.navigator.cancelTask()

                    # Log the obstacle detection and initiate avoidance
                    self.get_logger().info("Obstacle detected while returning, executing avoidance maneuver")
                    
                    self.previous_yaw = self.yaw
                    self.state = State.TURNING
                    self.turn_angle = 45
                    self.turn_direction = TURN_RIGHT if self.scan_triggered[SCAN_LEFT] else TURN_LEFT
                    msg = Twist()  # Stop the robot before turning
                    msg.linear.x = 0.0
                    self.cmd_vel_publisher.publish(msg)
                    return

                # Continue navigation to the goal
                self.navigator.goToPose(self.goal)

                
            case _:
                pass

    def destroy_node(self):
        msg = Twist()
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f"Stopping: {msg}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    node = RobotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()