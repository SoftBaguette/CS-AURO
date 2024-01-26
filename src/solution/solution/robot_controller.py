import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration
from rclpy.qos import QoSPresetProfiles

from assessment_interfaces.msg import ItemList, ItemHolders
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, PoseStamped
from sensor_msgs.msg import LaserScan
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

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
        
        # Class variables used to store persistent values between executions of callbacks and control loop
        self.state = State.FORWARD # Current FSM state
        self.pose = Pose() # Current pose (position and orientation), relative to the odom reference frame
        self.previous_pose = Pose() # Store a snapshot of the pose for comparison against future poses
        self.yaw = 0.0 # Angle the robot is facing (rotation around the Z axis, in radians), relative to the odom reference frame
        self.previous_yaw = 0.0 # Snapshot of the angle for comparison against future angles
        self.turn_angle = 0.0 # Relative angle to turn to in the TURNING state
        self.turn_direction = TURN_LEFT # Direction to turn in the TURNING state
        self.goal_distance = random.uniform(1.0, 2.0) # Goal distance to travel in FORWARD state
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

        # Publishes custom StringWithPose messages on the /marker_input topic NEVER AGAIN
        # self.marker_publisher = self.create_publisher(StringWithPose, '/marker_input', 10)


    def item_callback(self, msg):
        self.items = msg
        
    def item_holder_callback(self, msg):
        self.item_holder = msg
        try:
            self.is_holding_item = msg.data[0].holding_item
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

        self.get_logger().info(f"{self.state}")
        # self.get_logger().info(f"{self.item_holder.data}")
        # self.get_logger().info(f"{self.items.data}")
        
        match self.state:

            case State.FORWARD:
                # Check if holding item
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

                # Check for items and transition to collecting if found
                if len(self.items.data) > 0:
                    self.state = State.COLLECTING
                    return

                # Move forward by goal distance
                msg = Twist()
                msg.linear.x = LINEAR_VELOCITY
                self.cmd_vel_publisher.publish(msg)

                difference_x = self.pose.position.x - self.previous_pose.position.x
                difference_y = self.pose.position.y - self.previous_pose.position.y
                distance_travelled = math.sqrt(difference_x ** 2 + difference_y ** 2)

                if distance_travelled >= self.goal_distance:
                    self.previous_yaw = self.yaw
                    self.state = State.TURNING
                    self.turn_angle = random.uniform(45, 180)
                    self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
                    self.get_logger().info("Goal reached, turning " + ("left" if self.turn_direction == TURN_LEFT else "right") + f" by {self.turn_angle:.2f} degrees")
                return



            case State.TURNING:
                msg = Twist()
                msg.linear.x = 0.0  # Stop forward movement
                msg.angular.z = self.turn_direction * ANGULAR_VELOCITY
                self.cmd_vel_publisher.publish(msg)

                yaw_difference = angles.normalize_angle(self.yaw - self.previous_yaw)
                
                # DETECTED ITEM WHILE TURNING AND ITS NOT HOLDING ONE, GO FETCH IT DOGGY
                if len(self.items.data) > 0 and self.is_holding_item == False:
                    self.state = State.COLLECTING
                    return
                
                if math.fabs(yaw_difference) >= math.radians(self.turn_angle):
                    self.previous_pose = self.pose
                    self.goal_distance = random.uniform(1.0, 2.0)
                    self.state = State.FORWARD
                    self.get_logger().info(f"Finished turning, driving forward by {self.goal_distance:.2f} metres")
                return
            
            

            case State.COLLECTING:
                
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
                self.navigator.goToPose(self.goal)

                if not self.is_holding_item:
                    self.navigator.cancelTask()
                    self.previous_yaw = self.yaw
                    self.state = State.TURNING
                    self.turn_angle = random.uniform(180, 260)
                    self.get_logger().info(f"Item delivered, turning randomly by {self.turn_angle:.2f} degrees")
                    return

                # Optionally, you can add logic here to check if the task is complete
                # and handle different outcomes (succeeded, failed, etc.)

                return

                
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
