import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSPresetProfiles

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from assessment_interfaces.msg import ItemList, ItemHolders

from tf_transformations import euler_from_quaternion
import angles

from enum import Enum
import random
import math

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

        # Publishes custom StringWithPose messages on the /marker_input topic
        # self.marker_publisher = self.create_publisher(StringWithPose, '/marker_input', 10)

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

    def item_callback(self, msg):
        self.items = msg
        
    def item_holder_callback(self, msg):
        self.item_holder = msg

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
        self.get_logger().info(f"{self.items.data}")
        
        match self.state:

            case State.FORWARD:

                if self.scan_triggered[SCAN_FRONT]:
                    self.previous_yaw = self.yaw
                    self.state = State.TURNING
                    self.turn_angle = random.uniform(150, 170)
                    self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
                    self.get_logger().info("Detected obstacle in front, turning " + ("left" if self.turn_direction == TURN_LEFT else "right") + f" by {self.turn_angle:.2f} degrees")
                    return
                
                if self.scan_triggered[SCAN_LEFT] or self.scan_triggered[SCAN_RIGHT]:
                    self.previous_yaw = self.yaw
                    self.state = State.TURNING
                    self.turn_angle = 45

                    if self.scan_triggered[SCAN_LEFT] and self.scan_triggered[SCAN_RIGHT]:
                        self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
                        self.get_logger().info("Detected obstacle to both the left and right, turning " + ("left" if self.turn_direction == TURN_LEFT else "right") + f" by {self.turn_angle:.2f} degrees")
                    elif self.scan_triggered[SCAN_LEFT]:
                        self.turn_direction = TURN_RIGHT
                        self.get_logger().info(f"Detected obstacle to the left, turning right by {self.turn_angle} degrees")
                    else: # self.scan_triggered[SCAN_RIGHT]
                        self.turn_direction = TURN_LEFT
                        self.get_logger().info(f"Detected obstacle to the right, turning left by {self.turn_angle} degrees")
                    return
                
                if len(self.items.data) > 0 and self.item_holder.data[0].holding_item == False:
                    self.state = State.COLLECTING
                    return

                msg = Twist()
                msg.linear.x = LINEAR_VELOCITY
                self.cmd_vel_publisher.publish(msg)

                difference_x = self.pose.position.x - self.previous_pose.position.x
                difference_y = self.pose.position.y - self.previous_pose.position.y
                distance_travelled = math.sqrt(difference_x ** 2 + difference_y ** 2)

                # self.get_logger().info(f"Driven {distance_travelled:.2f} out of {self.goal_distance:.2f} metres")

                if distance_travelled >= self.goal_distance:
                    self.previous_yaw = self.yaw
                    self.state = State.TURNING
                    self.turn_angle = random.uniform(30, 150)
                    self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
                    self.get_logger().info("Goal reached, turning " + ("left" if self.turn_direction == TURN_LEFT else "right") + f" by {self.turn_angle:.2f} degrees")

            case State.TURNING:

                if len(self.items.data) > 0 and self.item_holder.data[0].holding_item == False:
                    self.state = State.COLLECTING
                    return

                msg = Twist()
                msg.angular.z = self.turn_direction * ANGULAR_VELOCITY
                self.cmd_vel_publisher.publish(msg)

                # self.get_logger().info(f"Turned {math.degrees(math.fabs(yaw_difference)):.2f} out of {self.turn_angle:.2f} degrees")

                yaw_difference = angles.normalize_angle(self.yaw - self.previous_yaw)                

                if math.fabs(yaw_difference) >= math.radians(self.turn_angle):
                    self.previous_pose = self.pose
                    self.goal_distance = random.uniform(1.0, 2.0)
                    self.state = State.FORWARD
                    self.get_logger().info(f"Finished turning, driving forward by {self.goal_distance:.2f} metres")

            case State.COLLECTING:

                if len(self.items.data) == 0:
                    self.previous_pose = self.pose
                    self.state = State.FORWARD
                    return
                
                item = self.items.data[0]

                estimated_distance = 69.0 * float(item.diameter) ** -0.89

                msg = Twist()
                msg.linear.x = 0.25 * estimated_distance
                msg.angular.z = item.x / 320.0

                self.cmd_vel_publisher.publish(msg)

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
