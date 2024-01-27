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

LINEAR_VELOCITY = 0.3
ANGULAR_VELOCITY = 0.5
STOP = 0.0
TURN_LEFT = 1
TURN_RIGHT = -1
SCAN_THRESHOLD = 0.5
SCAN_FRONT = 0
SCAN_LEFT = 1
SCAN_BACK = 2
SCAN_RIGHT = 3

class State(Enum):
    FORWARD = 0
    TURNING = 1
    CENTER = 2
    COLLECT = 3
    RETURN = 4
    
class RobotController(Node):
    
    def __init__(self):
        super().__init__('robot_controller')
        
        self.state = State.FORWARD
        
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        
        self.inital_x = self.get_parameter('x').get_parameter_value().double_value
        self.inital_y = self.get_parameter('y').get_parameter_value().double_value
        self.inital_yaw = self.get_parameter('yaw').get_parameter_value().double_value
        
        self.pose = Pose()
        self.prev_pose = Pose()
        self.yaw = 0.0
        self.prev_yaw = 0.0
        self.turn_angle = 0.0
        
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        self.counter = 0
        
        self.scan_triggered = [False] * 4
        self.turn_direction = TURN_LEFT
        self.items = ItemList()
        self.holders = ItemHolders()
        
        self.navigator = BasicNavigator()
        self.goal = PoseStamped()
        self.goal.header.frame_id = 'map'
        self.goal.header.stamp = self.get_clock().now().to_msg()
        self.goal.pose.position.x = self.inital_x
        self.goal.pose.position.y = self.inital_y
        self.goal.pose.orientation.z = 0.0
        self.goal.pose.orientation.w = 1.0
        self.navigator.setInitialPose(self.goal)
        # self.navigator.waitUntilNav2Active()
        
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )
        
        self.item_subscriber = self.create_subscription(
            ItemList,
            'items',
            self.item_callback,
            10
        )
        
        self.holders_subscriber = self.create_subscription(
            ItemHolders,
            '/item_holders',
            self.holders_callback,
            10
        )
        
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        def scan_callback(self, msg):
            front_ranges = msg.ranges[331:359] + msg.ranges[0:30]
            left_ranges  = msg.ranges[31:90]
            back_ranges  = msg.ranges[91:270]
            right_ranges = msg.ranges[271:330]
            
            self.scan_triggered[SCAN_FRONT] = min(front_ranges) < SCAN_THRESHOLD 
            self.scan_triggered[SCAN_LEFT]  = min(left_ranges)  < SCAN_THRESHOLD
            self.scan_triggered[SCAN_BACK]  = min(back_ranges)  < SCAN_THRESHOLD
            self.scan_triggered[SCAN_RIGHT] = min(right_ranges) < SCAN_THRESHOLD
        
        def item_callback(self, msg):
            self.items = msg
        
        def item_holder_callback(self, msg):
            self.holders = msg
            print(msg)

        def odom_callback(self, msg):
            self.pose = msg.pose.pose
            (roll, pitch, yaw) = euler_from_quaternion([self.pose.orientation.x,
                                                        self.pose.orientation.y,
                                                        self.pose.orientation.z,
                                                        self.pose.orientation.w])
            self.yaw = yaw
        
        
        def control_loop(self):
            self.get_logger().info(f"{self.state}")
            
            match self.state:
                
                case State.FORWARD:
                    if len(self.items.data) > 0:
                        self.state = State.CENTER
                        self.get_logger().info("CENTERING")
                        
                    if self.scan_triggered[SCAN_FRONT]:
                        self.prev_yaw = self.yaw
                        self.state = State.TURNING
                        self.turn_angle = random.uniform(150, 170)
                        self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
                        return
                    
                    if self.scan_triggered[SCAN_LEFT] or self.scan_triggered[SCAN_RIGHT]:
                        self.previous_yaw = self.yaw
                        # self.state = State.TURNING
                        self.turn_angle = 30
                        
                        if self.scan_triggered[SCAN_LEFT] and self.scan_triggered[SCAN_RIGHT]:
                            self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
                            
                        elif self.scan_triggered[SCAN_LEFT]:
                            self.turn_direction = TURN_RIGHT
                            
                        else: # self.scan_triggered[SCAN_RIGHT]
                            self.turn_direction = TURN_LEFT
                        return
                        
                    else:
                        msg = Twist()
                        msg.linear.x = LINEAR_VELOCITY
                        self.cmd_vel_publisher.publish(msg)
                        self.get_logger().info(f"FORWARD: {msg}")
                            
                case State.TURNING:
                    if len(self.items.data) > 0:
                        self.state = State.CENTER
                        self.get_logger().info("CENTERING")
                        
                    msg = Twist()
                    msg.angular.z = self.turn_direction * ANGULAR_VELOCITY
                    self.cmd_vel_publisher.publish(msg)
                    
                    yaw_difference = angles.normalize_angle(self.yaw - self.prev_yaw)
                    
                    if math.fabs(yaw_difference) >= math.radians(self.turn_angle):
                        self.prev_pose = self.pose
                        msg = Twist()
                        msg.linear.x = LINEAR_VELOCITY
                        self.cmd_vel_publisher.publish(msg)
                        self.state = State.FORWARD
                        self.get_logger().info(f"FORWARD: {msg}")
                        
                case State.CENTER:
                    if len(self.items.data) > 0:
                        if self.items.data[0].x < -75:
                            msg = Twist()
                            msg.angular.z = ANGULAR_VELOCITY * TURN_RIGHT
                            self.cmd_vel_publisher.publish(msg)
                            self.get_logger().info(f"CENTERING: {msg}")
                            
                        elif self.items.data[0].x < 75:
                            msg = Twist()
                            msg.angular.z = ANGULAR_VELOCITY * TURN_LEFT
                            self.cmd_vel_publisher.publish(msg)
                            self.get_logger().info(f"CENTERING: {msg}")
                            
                        else:
                            self.state = State.COLLECT
                    
                    else:
                        self.state = State.FORWARD
                        
                case State.COLLECT:
                    msg = Twist()
                    msg.linear.x = LINEAR_VELOCITY
                    self.cmd_vel_publisher.publish(msg)
                    
                    if len(self.holders.data) > 0:
                        if self.holders.data[0].holding_item == True:
                            self.state = State.RETURN
                
                case State.RETURN:
                    self.navigator.goToPose(self.goal)
                    
                    if not self.navigator.isTaskComplete():
                        feedback = self.navigator.getFeedback()
                        print('Estimated time of arrival: ' + '{0:.0f}'.format(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) + ' seconds')
                    else:
                        result = self.navigator.getResult()
                        
                        if result == TaskResult.SUCCEEDED:
                            print('Goal Succeeded!')
                            self.state = State.FORWARD
                        elif result == TaskResult.CANCELED:
                            print('Goal was canceled!')
                        elif result == TaskResult.FAILED:
                            print('Goal failed!')
                        else:
                            print('Goal has an invalid return status!')
                        
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