#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from vector_driver.utils import wrap_angle_pi
from tf_transformations import euler_from_quaternion
import math

def quaternion_to_yaw(q: Quaternion) -> float:
    """
    Convert geometry_msgs/Quaternion to yaw (in radians)
    """
    euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return euler[2]  # yaw

class EKFTestDriver(Node):
    def __init__(self):
        super().__init__('ekf_test_driver')

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Robot State
        self.x = None
        self.y = None 
        self.theta = None 

        # Square param 
        self.side_length = 0.1 # 300 mm 
        self.turn_angle = math.pi / 2 
        
        self.linear_speed = 0.05
        self.angular_speed = 0.7 

        # State machine 

        self.phase = "forward"
        self.current_side = 0
        
        self.start_x = None 
        self.start_y = None 
        self.start_theta = None 

        self.timer = self.create_timer(0.02, self.control_loop)


    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        self.theta = quaternion_to_yaw(q)

    def control_loop(self):

        if self.x is None: 
            return 
        
        msg = Twist()

        if self.current_side >= 4:
            self.cmd_pub.publish(Twist())
            self.get_logger().info("Square Complete")
            return

        if self.phase == "forward":
            
            if self.start_x is None:
                self.start_x = self.x 
                self.start_y = self.y 

            dist = math.sqrt(
                (self.x - self.start_x) ** 2 + 
                (self.y - self.start_y) ** 2
            )

            if dist >= self.side_length:
                self.phase = "turn"
                self.start_theta = self.theta
                self.start_x = None 
                self.start_y = None 
            else:
                msg.linear.x = self.linear_speed 
        
        elif self.phase == "turn":
            delta = wrap_angle_pi(self.theta - self.start_theta)

            if abs(delta) >= self.turn_angle:
                self.phase = "forward"
                self.current_side += 1

                self.start_theta = None
            else:
                msg.angular.z = self.angular_speed

        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EKFTestDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.cmd_pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
