#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class EKFTestDriver(Node):
    def __init__(self):
        super().__init__('ekf_test_driver')

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Test parameters
        self.side_length_m = 0.3       # 300 mm per side
        self.linear_speed_mps = 0.05   # 50 mm/s
        self.turn_speed_rps = math.pi / 2  # 22.5 deg/s ~ 0.3927 rad/s

        # State
        self.phase = 0  # 0 = moving straight, 1 = turning
        self.distance_travelled = 0.0
        self.angle_turned = 0.0
        self.current_side = 0
        self.total_sides = 4
        self.last_time = self.get_clock().now()

        # Timer
        self.timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(self.timer_period, self.run_square)

        self.get_logger().info("EKF Test Driver started. Driving square 0.3m per side.")

    def run_square(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        msg = Twist()

        if self.current_side >= self.total_sides:
            # Finished square
            self.stop_robot()
            self.get_logger().info("Square complete. Robot stopped.")
            return

        if self.phase == 0:
            # Move straight
            self.distance_travelled += self.linear_speed_mps * dt
            if self.distance_travelled >= self.side_length_m:
                self.distance_travelled = 0.0
                self.phase = 1  # start turning
            else:
                msg.linear.x = self.linear_speed_mps
                msg.angular.z = 0.0

        elif self.phase == 1:
            # Turn left 90 degrees
            self.angle_turned += self.turn_speed_rps * dt
            if self.angle_turned >= math.pi:
                self.angle_turned = 0.0
                self.phase = 0  # start next side
                self.current_side += 1
            else:
                msg.linear.x = 0.0
                msg.angular.z = self.turn_speed_rps

        self.cmd_pub.publish(msg)

    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_pub.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EKFTestDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt detected. Stopping robot...")
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
