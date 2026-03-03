#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DriveForward(Node):
    def __init__(self):
        super().__init__('drive_forward')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Parameters
        self.distance_m = 0.3       # 300 mm
        self.speed_mps = 0.05       # 50 mm/s

        # State
        self.distance_travelled = 0.0
        self.last_time = self.get_clock().now()

        # Timer
        self.timer_period = 0.05    # 20 Hz
        self.timer = self.create_timer(self.timer_period, self.drive)

        self.get_logger().info(f"Driving forward {self.distance_m*1000:.0f} mm")

    def drive(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        msg = Twist()

        if self.distance_travelled >= self.distance_m:
            self.stop_robot()
            self.get_logger().info("Reached target distance. Robot stopped.")
            return

        # Move straight
        msg.linear.x = self.speed_mps
        msg.angular.z = 0.0
        self.distance_travelled += self.speed_mps * dt

        self.cmd_pub.publish(msg)

    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_pub.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DriveForward()
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
