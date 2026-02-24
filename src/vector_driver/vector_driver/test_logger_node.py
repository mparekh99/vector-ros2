#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import csv
import os
import math
from tf_transformations import euler_from_quaternion
from datetime import datetime
from rclpy.qos import qos_profile_sensor_data

def quaternion_to_yaw(q: Quaternion) -> float:
    """
    Convert geometry_msgs/Quaternion to yaw (in radians)
    """
    euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return euler[2]  # yaw

class TestLogger(Node):
    def __init__(self):
        super().__init__('test_logger_node')

        # Prepare CSV file
        now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file_path = os.path.join(os.getcwd(), f'test_log_{now_str}.csv')
        self.csv_file = open(self.csv_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time', 'topic', 
            'x', 'y', 'yaw',
            'lin_vel', 'ang_vel'
        ])

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Odometry, '/odometry/local', self.odom_filtered_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, qos_profile_sensor_data)

        self.get_logger().info(f"TestLogger started, logging to {self.csv_file_path}")

    def log_odom(self, topic_name, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        lin_vel = msg.twist.twist.linear.x
        ang_vel = msg.twist.twist.angular.z
        t = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        self.csv_writer.writerow([t, topic_name, x, y, yaw, lin_vel, ang_vel])

    def log_imu(self, topic_name, msg: Imu):
        # Only use yaw and angular z for logging
        yaw = quaternion_to_yaw(msg.orientation)
        ang_vel = msg.angular_velocity.z
        # Positions unknown, set to zero
        x = 0.0
        y = 0.0
        t = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        self.csv_writer.writerow([t, topic_name, x, y, yaw, 0.0, ang_vel])

    # Callbacks
    def odom_callback(self, msg):
        self.log_odom('/odom', msg)

    def odom_filtered_callback(self, msg):
        self.log_odom('/odom_filtered', msg)

    def imu_callback(self, msg):
        self.log_imu('/imu/data', msg)

    def destroy_node(self):
        super().destroy_node()
        self.csv_file.close()
        self.get_logger().info(f"CSV file saved: {self.csv_file_path}")


def main(args=None):
    rclpy.init(args=args)
    node = TestLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down logger...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
