#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import csv
import os
from datetime import datetime
from tf_transformations import euler_from_quaternion
from rclpy.qos import qos_profile_sensor_data


def quaternion_to_yaw(q):
    euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return euler[2]


class MultiTopicLogger(Node):

    def __init__(self):
        super().__init__('multi_topic_logger')

        # Create timestamped CSV
        now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file_path = os.path.join(
            os.getcwd(),
            f'fusion_log_{now_str}.csv'
        )
        self.csv_file = open(self.csv_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Header
        self.csv_writer.writerow([
            'time',
            'topic',
            'x',
            'y',
            'yaw',
            'lin_vel',
            'ang_vel'
        ])

        # Subscribers
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/camera/pose_msg',
            self.camera_pose_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/camera/pose_filtered',
            self.camera_filtered_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Odometry,
            '/odometry/local',
            self.odom_local_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/odometry/global',
            self.odom_global_callback,
            10
        )

        self.get_logger().info(
            f"Logging started. Writing to {self.csv_file_path}"
        )

    # -------- Logging helpers --------

    def current_time(self):
        now = self.get_clock().now().to_msg()
        return now.sec + now.nanosec * 1e-9

    def log_pose_stamped(self, topic_name, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)

        # No velocity in PoseStamped
        lin_vel = 0.0
        ang_vel = 0.0

        self.csv_writer.writerow([
            self.current_time(),
            topic_name,
            x,
            y,
            yaw,
            lin_vel,
            ang_vel
        ])

    def log_odometry(self, topic_name, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)

        lin_vel = msg.twist.twist.linear.x
        ang_vel = msg.twist.twist.angular.z

        self.csv_writer.writerow([
            self.current_time(),
            topic_name,
            x,
            y,
            yaw,
            lin_vel,
            ang_vel
        ])

    # -------- Callbacks --------

    def camera_pose_callback(self, msg):
        self.log_pose_stamped('/camera/pose_msg', msg)

    def camera_filtered_callback(self, msg):
        self.log_pose_stamped('/camera/pose_filtered', msg)

    def odom_local_callback(self, msg):
        self.log_odometry('/odometry/local', msg)

    def odom_global_callback(self, msg):
        self.log_odometry('/odometry/global', msg)

    # -------- Cleanup --------

    def destroy_node(self):
        super().destroy_node()
        self.csv_file.close()
        self.get_logger().info(
            f"CSV file saved: {self.csv_file_path}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MultiTopicLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down logger...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
