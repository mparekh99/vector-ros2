#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
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

        # CSV Header
        self.csv_writer.writerow([
            'time',
            'topic',
            'x',
            'y',
            'yaw',
        ])

        # Subscribers
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/camera/pose_msg',
            self.camera_pose_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            PoseStamped,
            '/camera/filtered',
            self.camera_filtered_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # TF listener for map -> odom
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to log TF every 20 Hz
        self.tf_timer = self.create_timer(0.05, self.log_map_to_odom)

        self.get_logger().info(
            f"Logging started. Writing to {self.csv_file_path}"
        )

    # -------- Helpers --------

    def current_time(self):
        now = self.get_clock().now().to_msg()
        return now.sec + now.nanosec * 1e-9

    def log_pose_stamped(self, topic_name, msg):
        x = msg.pose.pose.position.x * 1000
        y = msg.pose.pose.position.y * 1000 
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)


        self.csv_writer.writerow([
            self.current_time(),
            topic_name,
            x,
            y,
            yaw,
        ])

    def log_odometry(self, topic_name, msg):
        x = msg.pose.pose.position.x * 1000
        y = msg.pose.pose.position.y * 1000
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)

        self.csv_writer.writerow([
            self.current_time(),
            topic_name,
            x,
            y,
            yaw,
        ])

    # -------- Callbacks --------

    def camera_pose_callback(self, msg):
        self.log_pose_stamped('/camera/pose_msg', msg)

    def camera_filtered_callback(self, msg):
        x = msg.pose.position.x * 1000
        y = msg.pose.position.y * 1000
        q = msg.pose.orientation
        yaw = quaternion_to_yaw(q)

        self.csv_writer.writerow([
            self.current_time(),
            '/camera/filtered',
            x,
            y,
            yaw,
        ])

    def odom_callback(self, msg):
        self.log_odometry('/odom', msg)

    # -------- TF Logging --------

    def log_map_to_odom(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'odom',
                Time()
            )

            t = transform.transform.translation
            r = transform.transform.rotation
            yaw = quaternion_to_yaw(r)

            self.csv_writer.writerow([
                self.current_time(),
                'tf_map_to_odom',
                t.x * 1000,
                t.y * 1000,
                yaw,
            ])

        except Exception:
            # TF not ready yet
            pass

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
