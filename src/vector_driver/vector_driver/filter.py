#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import math

from vector_driver.utils import wrap_angle_pi

class KalmanFilter(Node):
    def __init__(self):
        super().__init__('kalmanfilter')

        # QoS Profile
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Publish filterd pose for EKF 
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/camera/pose_filtered', camera_qos)

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odometry/local', 10, self.odom_callback)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/camera/pose_msg', self.pose_callback, camera_qos)


        # EKF State
        self.x_last = 0.0
        self.y_last = 0.0
        self.theta_last = math.pi / 2 # Facing forwards 

        # Covariances 

        self.P_last = np.diag([1.0, 1.0, np.deg2rad(2.0)**2]) # init uncertinaty 
        self.Q = np.diag([4.0, 4.0, np.deg2rad(8.0)**2])      # process noise 
        self.R = np.diag([1.0, 1.0, np.deg2rad(2.0)**2])      # camera measurement noise 

        self.H = np.eye(3)                                    # measurement matrix 
        self.mahalanobis_threshold = 20.0

        # Last_ODOM info
        self.last_odom_time = None 
        self.last_odom_pose = None 


    def odom_callback(self, msg: Odometry):
        
        v = msg.twist.twist.linear.x 
        omega = msg.twist.twist.angular.z

        # Time step 

        if self.last_odom_time is None:
            dt = 0.03 # 30 hz
        else:
            dt = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9) - self.last_odom_time 

        self.last_odom_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Predict 
        self.initial_predict(v, dt, omega)

    
    def initial_predict(self, velocity, dt, omega):
        
        theta_k = self.theta_last + omega *dt
        theta_k = wrap_angle_pi(theta_k)

        x_k = self.x_last + velocity * math.cos(theta_k) * dt
        y_k = self.y_last + velocity * math.sin(theta_k) * dt

        # Jacobian 
        F_k = np.array([
            [1, 0, -velocity * math.sin(theta_k) * dt],
            [0, 1, -velocity * math.cos(theta_k) * dt],
            [0, 0, 1]
        ])

        # Covariance prediction 
        self.P_last = F_k @ self.P_last @ F_k.T + self.Q

        # Update State 
        self.x_last = x_k 
        self.y_last = y_k 
        self.theta_last = theta_k

    
    def pose_callback(self, msg: PoseWithCovarianceStamped):

        x_est = np.array([self.x_last, self.y_last, self.theta_last])

        # Get reaidng 
        z_x = msg.pose.pose.position.x
        z_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y**2 + q.z**2)
        z_theta = math.atan2(siny_cosp, cosy_cosp)

        z_k = np.array([z_x, z_y, z_theta])

        # Innovation
        y_k = z_k - x_est
        y_k[2] = wrap_angle_pi(y_k[2])


        # Innovation covariance 
        S = self.H @ self.P_last @ self.H.T + self.R

        # Mahalanobis dist
        S_inv = np.linalg.inv(S)
        d2 = y_k.T @ S_inv @ y_k
        if d2 > self.mahalanobis_threshold:
            self.get_logger().warn(f'AprilTag outlier rejected, d2 = {d2:.2f}')
            return 
        
        # Kalman Gain 
        K = self.P_last @ self.H.T @ np.linalg.inv(S)

        # Update State
        x_updated = x_est + K @ y_k 
        self.P_last = (np.eye(3) - K @ self.H) @ self.P_last

        self.x_last = x_updated[0]
        self.y_last = x_updated[1]
        self.theta_last = wrap_angle_pi(x_updated[2])

        # Publish 
        filtered_msg = PoseWithCovarianceStamped()
        filtered_msg.header = msg.header
        filtered_msg.header.frame_id = 'map'

        filtered_msg.pose.pose.position.x = self.x_last 
        filtered_msg.pose.pose.position.y = self.y_last
        filtered_msg.pose.pose.position.z = 0.0

        # Yaw to quaternion
        filtered_msg.pose.pose.orientation.w = math.cos(self.theta_last/2)
        filtered_msg.pose.pose.orientation.x = 0.0
        filtered_msg.pose.pose.orientation.y = 0.0
        filtered_msg.pose.pose.orientation.z = math.sin(self.theta_last / 2)

        cov_filtered = [
            0.02, 0, 0, 0, 0, 0,
            0, 0.02, 0, 0, 0, 0,
            0, 0, 999, 0, 0, 0,
            0, 0, 0, 999, 0, 0,
            0, 0, 0, 0, 999, 0,
            0, 0, 0, 0, 0, 0.005
        ]

        filtered_msg.pose.covariance = cov_filtered 

        self.pose_pub.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
