#!/usr/bin/env python3 


# REMOVE POSE With Covariance stamped and swap to posestamped 

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math
from tf_transformations import quaternion_from_euler
from scipy.spatial.transform import Rotation as R


from tf_transformations import euler_from_quaternion
from vector_driver.utils import wrap_angle_pi
from vector_driver.kalman import KalmanFilter


def quaternion_to_yaw(q: Quaternion) -> float:
    """
    Convert geometry_msgs/Quaternion to yaw (in radians)
    """
    euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return euler[2]  # yaw


class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_fusion_node')
        
        self.kf = KalmanFilter()
        
        self.last_odom_time = None


        # QoS Profile
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )


        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cam_sub = self.create_subscription(PoseWithCovarianceStamped, '/camera/pose_msg', self.cam_callback, qos_profile_sensor_data)


        # Publish filterd pose for EKF 
        self.cam_filter_pub = self.create_publisher(
            PoseStamped,
            '/camera/filtered',
            camera_qos
        )


        # Timer EKF Loop -- 100 hz 100 times a second
        self.timer = self.create_timer(0.01, self.filter_loop)

        self.tf_broadcaster = TransformBroadcaster(self)


    def odom_callback(self, msg: Odometry):
        t_now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_odom_time is None:
            self.last_odom_time = t_now
            return
        
        dt = t_now - self.last_odom_time
        self.last_odom_time = t_now

        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z

        # KALMAN  
        x, y, theta = self.kf.initial_predict(v, dt, omega)

        print(x,y,theta)

        # # Publish to TF

        # t = TransformStamped()
        # t.header.stamp = msg.header.stamp
        # t.header.frame_id = "map"
        # t.child_frame_id = "odom"

        # t.transform.translation.x = x
        # t.transform.translation.y = y
        # t.transform.translation.z = 0.0
        # q = quaternion_from_euler(0.0, 0.0, theta)
        # t.transform.rotation.x = q[0]
        # t.transform.rotation.y = q[1]
        # t.transform.rotation.z = q[2]
        # t.transform.rotation.w = q[3]

        # self.tf_broadcaster.sendTransform(t)



    

    # Update 
    def cam_callback(self, msg: PoseWithCovarianceStamped):
        # Get reaidng 
        x = msg.pose.pose.position.x 
        y = msg.pose.pose.position.y 
        q_msg = msg.pose.pose.orientation

        r = R.from_quat([q_msg.x, q_msg.y, q_msg.z, q_msg.w])
        R_mat = r.as_matrix()

        # theta = quaternion_to_yaw(q_msg)
        theta = wrap_angle_pi(math.atan2(-R_mat[0,0], R_mat[1,0]) + math.pi)

        # print(theta)

        x_f, y_f, theta_f, valid_reading = self.kf.update(x,y,theta)

        if valid_reading is not None:
            # Publis valid camera readings here
            pose_msg = PoseStamped()
            pose_msg.header.stamp = msg.header.stamp
            pose_msg.header.frame_id = 'map'

            pose_msg.pose.position.x = valid_reading[0]
            pose_msg.pose.position.y = valid_reading[1]
            pose_msg.pose.position.z = 0.0
            
            q = quaternion_from_euler(.0, .0, valid_reading[2])
            pose_msg.pose.orientation.x = q[0]
            pose_msg.pose.orientation.y = q[1]
            pose_msg.pose.orientation.z = q[2]
            pose_msg.pose.orientation.w = q[3]


            self.cam_filter_pub.publish(pose_msg)

            # print(valid_reading[0], valid_reading[1])


            # t = TransformStamped()

            # t.header.stamp = msg.header.stamp
            # t.header.frame_id = "map"
            # t.child_frame_id = "odom"

            # t.transform.translation.x = valid_reading[0]
            # t.transform.translation.y = valid_reading[1]
            # t.transform.translation.z = 0.0
            # t.transform.rotation.x = q[0]
            # t.transform.rotation.y = q[1]
            # t.transform.rotation.z = q[2]
            # t.transform.rotation.w = q[3]

            # self.tf_broadcaster.sendTransform(t)

        


        # # Publish 

        # t = TransformStamped()

        # t.header.stamp = msg.header.stamp
        # t.header.frame_id = "map"
        # t.child_frame_id = "odom"

        # t.transform.translation.x = x_f
        # t.transform.translation.y = y_f
        # t.transform.translation.z = 0.0
        # q = quaternion_from_euler(.0, .0, theta_f)
        # t.transform.rotation.x = q[0]
        # t.transform.rotation.y = q[1]
        # t.transform.rotation.z = q[2]
        # t.transform.rotation.w = q[3]

        # self.tf_broadcaster.sendTransform(t)


    def filter_loop(self):

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_footprint"

        # Convert to ROS Frame (ROS2 like x forward, y left) 
        # I currently did evyerthing with x right and y forward

    

        t.transform.translation.x = self.kf.x_last
        t.transform.translation.y = self.kf.y_last
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(.0, .0, self.kf.theta_last)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


        

def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
