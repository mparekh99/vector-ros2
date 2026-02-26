#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

from geometry_msgs.msg import TransformStamped, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import CameraInfo, Image

from tf_transformations import quaternion_from_euler

# quat converter 
from scipy.spatial.transform import Rotation as R 
# Import Anki Vector Stuff

import anki_vector
from anki_vector import events
import math 
import csv
import os
from anki_vector.util import degrees
import cv2 
import numpy as np
from cv_bridge import CvBridge 
import threading 
import time
from vector_driver.utils import wrap_angle_pi
from vector_driver.world import Marker_World
# AprilTag 
from pupil_apriltags import Detector



class Marker_Localization_Node(Node):
    def __init__(self):
        super().__init__('marker_localization_node')

        # QoS Profile
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # CONSTANTS
        self.marker_length = 44 

        self.mtx = np.array([
            [365.25, 0.0, 319.43],
            [0.0, 368.12, 201.91],
            [0.0, 0.0, 1.0]
        ])

        self.dist = np.array([[0.33015985, -0.67084208, 0.6815479, -0.34481678]])

        # Publishers 
        # self.marker_raw_pub = self.create_publisher(PoseStamped, '/camera/marker_raw', camera_qos)
        # self.info_pub = self.create_publisher(CameraInfo, '/camera/info', camera_qos)
        self.pose_msg_pub = self.create_publisher(PoseWithCovarianceStamped, '/camera/pose_msg', camera_qos)

        # World INIT
        self.world = Marker_World()

        self.marker_world = Marker_World()

        # Subscriber 
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            camera_qos
        )

        self.bridge = CvBridge()

        # Detector 
        self.at_detector = Detector(
            families="tagStandard41h12",
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )


    def invert_homogeneous(self, T):
        R_mat = T[:3, :3]
        t = T[:3, 3]
        T_inv = np.eye(4)
        T_inv[:3, :3] = R_mat.T
        T_inv[:3, 3] = -R_mat.T @ t
        return T_inv

    def image_callback(self, msg: Image):
        
        # Convert ROS Image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        h, w = frame.shape[:2]

        # Undistort maps?
        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            self.mtx, self.dist, (w,h), np.eye(3), balance=0.0
        )

        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            self.mtx, self.dist, np.eye(3), new_K, (w,h), cv2.CV_16SC2
        )

        # undistort 
        undistorted = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)

        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)

        fx = new_K[0,0]
        fy = new_K[1,1]
        cx = new_K[0,2]
        cy = new_K[1,2]

        # Detect April Tags
        tags = self.at_detector.detect(
            gray, 
            estimate_tag_pose=True,
            camera_params=(fx,fy,cx,cy),
            tag_size=self.marker_length
        )

        if not tags:
            return
        
        pose_candidates = []

        for tag in tags:
            tag_id = tag.tag_id 
            if tag_id not in self.world.marker_transforms:
                self.get_logger().warn(f"Unknown tag ID {tag_id}")
                continue 

            # Build pose 
            marker_in_camera = np.eye(4)
            marker_in_camera[:3,:3] = tag.pose_R # rvec 
            marker_in_camera[:3, 3] = tag.pose_t.flatten() # tvec

            # Marker -> global
            # Frame Transformation
            marker = self.world.marker_transforms[tag.tag_id]

            fixed_pos, fixed_rot = marker["pos"], marker["rot"]

            marker_in_global = np.eye(4)
            marker_in_global[:3, :3] = fixed_rot
            marker_in_global[:3, 3] = fixed_pos

            # Camera -> global 
            camera_in_marker = self.invert_homogeneous(marker_in_camera)

            camera_in_global = marker_in_global @ camera_in_marker

            distance_to_marker = np.linalg.norm(camera_in_global[:3,3] - fixed_pos)
            pose_candidates.append((distance_to_marker, tag_id, camera_in_global))


        if not pose_candidates:
            return
        
        # Pick closest marker
        _, best_tag_id, best_camera_in_global = min(pose_candidates, key=lambda x:x[0])

        r = R.from_matrix(best_camera_in_global[:3,:3])
        q = r.as_quat() # x,y,z, w


        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.pose.position.x = best_camera_in_global[0,3] * 0.001 #CONVERT FROM MM to METERS!!!
        pose_msg.pose.pose.position.y = best_camera_in_global[1,3] * 0.001
        pose_msg.pose.pose.position.z = best_camera_in_global[2,3] * 0.001
        
        pose_msg.pose.pose.orientation.x = q[0]
        pose_msg.pose.pose.orientation.y = q[1]
        pose_msg.pose.pose.orientation.z = q[2]
        pose_msg.pose.pose.orientation.w = q[3]

        # Simple Covariance
        cov = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.01]

        pose_msg.pose.covariance = cov

        self.pose_msg_pub.publish(pose_msg)





def main(args=None):
    rclpy.init(args=args)
    node = Marker_Localization_Node()
    rclpy.spin(node)
    node.robot.disconnect()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
