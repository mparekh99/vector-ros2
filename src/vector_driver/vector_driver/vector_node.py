#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup


from sensor_msgs.msg import Imu, Image, BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import JointState 
from tf_transformations import quaternion_from_euler


# quat converter 
from scipy.spatial.transform import Rotation as R 
# Import Anki Vector Stuff

import anki_vector
from anki_vector import events
from vector_driver.utils import wrap_angle_pi
import math 
import csv
import os
from anki_vector.util import degrees
import cv2 
import numpy as np
from cv_bridge import CvBridge 
import threading 
import time
from concurrent.futures import ThreadPoolExecutor




# Constants 
WHEELBASE = 0.05 # in meters 
# WHEELBASE = 0.06
# Config from launch file
LIFT_SCALE = -13.7931034483
LIFT_OFFSET = 0.4413793103
LIFT_MIN_M = 0.032
LIFT_MAX_M = 0.09

HEAD_SCALE = -1.0
HEAD_OFFSET = 0.0


MAX_WHEEL_SPEED_MMPS = 230


class VectorNode(Node):
    def __init__(self):
        super().__init__('vector_node')

        # Does all my sdk calls for me 
        self.sdk_executor = ThreadPoolExecutor(max_workers=4)
        self.cmd_lock = threading.Lock()

        self.callback_group = ReentrantCallbackGroup()

        # QoS Profile
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # ROS 2 Publishers 
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        # self.imu_pub = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        # self.batt_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.camera_pub = self.create_publisher(Image, '/camera/image_raw', camera_qos)

        # TF Broadcaster 
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # CV Bridge 
        self.bridge = CvBridge()

        # Connect to Vector 
        self.get_logger().info("Connecting to Vector...")
        self.robot = anki_vector.Robot("00706c20")
        self.robot.connect()

        # Intialize Camera and set head
        self.robot.behavior.set_head_angle(degrees(7.0))
        self.robot.behavior.set_lift_height(0.0)
        self.robot.camera.init_camera_feed()
        # Subscribe to camera events from anki
        self.robot.events.subscribe(
            self.on_new_camera_image,
            events.Events.new_raw_camera_image
        )

    

        # State variables for integration
        self.x = 0.0
        self.y = 0.0
        self.theta = math.pi / 2
        self.last_time = self.get_clock().now() # ROS2 time

        self.linear_mmps = 0.0
        self.angular_mmps = 0.0

        # # Timer to periodically publish
        # self.timer = self.create_timer(0.1, self.publish_sensors) # 10 Hz

        # Timers ------
        # 50 times per second
        # self.odom_timer = self.create_timer(0.02, self.publish_odom) # 50 Hz
        
        # # IMU
        # self.imu_timer = self.create_timer(0.01, self.publish_imu) # 100Hz

        # # Joint states 
        # self.joint_timer = self.create_timer(0.1, self.publish_joints) # 10 HZ

        # allows multiple timers to run concurrrently without blocking 
        self.odom_timer = self.create_timer(0.02, self.publish_odom, callback_group=self.callback_group)
        self.imu_timer = self.create_timer(0.01, self.publish_imu, callback_group=self.callback_group)
        self.joint_timer = self.create_timer(0.1, self.publish_joints, callback_group=self.callback_group)
    
        # Subscriber for velocity commands
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )




    def on_new_camera_image(self, robot, event_type, event, done=None):

        # So sdk callback never blocks main thread
        self.sdk_executor.submit(self._publish_camera, event.image)


    def _publish_camera(self, image):
        
        # Handle the image
        frame_np = np.array(image)
        frame_np = cv2.cvtColor(frame_np, cv2.COLOR_RGB2BGR)

        img_msg = self.bridge.cv2_to_imgmsg(frame_np, encoding='bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "camera_link"

        self.camera_pub.publish(img_msg)



    def cmd_vel_callback(self, msg:Twist):
        self.linear_mmps = msg.linear.x
        self.angular_mmps = msg.angular.z

        linear_mmps = self.linear_mmps
        angular_mmps = self.angular_mmps

        left_wheel = (linear_mmps - (angular_mmps * WHEELBASE / 2)) * 1000
        right_wheel = (linear_mmps + (angular_mmps * WHEELBASE / 2)) * 1000

        left_wheel = max(min(left_wheel, MAX_WHEEL_SPEED_MMPS), -MAX_WHEEL_SPEED_MMPS)
        right_wheel = max(min(right_wheel, MAX_WHEEL_SPEED_MMPS), -MAX_WHEEL_SPEED_MMPS)

        with self.cmd_lock:
            self.sdk_executor.submit(self.robot.motors.set_wheel_motors(left_wheel, right_wheel))


    def publish_odom(self):

        # GENERAL ODOMETRY I AM CALCULATING IN METERS NoT MM!!! 

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9 # convert to seconds
        self.last_time = now

        vLeft = self.robot.left_wheel_speed_mmps / 1000.0
        vRight = self.robot.right_wheel_speed_mmps / 1000.0

        v = (vRight + vLeft) / 2.0 # m/s 

        theta_k = self.theta + self.robot.gyro.z *dt
        theta_k = wrap_angle_pi(theta_k)

        self.x = self.x + v * math.cos(theta_k) * dt
        self.y = self.y + v * math.sin(theta_k) * dt
        self.theta = theta_k

        # CHANGE FOR ROS2 INTEGRATIONs

        x_ros = self.y
        y_ros = -1 * self.x
        theta_ros = wrap_angle_pi(self.theta - math.pi/2)


        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = x_ros
        odom_msg.pose.pose.position.y = y_ros
        odom_msg.pose.pose.position.z = 0.0
        q = quaternion_from_euler(.0, .0, theta_ros)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = self.robot.gyro.z

        odom_msg.twist.covariance = np.diag([1e-2,1e3, 1e3,1e3, 1e3, 1e-2]).ravel()

        # t = TransformStamped()
        # t.header.stamp = now.to_msg()
        # t.header.frame_id = 'odom'
        # t.child_frame_id = 'base_footprint'
        # t.transform.translation.x = x_ros
        # t.transform.translation.y = y_ros
        # t.transform.translation.z = 0.0
        # t.transform.rotation.x = q[0]
        # t.transform.rotation.y = q[1]
        # t.transform.rotation.z = q[2]
        # t.transform.rotation.w = q[3]
        # self.tf_broadcaster.sendTransform(t)


        #(vRight - vLeft) / WHEELBASE didn't give good values 
        

        self.odom_pub.publish(odom_msg)


    def publish_imu(self):

        now = self.get_clock().now()
        # ------IMU-----
        imu_msg = Imu()
        imu_msg.header.stamp = now.to_msg()
        imu_msg.header.frame_id = 'imu'

        imu_msg.orientation.w = self.robot.pose.rotation.q0
        imu_msg.orientation.x = self.robot.pose.rotation.q1
        imu_msg.orientation.y = self.robot.pose.rotation.q2
        imu_msg.orientation.z = self.robot.pose.rotation.q3

        # -- GYRO 
        imu_msg.angular_velocity.x = self.robot.gyro.x
        imu_msg.angular_velocity.y = self.robot.gyro.y
        imu_msg.angular_velocity.z = self.robot.gyro.z 
        
        # -- ACCEL
        imu_msg.linear_acceleration.x = self.robot.accel.x * 0.001 # from mm/sec^2 to m/sec^2
        imu_msg.linear_acceleration.y = self.robot.accel.y * 0.001  
        imu_msg.linear_acceleration.z = self.robot.accel.z * 0.001
        
        self.imu_pub.publish(imu_msg)

    def publish_joints(self):

        now = self.get_clock().now()

        # ---- JointStates ----
        lift_m = self.robot.lift_height_mm / 1000.0
        lift_angle = lift_m * LIFT_SCALE + LIFT_OFFSET
        # clamp angle 
        lift_angle = max(
            min(lift_angle, LIFT_MAX_M * LIFT_SCALE + LIFT_OFFSET), 
            LIFT_MIN_M * LIFT_SCALE + LIFT_OFFSET
        )

        head_angle = self.robot.head_angle_rad * HEAD_SCALE + HEAD_OFFSET

        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = ['base_to_head', 'base_to_lift']
        js.position = [head_angle, lift_angle]
        js.header.frame_id = 'base_link'
        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = VectorNode()
    rclpy.spin(node)
    node.robot.disconnect()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
