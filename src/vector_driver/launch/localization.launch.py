from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, Shutdown

def generate_launch_description():
    return LaunchDescription([
        # Static trasnform to fix base_footprint -> base_link for EKF from xacro file. 
        # Issue: it is offset in vector.xacro want it to look visually good in rviz
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='basefootprint_to_baselink',
            arguments=['-0.01','0','0.027','0','0','0', 'base_footprint', 'base_link']
        ),

        Node(
            package='vector_driver',
            executable='vector_node',
            name='vector_node',
            output='screen'
        ),

        Node(
            package='vector_driver',
            executable='marker_localization_node',
            name='marker_localization_node',
            output='screen'
        ),

        Node(
            package='vector_driver',
            executable='ekf_node',
            name='ekf_node',
            output='screen'
        ),

        # Node(
        #     package='vector_driver',
        #     executable='multi_topic_logger',
        #     name='multi_topic_logger',
        #     output='screen'
        # ),


        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_local',
        #     output='screen',
        #     parameters=['/home/mihir/ROS2_PROJ/vector_ws/src/vector_driver/config/ekf_local.yaml'],
        #     remappings=[('odometry/filtered', 'odometry/local')]
        # ),


        # Node(
        #     package='vector_driver',
        #     executable='test_logger_node',
        #     name='test_logger_node',
        #     output='screen',
        # ),

        # Node(
        #     package='vector_driver',
        #     executable='drive_square',
        #     name='drive_square',
        #     output='screen',
        # ),

        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_global',
        #     output='screen',
        #     parameters=['/home/mihir/ROS2_PROJ/vector_ws/src/vector_driver/config/ekf_global.yaml'],
        #     remappings=[('odometry/filtered', 'odometry/global')]
        # ),

        # Node(
        #     package='vector_driver',
        #     executable='multi_topic_logger',
        #     name='multi_topic_logger',
        #     output='screen'
        # ),

        # Shut down after 10 seconds 
        # TimerAction(
        #     period=30.0, #seconds
        #     actions=[Shutdown()]
        # )

    ])
