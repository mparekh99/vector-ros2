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

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='map_to_odom',
        #     arguments=['0','0','0','0','0','0', 'map', 'odom']
        # ),


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


        Node(
            package='vector_driver',
            executable='multi_topic_logger',
            name='multi_topic_logger',
            output='screen'
        ),

        # Node(
        #     package='nav2_map_server',
        #     executable='map_server',
        #     name='map_server',
        #     output='screen',
        #     parameters=[{
        #         'yaml_filename':'/home/mihir/ROS2_PROJ/vector_ws/src/vector_driver/config/map.yaml'
        #     }]
        # ),
        
        # Node(
        #     package='nav2_planner',
        #     executable='planner_server',
        #     name='planner_server',
        #     output='screen',
        #     parameters=['/home/mihir/ROS2_PROJ/vector_ws/src/vector_driver/config/nav2.yaml']
        # ),

        # Node(
        #     package='nav2_controller',
        #     executable='controller_server',
        #     name='controller_server',
        #     output='screen',
        #     parameters=['/home/mihir/ROS2_PROJ/vector_ws/src/vector_driver/config/nav2.yaml']
        # ),

        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager_navigation',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': False, 
        #         'autostart': True, 
        #         'node_names': ['map_server', 'planner_server', 'controller_server']
        #     }]
        # ),


    ])
