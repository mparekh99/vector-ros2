from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess

def generate_launch_description():
    return LaunchDescription([

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
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=['/home/mihir/ROS2_PROJ/vector_ws/src/vector_driver/config/ekf_local.yaml'],
            remappings=[('odometry/filtered', 'odometry/local')]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=['/home/mihir/ROS2_PROJ/vector_ws/src/vector_driver/config/ekf_global.yaml'],
            remappings=[('odometry/filtered', 'odometry/global')]
        ),

    ])
