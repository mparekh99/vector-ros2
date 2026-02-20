from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
          # Lifecycle manager to autostart Nav2 nodes
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': True, 
                            'autostart': True,
                            'node_names':[
                                'map_server',
                                'planner_server',
                                'controller_server',
                                'bt_navigator',
                                'behavior_server',
                                'acml'
                            ]}]
        ),
        # Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=['/home/mihir/ROS2_PROJ/vector_ws/src/vector_driver/params/vector_nav2_params.yaml']
        ),

        # ACML for localization 
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=['/home/mihir/ROS2_PROJ/vector_ws/src/vector_driver/params/vector_nav2_params.yaml']
        ),
        # Planner server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=['/home/mihir/ROS2_PROJ/vector_ws/src/vector_driver/params/vector_nav2_params.yaml']
        ),
        # Controller server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=['/home/mihir/ROS2_PROJ/vector_ws/src/vector_driver/params/vector_nav2_params.yaml']
        ),

        # BT navigator 
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=['/home/mihir/ROS2_PROJ/vector_ws/src/vector_driver/params/vector_nav2_params.yaml']
        ),    


        # Behaviors 
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=['/home/mihir/ROS2_PROJ/vector_ws/src/vector_driver/params/vector_nav2_params.yaml']
        ),
    ])
 
