from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

def generate_launch_description():
    
    container = ComposableNodeContainer(
        name='component_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[
            # NavigationServer
            ComposableNode(
                package='robot_nav',
                plugin='robot_nav::NavigationServer',
                name='navigation_server'
            ),
            # MoveToPoseClient
            ComposableNode(
                package='robot_nav',
                plugin='robot_nav::MoveToPoseClient',
                name='move_to_pose_client'
            )
        ]
    )

    return LaunchDescription([
        container
    ])