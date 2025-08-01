#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('flame_ros'),
            'config',
            'flame_params.yaml'
        ]),
        description='Path to the FLAME configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error, fatal)'
    )
    
    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='flame_container',
        description='Name of the component container'
    )

    # Get launch configurations
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    container_name = LaunchConfiguration('container_name')

    # Define the FLAME composable node
    flame_node = ComposableNode(
        package='flame_ros',
        plugin='flame_ros::FlameRos',
        name='flame',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
        remappings=[
            # Add any topic remappings here if needed
            # ('input_topic', 'remapped_input_topic'),
        ]
    )

    # Create component container
    container = ComposableNodeContainer(
        name=container_name,
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[flame_node],
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen'
    )

    # Alternative: Load into existing container (uncomment if needed)
    # load_nodes = LoadComposableNodes(
    #     target_container=container_name,
    #     composable_node_descriptions=[flame_node]
    # )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        log_level_arg,
        container_name_arg,
        container,
        # load_nodes,  # Uncomment if using existing container
    ])