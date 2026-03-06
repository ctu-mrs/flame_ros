from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from mrs_lib.remappings_custom_config_parser import RemappingsCustomConfigParser

from ament_index_python.packages import get_package_share_directory

from launch.substitutions import (
        LaunchConfiguration,
        IfElseSubstitution,
        PythonExpression,
        PathJoinSubstitution,
        EnvironmentVariable,
        )

import launch

import os
import sys

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "flame_ros"

    this_pkg_path = get_package_share_directory(pkg_name)

    # #{ uav_name

    uav_name = LaunchConfiguration('uav_name')

    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', "uav1"),
        description="The uav name used for namespacing.",
    ))

    # #} end of uav_name

    # #{ custom_config

    custom_config = LaunchConfiguration('custom_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value="",
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
        ))

    # behaviour:
    #     custom_config == "" => custom_config: ""
    #     custom_config == "/<path>" => custom_config: "/<path>"
    #     custom_config == "<path>" => custom_config: "$(pwd)/<path>"
    custom_config = IfElseSubstitution(
            condition=PythonExpression(['"', custom_config, '" != "" and ', 'not "', custom_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config]),
            else_value=custom_config
            )

    # #} end of custom_config

    # #{ calibration_file

    calibration_file = LaunchConfiguration('calibration_file')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'calibration_file',
        default_value=[this_pkg_path, "/config/calibration.yaml"],
        description="Path to the calibratin configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
        ))

    # behaviour:
    #     calibration_file == "" => calibration_file: ""
    #     calibration_file == "/<path>" => calibration_file: "/<path>"
    #     calibration_file == "<path>" => calibration_file: "$(pwd)/<path>"
    calibration_file = IfElseSubstitution(
            condition=PythonExpression(['"', calibration_file, '" != "" and ', 'not "', calibration_file, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), calibration_file]),
            else_value=calibration_file
            )

    # #} end of calibration_file

    # #{ camera

    camera_topic = LaunchConfiguration('camera_topic')

    ld.add_action(DeclareLaunchArgument(
        'camera_topic',
        default_value='rgb',
        description='topic of the camera that contains image_raw'
    ))

    # #} end of camera

    # #{ use_sim_time

    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=os.getenv('USE_SIM_TIME', "false"),
        description="Should the node subscribe to sim time?",
    ))

    # #} end of custom_config

    # #{ world_frame

    world_frame = LaunchConfiguration('world_frame')

    ld.add_action(DeclareLaunchArgument(
        'world_frame',
        default_value=[uav_name, "/local_origin"],
        description='The frame id of the world frame for the mapping.'
    ))

    # #} end of world_frame

    # #{ body_frame

    body_frame = LaunchConfiguration('body_frame')

    ld.add_action(DeclareLaunchArgument(
        'body_frame',
        default_value=[uav_name, "/fcu"],
        description='The UAV body frame'
    ))

    # #} end of world_frame

    # #{ camera_frame

    camera_frame = LaunchConfiguration('camera_frame')

    ld.add_action(DeclareLaunchArgument(
        'camera_frame',
        default_value="",
        description='If non-empty -> overrides camera frame'
    ))

    # #} end of world_frame

    # #{ path_frame

    path_frame = LaunchConfiguration('path_frame')

    ld.add_action(DeclareLaunchArgument(
        'path_frame',
        default_value="",
        description='If non-empty -> overrides path frame'
    ))

    # #} end of world_frame

    # #{ odom_topic

    odom_topic = LaunchConfiguration('odom_topic')

    ld.add_action(DeclareLaunchArgument(
        'odom_topic',
        default_value="~/odom_in",
        description='Odometry input topic'
    ))

    # #} end of world_frame

    # #{ path_topic

    path_topic = LaunchConfiguration('path_topic')

    ld.add_action(DeclareLaunchArgument(
        'path_topic',
        default_value="~/path_in",
        description='Path input topic'
    ))

    # #} end of world_frame

    node = ComposableNode(
        package='flame_ros',
        plugin='flame_ros::FlameRos',
        name='flame',
        namespace=uav_name,
        parameters=[
            {"uav_name": uav_name},
            {"world_frame": world_frame},
            {"body_frame": body_frame},
            {"camera_frame": camera_frame},
            {"path_frame": path_frame},
            {"use_sim_time": True},
            {"default_config": this_pkg_path + "/config/default.yaml"},
            {"calibration_file": calibration_file},
            {"custom_config": custom_config},
        ],
        remappings=[
            # subscribers
            ('~/image_in', [camera_topic, '/image_raw']),
            ('~/camera_info', [camera_topic, '/camera_info']),
            # publishers
            ('~/mesh_out', '~/mesh'),
            ('~/cloud_out', '~/cloud'),
            ('~/stats_out', '~/stats'),
            ('~/nodelet_stats_out', '~/nodelet_stats'),
            # image publishers
            ('~/debug/wireframe', '~/debug/wireframe'),
            ('~/debug/features', '~/debug/features'),
            ('~/debug/directions', '~/debug/directions'),
            ('~/debug/matches', '~/debug/matches'),
            ('~/debug/normals', '~/debug/normals'),
            ('~/debug/idepthmap', '~/debug/idepthmap'),
            ('~/idepth_registered/image_rect', '~/idepth_registered/image_rect'),
            ('~/depth_registered/image_rect', '~/depth_registered/image_rect'),
            ('~/depth_registered_raw/image_rect', '~/depth_registered_raw/image_rect'),
            ]
    )

    ld.add_action(RemappingsCustomConfigParser(node, custom_config))

    # #{ container

    container = ComposableNodeContainer(
        name='flame_container',
        namespace=uav_name,
        package='rclcpp_components',
        executable='component_container_mt',
        output="screen",
        # prefix=['debug_roslaunch ' + os.ttyname(sys.stdout.fileno())],
        composable_node_descriptions=[node],
        parameters=[
            {'use_intra_process_comms': True},
            {'thread_num': os.cpu_count()},
            {'use_sim_time': use_sim_time},
        ],
    )

    ld.add_action(container)

    # #} end of container

    return ld
