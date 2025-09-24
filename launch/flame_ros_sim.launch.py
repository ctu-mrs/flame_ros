from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from mrs_lib.remappings_custom_config_parser import RemappingsCustomConfigParser

def generate_launch_description():
    
    custom_config = DeclareLaunchArgument(
        'custom_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('flame_ros'),
            'config',
            'flame_component_sim.yaml'
        ]),
        description='Path to the FLAME configuration file'
    )
    
    node = ComposableNode(
        package='flame_ros',
        plugin='flame_ros::FlameRos',
        name='flame_ros',
        namespace='uav1',
        parameters=[{'use_sim_time': True},
                    LaunchConfiguration('custom_config'),
                    {'input': {'use_poseframe_updates': False}}],
        # ..
        extra_arguments=[{'use_intra_process_comms': True}],
        remappings=[('/uav1/image_raw', '/uav1/stereo/left/image_mono'),
                    ('/uav1/camera_info', '/uav1/stereo/left/camera_info'),
                    ('/uav1/odom', '/uav1/odomimu')]
    )
    
    parser = RemappingsCustomConfigParser(node, LaunchConfiguration('custom_config'))
    
    container = ComposableNodeContainer(
        name='flame_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        #prefix='xterm -e gdb -ex run --args',
        #prefix='gdb -ex run --args',
        composable_node_descriptions=[node]
    )
    
    return LaunchDescription([
        custom_config,
        parser,
        container,
    ])

