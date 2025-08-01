from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('flame_ros'),
                'config',
                'flame_component.yaml'
            ]),
            description='Path to the FLAME configuration file'
        ),
        ComposableNodeContainer(
            name='flame_test',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            prefix='xterm -e gdb -ex run --args',
            composable_node_descriptions=[
                ComposableNode(
                    package='flame_ros',
                    plugin='flame_ros::FlameRos',
                    name='flame_ros',
                    parameters=["/flame/src/flame_ros/config/flame_component.yaml"],
                    # ..
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ]
        )
    ])

