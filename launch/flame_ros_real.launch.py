from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'custom_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('flame_ros'),
                'config',
                'flame_component_real.yaml'
            ]),
            description='Path to the FLAME configuration file'
        ),
        ComposableNodeContainer(
            name='flame_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            #prefix='xterm -e gdb -ex run --args',
            #prefix='gdb -ex run --args',
            composable_node_descriptions=[
                ComposableNode(
                    package='flame_ros',
                    plugin='flame_ros::FlameRos',
                    name='flame_ros',
                    namespace='uav1',
                    parameters=[LaunchConfiguration("custom_config")],
                    # ..
                    extra_arguments=[{'use_intra_process_comms': True}],
                    remappings=[('/uav1/image_raw', '/uav1/image_rect')]
                ),
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0','0','0','0','0','0','cam0','mv_25003671']
        )
    ])

