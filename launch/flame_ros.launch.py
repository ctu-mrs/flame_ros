from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
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
                    # ..
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ]
        )
    ])

