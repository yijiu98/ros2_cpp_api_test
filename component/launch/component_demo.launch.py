import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='container_component_demo',
            namespace='',
            package='rclcpp_components',

            executable='component_container',#单线程的
            # component_container_isolated#多线程的
            
            composable_node_descriptions=[
                ComposableNode(
                    package='component',
                    plugin='ne::ComponentDemoPub',
                    name='ComponentDemoPub',
                    extra_arguments=[{'use_intra_process_comms': True}] # 使用进程内通讯
                ),
                ComposableNode(
                    package='component',
                    plugin='ne::ComponentDemoSub',
                    name='ComponentDemoSub',
                    extra_arguments=[{'use_intra_process_comms': True}] # 使用进程内通讯
                )
            ],
            output='both',
    )

    return launch.LaunchDescription([container])

