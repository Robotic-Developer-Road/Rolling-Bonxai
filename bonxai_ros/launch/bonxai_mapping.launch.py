from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for BonxaiServer."""
    
    # Declare launch arguments
    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='bonxai_container',
        description='Name of the component container for BonxaiServer'
    )
    
    foxglove_port_arg = DeclareLaunchArgument(
        'foxglove_port',
        default_value='8765',
        description='Port for Foxglove Bridge WebSocket server'
    )
    
    # Get launch configuration
    container_name = LaunchConfiguration('container_name')
    foxglove_port = LaunchConfiguration('foxglove_port')
    
    # Parameters from bonxai_ros/params folder
    bonxai_params = PathJoinSubstitution([
        FindPackageShare('bonxai_ros'),
        'params',
        'bonxai_params.yaml'
    ])
    
    # Create BonxaiServer component
    bonxai_component = ComposableNode(
        package='bonxai_ros',
        plugin='Bonxai::BonxaiServer',
        name='bonxai_server_node',
        parameters=[bonxai_params],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    
    # Create container with BonxaiServer
    bonxai_container = ComposableNodeContainer(
        name=container_name,
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[bonxai_component],
        output='screen',
        emulate_tty=True,
    )
    
    # Foxglove Bridge (runs as separate node, not in container)
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[
            {'port': foxglove_port},
            {'address': '0.0.0.0'},
            {'send_buffer_limit': 10000000},
        ],
        output='screen',
    )
    
    return LaunchDescription([
        container_name_arg,
        foxglove_port_arg,
        bonxai_container,
        foxglove_bridge,
    ])