"""
ZED2i Launch Wrapper for Bonxai

This launch file wraps the official ZED wrapper launch file and provides
simplified arguments for Bonxai integration.

Usage:
    ros2 launch bonxai_ros zed2i.launch.py
    ros2 launch bonxai_ros zed2i.launch.py container_name:=my_container
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description that wraps official ZED launch."""
    
    # Declare our arguments
    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='zed_container',
        description='Name of the component container for ZED2i'
    )
    
    enable_ipc_arg = DeclareLaunchArgument(
        'enable_ipc',
        default_value='true',
        description='Enable intra-process communication',
        choices=['true', 'false']
    )
    
    # Get launch configurations
    container_name = LaunchConfiguration('container_name')
    enable_ipc = LaunchConfiguration('enable_ipc')
    
    # Path to our custom params file
    zed_params_override = PathJoinSubstitution([
        FindPackageShare('bonxai_ros'),
        'params',
        'zed2i.yaml'
    ])
    
    # Include official ZED wrapper launch file
    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            # Fixed arguments for ZED2i
            'camera_model': 'zed2i',
            'camera_name': 'zed2i',
            
            # Container configuration
            'container_name': container_name,
            'enable_ipc': enable_ipc,
            
            # Override with our params
            'ros_params_override_path': zed_params_override,
            
            # Reasonable defaults
            'publish_urdf': 'true',
            'publish_tf': 'true',
            'publish_map_tf': 'false',  # Let your SLAM handle this
            'publish_imu_tf': 'false',
            
            # Live mode (not SVO playback)
            'svo_path': 'live',
            'publish_svo_clock': 'false',
            
            # No simulation
            'use_sim_time': 'false',
            'sim_mode': 'false',
            
        }.items()
    )
    
    return LaunchDescription([
        # Our arguments
        container_name_arg,
        enable_ipc_arg,
        
        # Include official launch
        zed_wrapper_launch,
    ])