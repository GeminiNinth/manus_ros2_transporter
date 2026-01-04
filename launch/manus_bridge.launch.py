"""
Launch file for MANUS Glove ZMQ Bridge (Complete setup)

This launch file starts both the manus_data_publisher and zmq_bridge,
providing a complete server-side setup for sharing glove data over ZMQ.

Usage:
    # Start with default settings (both hands)
    ros2 launch manus_ros2_transporter manus_bridge.launch.py

    # Start with right hand only
    ros2 launch manus_ros2_transporter manus_bridge.launch.py side:=right

    # Custom ports
    ros2 launch manus_ros2_transporter manus_bridge.launch.py right_port:=9000 left_port:=9001

Prerequisites:
    - MANUS SDK and manus_ros2 package must be installed
    - See: https://docs.manus-meta.com/3.1.0/Plugins/SDK/ROS2/getting%20started/
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    side_arg = DeclareLaunchArgument(
        'side',
        default_value='both',
        description='Which hand side to bridge: left, right, or both'
    )
    
    right_port_arg = DeclareLaunchArgument(
        'right_port',
        default_value='8760',
        description='ZMQ port for right hand data'
    )
    
    left_port_arg = DeclareLaunchArgument(
        'left_port',
        default_value='8761',
        description='ZMQ port for left hand data'
    )
    
    bind_address_arg = DeclareLaunchArgument(
        'bind_address',
        default_value='0.0.0.0',
        description='Address to bind ZMQ sockets (0.0.0.0 for all interfaces)'
    )
    
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value='',
        description='Path to manus_devices.yaml config file (optional)'
    )
    
    # MANUS Data Publisher node (from manus_ros2 package)
    # Note: This requires manus_ros2 package to be installed
    # See: https://docs.manus-meta.com/3.1.0/Plugins/SDK/ROS2/getting%20started/
    manus_publisher_node = Node(
        package='manus_ros2_publisher',
        executable='manus_data_publisher',
        name='manus_data_publisher',
        output='screen',
    )
    
    # ZMQ Bridge node
    zmq_bridge_node = Node(
        package='manus_ros2_transporter',
        executable='zmq_bridge',
        name='manus_zmq_bridge',
        output='screen',
        parameters=[{
            'side': LaunchConfiguration('side'),
            'right_port': LaunchConfiguration('right_port'),
            'left_port': LaunchConfiguration('left_port'),
            'bind_address': LaunchConfiguration('bind_address'),
            'config_path': LaunchConfiguration('config_path'),
        }],
        # Disable Python stdout buffering for real-time log output
        additional_env={'PYTHONUNBUFFERED': '1'},
    )
    
    # Startup message
    startup_info = LogInfo(
        msg='Starting MANUS Glove ZMQ Bridge...\n'
            '  - manus_data_publisher: Publishes glove data to ROS topics\n'
            '  - zmq_bridge: Forwards glove data over ZMQ for remote PCs\n'
            '\n'
            'Ensure glove dongle and license key are connected.'
    )
    
    return LaunchDescription([
        side_arg,
        right_port_arg,
        left_port_arg,
        bind_address_arg,
        config_path_arg,
        startup_info,
        manus_publisher_node,
        zmq_bridge_node,
    ])

