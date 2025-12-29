"""
Launch file for ZMQ Bridge (Server side)

Usage:
    ros2 launch manus_ros2_transporter bridge.launch.py
    ros2 launch manus_ros2_transporter bridge.launch.py side:=right
    ros2 launch manus_ros2_transporter bridge.launch.py right_port:=8765 left_port:=8766
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
        default_value='8765',
        description='ZMQ port for right hand data'
    )
    
    left_port_arg = DeclareLaunchArgument(
        'left_port',
        default_value='8766',
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
    )
    
    return LaunchDescription([
        side_arg,
        right_port_arg,
        left_port_arg,
        bind_address_arg,
        config_path_arg,
        zmq_bridge_node,
    ])

