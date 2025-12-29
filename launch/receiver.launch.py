"""
Launch file for ZMQ Receiver (Client side)

Usage:
    ros2 launch manus_ros2_transporter receiver.launch.py server_ip:=192.168.1.100 side:=left
    ros2 launch manus_ros2_transporter receiver.launch.py server_ip:=10.0.0.1 side:=right port:=8765
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    side_arg = DeclareLaunchArgument(
        'side',
        default_value='left',
        description='Which hand side to receive: left or right'
    )
    
    server_ip_arg = DeclareLaunchArgument(
        'server_ip',
        default_value='127.0.0.1',
        description='IP address of the ZMQ bridge server'
    )
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='0',
        description='ZMQ port to connect to (0 = auto-select based on side)'
    )
    
    topic_arg = DeclareLaunchArgument(
        'topic',
        default_value='/manus_glove_0',
        description='ROS topic to publish received data'
    )
    
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value='',
        description='Path to manus_devices.yaml config file (optional)'
    )
    
    reconnect_interval_arg = DeclareLaunchArgument(
        'reconnect_interval',
        default_value='1.0',
        description='Reconnection interval in seconds'
    )
    
    # ZMQ Receiver node
    zmq_receiver_node = Node(
        package='manus_ros2_transporter',
        executable='zmq_receiver',
        name='manus_zmq_receiver',
        output='screen',
        parameters=[{
            'side': LaunchConfiguration('side'),
            'server_ip': LaunchConfiguration('server_ip'),
            'port': LaunchConfiguration('port'),
            'topic': LaunchConfiguration('topic'),
            'config_path': LaunchConfiguration('config_path'),
            'reconnect_interval': LaunchConfiguration('reconnect_interval'),
        }],
    )
    
    return LaunchDescription([
        side_arg,
        server_ip_arg,
        port_arg,
        topic_arg,
        config_path_arg,
        reconnect_interval_arg,
        zmq_receiver_node,
    ])

