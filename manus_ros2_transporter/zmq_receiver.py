"""
ZMQ Receiver: ZMQ Subscriber to ROS Topic Publisher

This node receives Manus glove data over ZMQ from a remote bridge
and publishes it as a local ROS topic.

Usage:
    ros2 run manus_ros2_transporter zmq_receiver --ros-args -p side:=left -p server_ip:=192.168.1.100
    ros2 run manus_ros2_transporter zmq_receiver --ros-args -p side:=right -p server_ip:=10.0.0.1 -p port:=8765
"""

import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import zmq

from manus_ros2_msgs.msg import ManusGlove
from .common import (
    load_config,
    get_port_for_side,
    deserialize_glove_msg,
    get_default_config_path,
)
from .logging_utils import (
    log_info,
    log_warn,
    log_error,
    log_zmq_connected,
    log_zmq_disconnected,
    log_zmq_reconnecting,
    log_data_received,
    log_startup_banner,
    log_shutdown,
)


class ZmqReceiverNode(Node):
    """
    ROS2 Node that receives Manus glove data from ZMQ and publishes to ROS topics.
    
    This allows a remote PC to receive glove data without using ROS networking.
    The data is published locally for use by manus_allegro_teleop.py.
    """
    
    def __init__(self):
        super().__init__('manus_zmq_receiver')
        
        # Print startup banner
        log_startup_banner("ZMQ Receiver (Client)")
        
        # Declare parameters
        self.declare_parameter('side', 'left')  # 'left' or 'right'
        self.declare_parameter('server_ip', '127.0.0.1')
        self.declare_parameter('port', 0)  # 0 = auto-select based on side
        self.declare_parameter('topic', '/manus_glove_0')
        self.declare_parameter('config_path', '')
        self.declare_parameter('reconnect_interval', 1.0)
        
        # Get parameters
        self.side = self.get_parameter('side').get_parameter_value().string_value
        self.server_ip = self.get_parameter('server_ip').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        config_path_str = self.get_parameter('config_path').get_parameter_value().string_value
        self.reconnect_interval = self.get_parameter('reconnect_interval').get_parameter_value().double_value
        
        # Load configuration
        config_path = Path(config_path_str) if config_path_str else None
        try:
            self.config = load_config(config_path)
            log_info(f'Loaded config from: {config_path or get_default_config_path()}')
        except Exception as e:
            log_warn(f'Could not load config: {e}. Using defaults.')
            self.config = {}
        
        # Auto-select port based on side if not specified
        if self.port == 0:
            self.port = get_port_for_side(self.config, self.side)
        
        # QoS profile for publishing
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create publisher
        self.publisher = self.create_publisher(ManusGlove, self.topic, qos)
        
        # Initialize ZMQ
        self.zmq_context = zmq.Context()
        self.zmq_socket: Optional[zmq.Socket] = None
        self.zmq_connected = False
        self._connect_addr = f'tcp://{self.server_ip}:{self.port}'
        
        # Statistics
        self.msg_count = 0
        self.last_msg_time = 0.0
        self._last_status_count = 0
        
        # Start receiver thread
        self._running = True
        self._receiver_thread = threading.Thread(target=self._receiver_loop, daemon=True)
        self._receiver_thread.start()
        
        # Status timer
        self.create_timer(10.0, self._log_status)
        
        log_info(f'Receiving {self.side} hand data from {self._connect_addr}')
        log_info(f'Publishing to ROS topic: {self.topic}')
    
    def _connect(self) -> bool:
        """Connect to ZMQ server"""
        try:
            if self.zmq_socket is not None:
                self.zmq_socket.close()
            
            self.zmq_socket = self.zmq_context.socket(zmq.SUB)
            self.zmq_socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1 second timeout
            self.zmq_socket.setsockopt_string(zmq.SUBSCRIBE, '')  # Subscribe to all
            
            self.zmq_socket.connect(self._connect_addr)
            
            self.zmq_connected = True
            log_zmq_connected(self._connect_addr)
            return True
            
        except zmq.ZMQError as e:
            log_error(f'Failed to connect to ZMQ server: {e}')
            self.zmq_connected = False
            return False
    
    def _receiver_loop(self) -> None:
        """Background thread that receives ZMQ messages"""
        self._connect()
        
        while self._running:
            if not self.zmq_connected:
                time.sleep(self.reconnect_interval)
                log_zmq_reconnecting(self._connect_addr)
                self._connect()
                continue
            
            try:
                # Receive message (with timeout)
                data = self.zmq_socket.recv()
                
                # Deserialize
                msg = deserialize_glove_msg(data)
                
                # Check side filter
                if msg.side.lower() != self.side.lower():
                    continue
                
                # Publish to ROS
                self.publisher.publish(msg)
                
                # Update statistics
                self.msg_count += 1
                self.last_msg_time = time.time()
                
            except zmq.Again:
                # Timeout, no data received
                pass
            except zmq.ZMQError as e:
                log_warn(f'ZMQ error: {e}')
                self.zmq_connected = False
                log_zmq_disconnected()
            except Exception as e:
                log_error(f'Error processing message: {e}')
    
    def _log_status(self) -> None:
        """Log receiver status periodically"""
        if self.msg_count > 0:
            msgs_since_last = self.msg_count - self._last_status_count
            self._last_status_count = self.msg_count
            
            if msgs_since_last > 0:
                hz = msgs_since_last / 10.0  # 10 second interval
                log_data_received(self.side, msgs_since_last)
                log_info(f'  Average rate: {hz:.1f} Hz, Total: {self.msg_count}')
        else:
            if self.zmq_connected:
                log_info(f'Connected to {self._connect_addr}, waiting for {self.side} hand data...')
            else:
                log_warn(f'Not connected to {self._connect_addr}')
    
    def destroy_node(self):
        """Cleanup resources on shutdown"""
        log_shutdown()
        
        self._running = False
        self._receiver_thread.join(timeout=2.0)
        
        if self.zmq_socket is not None:
            self.zmq_socket.close()
        self.zmq_context.term()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = ZmqReceiverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
