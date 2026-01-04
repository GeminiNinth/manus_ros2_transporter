"""
ZMQ Bridge: ROS Topic to ZMQ Publisher

This node subscribes to Manus glove ROS topics and publishes them over ZMQ
for consumption by remote PCs without ROS networking.

Usage:
    ros2 run manus_ros2_transporter zmq_bridge
    ros2 run manus_ros2_transporter zmq_bridge --ros-args -p side:=both
    ros2 run manus_ros2_transporter zmq_bridge --ros-args -p config_path:=/path/to/config.yaml
"""

import re
import threading
from pathlib import Path
from typing import Dict, Optional, Set

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.subscription import Subscription
import zmq

from manus_ros2_msgs.msg import ManusGlove
from .common import (
    load_config,
    get_device_config,
    get_port_for_side,
    serialize_glove_msg,
    get_default_config_path,
)
from .logging_utils import (
    log_info,
    log_warn,
    log_error,
    log_glove_discovered,
    log_zmq_bound,
    log_startup_banner,
    log_shutdown,
)


class ZmqBridgeNode(Node):
    """
    ROS2 Node that bridges Manus glove data from ROS topics to ZMQ sockets.
    
    This allows remote PCs to receive glove data without using ROS networking,
    which is useful when ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST is set.
    """
    
    def __init__(self):
        super().__init__('manus_zmq_bridge')
        
        # Print startup banner
        log_startup_banner("ZMQ Bridge (Server)")
        
        # Declare parameters
        self.declare_parameter('side', 'both')  # 'left', 'right', or 'both'
        self.declare_parameter('config_path', '')
        self.declare_parameter('right_port', 8760)
        self.declare_parameter('left_port', 8761)
        self.declare_parameter('bind_address', '0.0.0.0')
        
        # Get parameters
        self.side_filter = self.get_parameter('side').get_parameter_value().string_value
        config_path_str = self.get_parameter('config_path').get_parameter_value().string_value
        self.right_port = self.get_parameter('right_port').get_parameter_value().integer_value
        self.left_port = self.get_parameter('left_port').get_parameter_value().integer_value
        self.bind_address = self.get_parameter('bind_address').get_parameter_value().string_value
        
        # Load configuration
        config_path = Path(config_path_str) if config_path_str else None
        try:
            self.config = load_config(config_path)
            log_info(f'Loaded config from: {config_path or get_default_config_path()}')
        except Exception as e:
            log_warn(f'Could not load config: {e}. Using defaults.')
            self.config = {}
        
        # Update ports from config if not explicitly set
        server_config = self.config.get('server', {})
        if not config_path_str:  # Only use config defaults if config_path not specified
            self.right_port = server_config.get('default_right_port', self.right_port)
            self.left_port = server_config.get('default_left_port', self.left_port)
            self.bind_address = server_config.get('bind_address', self.bind_address)
        
        # Initialize ZMQ context and sockets
        self.zmq_context = zmq.Context()
        self.zmq_sockets: Dict[str, zmq.Socket] = {}
        self.zmq_lock = threading.Lock()
        
        # Create sockets based on side filter
        if self.side_filter in ('both', 'right'):
            self._create_socket('right', self.right_port)
        if self.side_filter in ('both', 'left'):
            self._create_socket('left', self.left_port)
        
        # Track discovered gloves
        self.discovered_gloves: Set[int] = set()
        self.glove_side_map: Dict[int, str] = {}
        
        # QoS profile for subscriptions
        self._qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Dynamic topic subscription
        self._glove_subscriptions: Dict[str, Subscription] = {}
        self._subscribed_topics: Set[str] = set()
        self._topic_pattern = re.compile(r'^/manus_glove_\d+$')
        
        # Timer for dynamic topic discovery (check every 1 second)
        self._discovery_timer = self.create_timer(1.0, self._discover_topics)
        
        log_info(f'Waiting for glove topics (dynamic discovery)...')
        log_info(f'Side filter: {self.side_filter}')
    
    def _create_socket(self, side: str, port: int) -> None:
        """Create a ZMQ PUB socket for the given side"""
        socket = self.zmq_context.socket(zmq.PUB)
        bind_addr = f'tcp://{self.bind_address}:{port}'
        try:
            socket.bind(bind_addr)
            self.zmq_sockets[side] = socket
            log_zmq_bound(bind_addr, side)
        except zmq.ZMQError as e:
            log_error(f'Failed to bind ZMQ socket at {bind_addr}: {e}')
    
    def _discover_topics(self) -> None:
        """Periodically check for new manus_glove topics and subscribe to them"""
        # Get all available topics
        topic_names_and_types = self.get_topic_names_and_types()
        
        for topic_name, type_names in topic_names_and_types:
            # Check if it matches our pattern and hasn't been subscribed yet
            if (self._topic_pattern.match(topic_name) and 
                topic_name not in self._subscribed_topics and
                'manus_ros2_msgs/msg/ManusGlove' in type_names):
                
                self._subscribe_to_topic(topic_name)
    
    def _subscribe_to_topic(self, topic_name: str) -> None:
        """Subscribe to a manus_glove topic"""
        log_info(f'Discovered topic: {topic_name}')
        
        sub = self.create_subscription(
            ManusGlove,
            topic_name,
            lambda msg, topic=topic_name: self._glove_callback(msg, topic),
            self._qos
        )
        self._glove_subscriptions[topic_name] = sub
        self._subscribed_topics.add(topic_name)
    
    def _glove_callback(self, msg: ManusGlove, topic: str) -> None:
        """Handle incoming ManusGlove message"""
        glove_id = msg.glove_id
        side = msg.side.lower()
        
        # Log new glove discovery
        if glove_id not in self.discovered_gloves:
            self.discovered_gloves.add(glove_id)
            self.glove_side_map[glove_id] = side
            
            # Check if we have a config entry for this glove
            device_config = get_device_config(self.config, glove_id)
            alias = device_config.get('alias') if device_config else None
            log_glove_discovered(glove_id, side, alias)
            
            if not device_config:
                log_info(f'  (Glove not in config - add to manus_devices.yaml for custom settings)')
        
        # Check if this side should be published
        if self.side_filter != 'both' and side != self.side_filter:
            return
        
        # Get the socket for this side
        socket = self.zmq_sockets.get(side)
        if socket is None:
            return
        
        # Serialize and publish
        try:
            data = serialize_glove_msg(msg)
            with self.zmq_lock:
                socket.send(data, zmq.NOBLOCK)
        except zmq.ZMQError as e:
            log_warn(f'Failed to send ZMQ message: {e}')
        except Exception as e:
            log_error(f'Error serializing message: {e}')
    
    def destroy_node(self):
        """Cleanup ZMQ resources on shutdown"""
        log_shutdown()
        
        # Cancel discovery timer
        if self._discovery_timer:
            self._discovery_timer.cancel()
        
        # Close all sockets
        with self.zmq_lock:
            for side, socket in self.zmq_sockets.items():
                log_info(f'Closing {side} socket')
                socket.close()
            self.zmq_sockets.clear()
        
        # Terminate context
        self.zmq_context.term()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = ZmqBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
