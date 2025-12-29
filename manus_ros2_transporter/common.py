"""
Common utilities for manus_ros2_transporter

Provides serialization/deserialization of ManusGlove messages and configuration loading.
"""

import json
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import yaml
from geometry_msgs.msg import Pose, Quaternion
from manus_ros2_msgs.msg import ManusErgonomics, ManusGlove, ManusRawNode


def get_default_config_path() -> Path:
    """Get the default path to manus_devices.yaml"""
    # Try to find in ROS2 share directory first
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory('manus_ros2_transporter')
        config_path = Path(share_dir) / 'config' / 'manus_devices.yaml'
        if config_path.exists():
            return config_path
    except Exception:
        pass
    
    # Fallback: look relative to this file (source directory)
    current_dir = Path(__file__).parent.parent
    config_path = current_dir / 'config' / 'manus_devices.yaml'
    if config_path.exists():
        return config_path
    
    # If still not found, raise an error with helpful message
    raise FileNotFoundError(
        "manus_devices.yaml not found. Please ensure the package is built "
        "with 'colcon build --packages-select manus_ros2_transporter' "
        "and sourced with 'source install/setup.bash'"
    )


def load_config(config_path: Optional[Path] = None) -> Dict[str, Any]:
    """Load configuration from YAML file"""
    if config_path is None:
        config_path = get_default_config_path()
    
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def get_device_config(config: Dict[str, Any], glove_id: int) -> Optional[Dict[str, Any]]:
    """Get device configuration by glove ID"""
    devices = config.get('devices', [])
    for device in devices:
        if device.get('glove_id') == glove_id:
            return device
    return None


def get_port_for_side(config: Dict[str, Any], side: str) -> int:
    """Get the ZMQ port for a given side (left/right)"""
    server_config = config.get('server', {})
    if side.lower() == 'left':
        return server_config.get('default_left_port', 8766)
    else:
        return server_config.get('default_right_port', 8765)


def serialize_pose(pose: Pose) -> Dict[str, Any]:
    """Serialize a geometry_msgs/Pose to a dictionary"""
    return {
        'position': {
            'x': pose.position.x,
            'y': pose.position.y,
            'z': pose.position.z,
        },
        'orientation': {
            'x': pose.orientation.x,
            'y': pose.orientation.y,
            'z': pose.orientation.z,
            'w': pose.orientation.w,
        }
    }


def deserialize_pose(data: Dict[str, Any]) -> Pose:
    """Deserialize a dictionary to geometry_msgs/Pose"""
    pose = Pose()
    pos = data.get('position', {})
    pose.position.x = float(pos.get('x', 0.0))
    pose.position.y = float(pos.get('y', 0.0))
    pose.position.z = float(pos.get('z', 0.0))
    
    ori = data.get('orientation', {})
    pose.orientation.x = float(ori.get('x', 0.0))
    pose.orientation.y = float(ori.get('y', 0.0))
    pose.orientation.z = float(ori.get('z', 0.0))
    pose.orientation.w = float(ori.get('w', 1.0))
    
    return pose


def serialize_quaternion(quat: Quaternion) -> Dict[str, float]:
    """Serialize a geometry_msgs/Quaternion to a dictionary"""
    return {
        'x': quat.x,
        'y': quat.y,
        'z': quat.z,
        'w': quat.w,
    }


def deserialize_quaternion(data: Dict[str, Any]) -> Quaternion:
    """Deserialize a dictionary to geometry_msgs/Quaternion"""
    quat = Quaternion()
    quat.x = float(data.get('x', 0.0))
    quat.y = float(data.get('y', 0.0))
    quat.z = float(data.get('z', 0.0))
    quat.w = float(data.get('w', 1.0))
    return quat


def serialize_raw_node(node: ManusRawNode) -> Dict[str, Any]:
    """Serialize a ManusRawNode to a dictionary"""
    return {
        'node_id': node.node_id,
        'parent_node_id': node.parent_node_id,
        'joint_type': node.joint_type,
        'chain_type': node.chain_type,
        'pose': serialize_pose(node.pose),
    }


def deserialize_raw_node(data: Dict[str, Any]) -> ManusRawNode:
    """Deserialize a dictionary to ManusRawNode"""
    node = ManusRawNode()
    node.node_id = int(data.get('node_id', 0))
    node.parent_node_id = int(data.get('parent_node_id', 0))
    node.joint_type = str(data.get('joint_type', ''))
    node.chain_type = str(data.get('chain_type', ''))
    node.pose = deserialize_pose(data.get('pose', {}))
    return node


def serialize_ergonomics(ergo: ManusErgonomics) -> Dict[str, Any]:
    """Serialize a ManusErgonomics to a dictionary"""
    return {
        'type': ergo.type,
        'value': ergo.value,
    }


def deserialize_ergonomics(data: Dict[str, Any]) -> ManusErgonomics:
    """Deserialize a dictionary to ManusErgonomics"""
    ergo = ManusErgonomics()
    ergo.type = str(data.get('type', ''))
    ergo.value = float(data.get('value', 0.0))
    return ergo


def serialize_glove_msg(msg: ManusGlove) -> bytes:
    """
    Serialize a ManusGlove message to bytes (JSON format).
    
    Returns:
        bytes: JSON-encoded message data
    """
    data = {
        'glove_id': msg.glove_id,
        'side': msg.side,
        'raw_node_count': msg.raw_node_count,
        'raw_nodes': [serialize_raw_node(node) for node in msg.raw_nodes],
        'ergonomics_count': msg.ergonomics_count,
        'ergonomics': [serialize_ergonomics(ergo) for ergo in msg.ergonomics],
        'raw_sensor_orientation': serialize_quaternion(msg.raw_sensor_orientation),
        'raw_sensor_count': msg.raw_sensor_count,
        'raw_sensor': [serialize_pose(pose) for pose in msg.raw_sensor],
    }
    return json.dumps(data).encode('utf-8')


def deserialize_glove_msg(data: bytes) -> ManusGlove:
    """
    Deserialize bytes to a ManusGlove message.
    
    Args:
        data: JSON-encoded message data
        
    Returns:
        ManusGlove: Deserialized message
    """
    parsed = json.loads(data.decode('utf-8'))
    
    msg = ManusGlove()
    msg.glove_id = int(parsed.get('glove_id', 0))
    msg.side = str(parsed.get('side', ''))
    msg.raw_node_count = int(parsed.get('raw_node_count', 0))
    msg.raw_nodes = [deserialize_raw_node(n) for n in parsed.get('raw_nodes', [])]
    msg.ergonomics_count = int(parsed.get('ergonomics_count', 0))
    msg.ergonomics = [deserialize_ergonomics(e) for e in parsed.get('ergonomics', [])]
    msg.raw_sensor_orientation = deserialize_quaternion(parsed.get('raw_sensor_orientation', {}))
    msg.raw_sensor_count = int(parsed.get('raw_sensor_count', 0))
    msg.raw_sensor = [deserialize_pose(p) for p in parsed.get('raw_sensor', [])]
    
    return msg


def serialize_glove_msg_compact(msg: ManusGlove) -> bytes:
    """
    Serialize ManusGlove message to a compact binary format.
    
    This format is optimized for low latency by only sending essential data:
    - glove_id (4 bytes, int32)
    - side (1 byte: 0=unknown, 1=left, 2=right)
    - raw_sensor_count (4 bytes, int32)
    - raw_sensor positions (N * 3 * 4 bytes, float32 each)
    
    Returns:
        bytes: Binary encoded message data
    """
    # Encode side as single byte
    side_byte = 0
    if msg.side.lower() == 'left':
        side_byte = 1
    elif msg.side.lower() == 'right':
        side_byte = 2
    
    # Build header
    header = np.array([msg.glove_id], dtype=np.int32).tobytes()
    header += bytes([side_byte])
    header += np.array([msg.raw_sensor_count], dtype=np.int32).tobytes()
    
    # Build position data
    positions = np.zeros((msg.raw_sensor_count, 3), dtype=np.float32)
    for i, pose in enumerate(msg.raw_sensor[:msg.raw_sensor_count]):
        positions[i, 0] = pose.position.x
        positions[i, 1] = pose.position.y
        positions[i, 2] = pose.position.z
    
    return header + positions.tobytes()


def deserialize_glove_msg_compact(data: bytes) -> ManusGlove:
    """
    Deserialize compact binary format to ManusGlove message.
    
    Args:
        data: Binary encoded message data
        
    Returns:
        ManusGlove: Deserialized message (only raw_sensor positions filled)
    """
    msg = ManusGlove()
    
    # Parse header
    msg.glove_id = int(np.frombuffer(data[0:4], dtype=np.int32)[0])
    
    side_byte = data[4]
    if side_byte == 1:
        msg.side = 'left'
    elif side_byte == 2:
        msg.side = 'right'
    else:
        msg.side = 'unknown'
    
    msg.raw_sensor_count = int(np.frombuffer(data[5:9], dtype=np.int32)[0])
    
    # Parse positions
    positions = np.frombuffer(data[9:], dtype=np.float32).reshape(-1, 3)
    
    msg.raw_sensor = []
    for i in range(min(msg.raw_sensor_count, len(positions))):
        pose = Pose()
        pose.position.x = float(positions[i, 0])
        pose.position.y = float(positions[i, 1])
        pose.position.z = float(positions[i, 2])
        pose.orientation.w = 1.0  # Default identity quaternion
        msg.raw_sensor.append(pose)
    
    return msg

