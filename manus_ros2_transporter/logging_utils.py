"""
MANUS-style colored logging utilities for manus_ros2_transporter

Provides colored terminal output matching the MANUS SDK logging style.
"""

import sys
from datetime import datetime
from typing import Optional


class Colors:
    """ANSI color codes for terminal output"""
    RESET = '\033[0m'
    BOLD = '\033[1m'
    
    # Log level colors (matching spdlog style used by MANUS SDK)
    DEBUG = '\033[36m'      # Cyan
    INFO = '\033[32m'       # Green
    WARN = '\033[33m'       # Yellow
    ERROR = '\033[31m'      # Red
    
    # Additional colors
    CYAN = '\033[36m'
    MAGENTA = '\033[35m'
    WHITE = '\033[37m'
    GRAY = '\033[90m'


def _timestamp() -> str:
    """Get current timestamp in MANUS style"""
    return datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]


def log_debug(msg: str) -> None:
    """Print debug message with cyan color"""
    print(f"{Colors.GRAY}[{_timestamp()}] {Colors.DEBUG}[debug]{Colors.RESET} {msg}")
    sys.stdout.flush()


def log_info(msg: str) -> None:
    """Print info message with green color"""
    print(f"{Colors.GRAY}[{_timestamp()}] {Colors.INFO}[info]{Colors.RESET} {msg}")
    sys.stdout.flush()


def log_warn(msg: str) -> None:
    """Print warning message with yellow color"""
    print(f"{Colors.GRAY}[{_timestamp()}] {Colors.WARN}[warn]{Colors.RESET} {msg}", file=sys.stderr)
    sys.stderr.flush()


def log_error(msg: str) -> None:
    """Print error message with red color"""
    print(f"{Colors.GRAY}[{_timestamp()}] {Colors.ERROR}[error]{Colors.RESET} {msg}", file=sys.stderr)
    sys.stderr.flush()


def log_glove_discovered(glove_id: int, side: str, alias: Optional[str] = None) -> None:
    """Print glove discovery message in MANUS style"""
    alias_str = f" ({alias})" if alias else ""
    # Format glove_id as hex (matching MANUS SDK output)
    hex_id = f"0x{glove_id & 0xFFFFFFFF:08X}"
    print(
        f"{Colors.GRAY}[{_timestamp()}] {Colors.INFO}[info]{Colors.RESET} "
        f"Glove ID {Colors.CYAN}{hex_id}{Colors.RESET}: "
        f"Detected {Colors.BOLD}{side}{Colors.RESET} hand{alias_str}"
    )
    sys.stdout.flush()


def log_zmq_bound(address: str, side: str) -> None:
    """Print ZMQ socket bound message"""
    print(
        f"{Colors.GRAY}[{_timestamp()}] {Colors.INFO}[info]{Colors.RESET} "
        f"ZMQ PUB socket bound: {Colors.CYAN}{address}{Colors.RESET} ({side} hand)"
    )
    sys.stdout.flush()


def log_zmq_connected(address: str) -> None:
    """Print ZMQ connection message"""
    print(
        f"{Colors.GRAY}[{_timestamp()}] {Colors.INFO}[info]{Colors.RESET} "
        f"Connected to ZMQ server: {Colors.CYAN}{address}{Colors.RESET}"
    )
    sys.stdout.flush()


def log_zmq_disconnected() -> None:
    """Print ZMQ disconnection message"""
    print(
        f"{Colors.GRAY}[{_timestamp()}] {Colors.WARN}[warn]{Colors.RESET} "
        f"Disconnected from ZMQ server"
    )
    sys.stdout.flush()


def log_zmq_reconnecting(address: str) -> None:
    """Print ZMQ reconnection attempt message"""
    print(
        f"{Colors.GRAY}[{_timestamp()}] {Colors.INFO}[info]{Colors.RESET} "
        f"Reconnecting to {Colors.CYAN}{address}{Colors.RESET}..."
    )
    sys.stdout.flush()


def log_data_received(side: str, count: int) -> None:
    """Print data received statistics"""
    print(
        f"{Colors.GRAY}[{_timestamp()}] {Colors.INFO}[info]{Colors.RESET} "
        f"{side} hand: {Colors.CYAN}{count}{Colors.RESET} messages received"
    )
    sys.stdout.flush()


def log_startup_banner(component: str, version: str = "0.1.0") -> None:
    """Print startup banner"""
    print(f"\n{Colors.CYAN}{'='*60}{Colors.RESET}")
    print(f"{Colors.BOLD}MANUS ROS2 Transporter - {component}{Colors.RESET}")
    print(f"Version: {version}")
    print(f"{Colors.CYAN}{'='*60}{Colors.RESET}\n")
    sys.stdout.flush()


def log_shutdown() -> None:
    """Print shutdown message"""
    print(
        f"{Colors.GRAY}[{_timestamp()}] {Colors.INFO}[info]{Colors.RESET} "
        f"Shutting down..."
    )
    sys.stdout.flush()

