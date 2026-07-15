#!/usr/bin/env python3

"""
GPS Spoofing Detector → MAVLink Bridge

Subscribes to /gps_spoof_alert from gps_spoof_detector_node and converts
spoofing alerts into MAVLink STATUSTEXT messages for display in QGroundControl.

Publishes to UDP port 14550 (QGC standard telemetry port) with proper MAVLink
framing, sequence numbering, and system/component IDs.

Alert Level Mapping
-------------------
  INFO     → MAV_SEVERITY_INFO (6)
  WARNING  → MAV_SEVERITY_WARNING (4)
  CRITICAL → MAV_SEVERITY_CRITICAL (2)

State Mapping
-----------
  NOMINAL           → display in green (info level)
  SUSPICIOUS        → display in yellow (warning level)
  SPOOFING_DETECTED → display in red (critical level)
"""

import json
import socket
from typing import Optional
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String

import mavlink_v2 as mav


class MAVSeverity(IntEnum):
    """MAV_SEVERITY, per the MAVLink common spec (lower = more severe).

    This enum previously had its values transposed (INFO=0, CRITICAL=5,
    ALERT=2, EMERGENCY=6 -- i.e. INFO and EMERGENCY were swapped, and ALERT
    and CRITICAL were swapped), so a genuine CRITICAL spoofing alert was
    transmitted at NOTICE priority and an INFO alert at EMERGENCY priority --
    backwards from what QGC's severity-based color coding expects. Fixed to
    the real MAV_SEVERITY values, matching fleet_manager_mavlink_bridge.py and
    emergency_wipe_mavlink_bridge.py.
    """
    EMERGENCY = 0
    ALERT = 1
    CRITICAL = 2
    ERROR = 3
    WARNING = 4
    NOTICE = 5
    INFO = 6
    DEBUG = 7


class MAVFrameType(IntEnum):
    """MAVLink frame type bytes."""
    REQUEST      = 0x00  # Unused in STATUSTEXT context
    REPLY        = 0x01  # Unused in STATUSTEXT context
    ENUM         = 0x02
    VERSION      = 0x03
    COMMAND_INT  = 0x04
    COMMAND_LONG = 0x05


class GPSSpoofMAVLinkBridge(Node):
    """
    Bridges GPS spoofing alerts from ROS 2 to MAVLink STATUSTEXT messages.

    Publishes MAVLink packets to a UDP socket at configurable host:port
    (default localhost:14550 for QGroundControl).
    """

    def __init__(self):
        super().__init__('gps_spoof_mavlink_bridge')

        # Parameters
        self.declare_parameter('system_id', 1)
        self.declare_parameter('component_id', 200)  # Custom component ID for SAS
        self.declare_parameter('mavlink_host', 'localhost')
        self.declare_parameter('mavlink_port', 14550)

        self.system_id = self.get_parameter('system_id').value
        self.component_id = self.get_parameter('component_id').value
        mavlink_host = self.get_parameter('mavlink_host').value
        mavlink_port = self.get_parameter('mavlink_port').value

        self.get_logger().info(
            f'GPS Spoof MAVLink Bridge initialized: '
            f'system_id={self.system_id}, component_id={self.component_id}, '
            f'target={mavlink_host}:{mavlink_port}'
        )

        # UDP socket for MAVLink output
        self._socket: Optional[socket.socket] = None
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.connect((mavlink_host, mavlink_port))
            self.get_logger().info(f'Connected to MAVLink endpoint {mavlink_host}:{mavlink_port}')
        except OSError as e:
            self.get_logger().error(f'Failed to connect UDP socket: {e}')
            self._socket = None

        # Sequence counter for MAVLink packets
        self._sequence = 0

        # QoS profile matching PX4's DDS bridge
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to GPS spoofing alerts
        self.create_subscription(
            String,
            '/gps_spoof_alert',
            self._cb_gps_spoof_alert,
            qos
        )

        self.get_logger().info('GPS Spoof MAVLink Bridge started')

    def _cb_gps_spoof_alert(self, msg: String):
        """
        Handle incoming GPS spoofing alert from detector node.

        Converts JSON alert to MAVLink STATUSTEXT and sends via UDP.
        """
        try:
            alert = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f'Failed to parse JSON alert: {msg.data}')
            return

        level = alert.get('level', 'WARNING')
        state = alert.get('state', 'NOMINAL')
        strategy = alert.get('strategy', 'UNKNOWN')
        detail = alert.get('detail', {})
        alert_id = alert.get('alert_id', 0)

        # Map alert level to MAVLink severity
        severity_map = {
            'INFO': MAVSeverity.INFO,
            'WARNING': MAVSeverity.WARNING,
            'CRITICAL': MAVSeverity.CRITICAL,
        }
        mav_severity = severity_map.get(level, MAVSeverity.WARNING)

        # Build readable message for operator
        if state == 'SPOOFING_DETECTED':
            desc = f'[GPS SPOOF DETECTED] {strategy}: {detail.get("description", "")}'
        elif state == 'SUSPICIOUS':
            desc = f'[GPS SUSPICIOUS] {strategy}: {detail.get("description", "")}'
        else:
            desc = f'[GPS OK] {strategy} nominal'

        # Truncate to MAVLink STATUSTEXT max length (50 chars)
        msg_text = desc[:50]

        self.get_logger().info(
            f'GPS Spoof Alert: id={alert_id} level={level} state={state} '
            f'strategy={strategy} → MAVLink STATUSTEXT (severity={mav_severity.value})'
        )

        # Send MAVLink STATUSTEXT packet
        self._send_statustext(msg_text, int(mav_severity))

    def _send_statustext(self, text: str, severity: int):
        """Send a MAVLink 2.0 STATUSTEXT message via UDP."""
        if self._socket is None:
            self.get_logger().warn('UDP socket not connected; cannot send STATUSTEXT')
            return

        seq = self._sequence % 256
        self._sequence += 1

        payload = mav.build_statustext(text, severity)
        frame = mav.build_frame(
            mav.MAVLINK_MSG_ID_STATUSTEXT, seq, payload,
            self.system_id, self.component_id
        )

        try:
            self._socket.send(frame)
        except OSError as e:
            self.get_logger().error(f'Failed to send MAVLink packet: {e}')


def main(args=None):
    """Entry point for GPS Spoof MAVLink Bridge node."""
    rclpy.init(args=args)
    node = GPSSpoofMAVLinkBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
