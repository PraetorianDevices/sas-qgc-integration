#!/usr/bin/env python3

"""
GPS Spoofing Detector → MAVLink Bridge

Subscribes to /gps_spoof_alert from gps_spoof_detector_node and converts
spoofing alerts into MAVLink STATUSTEXT messages for display in QGroundControl.

Publishes to UDP port 14550 (QGC standard telemetry port) with proper MAVLink
framing, sequence numbering, and system/component IDs.

Alert Level Mapping
-------------------
  INFO     → MAV_SEVERITY_INFO (0)
  WARNING  → MAV_SEVERITY_WARNING (4)
  CRITICAL → MAV_SEVERITY_CRITICAL (5)

State Mapping
-----------
  NOMINAL           → display in green (info level)
  SUSPICIOUS        → display in yellow (warning level)
  SPOOFING_DETECTED → display in red (critical level)
"""

import json
import socket
import struct
from typing import Optional
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String


class MAVSeverity(IntEnum):
    """MAVLink message severity levels."""
    INFO     = 0
    NOTICE   = 1
    WARNING  = 4
    ERROR    = 3
    CRITICAL = 5
    ALERT    = 2
    EMERGENCY = 6


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

    # MAVLink message IDs
    MAVLINK_MSG_ID_STATUSTEXT = 253
    MAVLINK_MSG_ID_HEARTBEAT = 0

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
        """
        Send a MAVLink STATUSTEXT message via UDP.

        STATUSTEXT format (MAVLink 2.0, msg ID 253):
        - timestamp (uint32): system uptime in ms
        - severity (uint8): MAV_SEVERITY_*
        - text (char[50]): status text message
        - id (uint16): unique message id
        - chunk_seq (uint8): chunking sequence
        """
        if self._socket is None:
            self.get_logger().warn('UDP socket not connected; cannot send STATUSTEXT')
            return

        # Increment sequence for this packet
        seq = self._sequence % 256
        self._sequence += 1

        # Construct MAVLink v2.0 frame
        # Frame format: <STX> <LEN> <INV> <MSG_ID> <SYSID> <COMPID> <SEQ> <PAYLOAD> <CHECKSUM>
        payload = self._build_statustext_payload(text, severity)
        frame = self._build_mavlink_frame(
            msg_id=self.MAVLINK_MSG_ID_STATUSTEXT,
            seq=seq,
            payload=payload
        )

        try:
            self._socket.send(frame)
        except OSError as e:
            self.get_logger().error(f'Failed to send MAVLink packet: {e}')

    def _build_statustext_payload(self, text: str, severity: int) -> bytes:
        """Build STATUSTEXT message payload (50 bytes text + metadata)."""
        # Ensure text is exactly 50 chars (pad with nulls)
        text_bytes = text.encode('ascii', errors='replace').ljust(50, b'\x00')[:50]

        # Severity (uint8)
        severity_bytes = struct.pack('B', severity)

        # ID and chunk_seq (optional, set to 0)
        id_bytes = struct.pack('<H', 0)  # msg_id (uint16, little-endian)
        chunk_seq_bytes = struct.pack('B', 0)  # chunk_seq (uint8)

        return severity_bytes + text_bytes + id_bytes + chunk_seq_bytes

    def _build_mavlink_frame(self, msg_id: int, seq: int, payload: bytes) -> bytes:
        """
        Build a MAVLink 2.0 frame.

        Frame structure:
            [0]     STX (0xFD for v2.0)
            [1]     payload length (0-255)
            [2]     incompatibility flags (0 for standard messages)
            [3]     message ID (8-bit for compatibility; extended in full frame)
            [4]     system ID
            [5]     component ID
            [6]     sequence
            [7:N]   payload
            [N:N+2] CRC
        """
        stx = 0xFD
        payload_len = len(payload)
        incomp_flags = 0x00
        sysid = self.system_id
        compid = self.component_id

        # Construct frame header + payload (without STX, CRC)
        frame_data = struct.pack(
            '<BBBBBBB',
            stx,
            payload_len,
            incomp_flags,
            msg_id & 0xFF,  # msg_id fits in 8 bits for compatibility
            sysid,
            compid,
            seq
        ) + payload

        # Compute CRC (MAVLink CRC_EXTRA + payload CRC)
        crc = self._compute_mavlink_crc(frame_data[1:], msg_id)

        return frame_data + struct.pack('<H', crc)

    @staticmethod
    def _compute_mavlink_crc(data: bytes, msg_id: int) -> int:
        """
        Compute MAVLink CRC16-CCITT.

        CRC is computed over frame data (excluding STX) and includes the
        message-type-specific CRC_EXTRA byte.
        """
        CRC_INIT = 0xFFFF
        CRC_POLY = 0xEF01

        # CRC_EXTRA for message 253 (STATUSTEXT)
        CRC_EXTRA_STATUSTEXT = 83

        crc = CRC_INIT
        for byte in data:
            tmp = byte ^ (crc & 0xFF)
            tmp = (tmp ^ (tmp << 4)) & 0xFF
            crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
            crc &= 0xFFFF

        # Include CRC_EXTRA
        tmp = CRC_EXTRA_STATUSTEXT ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        crc &= 0xFFFF

        return crc


def main(args=None):
    """Entry point for GPS Spoof MAVLink Bridge node."""
    rclpy.init(args=args)
    node = GPSSpoofMAVLinkBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
