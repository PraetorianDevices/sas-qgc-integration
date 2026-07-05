#!/usr/bin/env python3

"""
Mission Control Bridge: QGroundControl ↔ SAS Mission Executor

Bridges MAVLink mission commands from QGC to the SAS mission_executor_node,
enabling real-time mission upload, progress tracking, and waypoint editing.

MAVLink Messages Handled:
  - MISSION_REQUEST_LIST (from QGC) → list missions on drone
  - MISSION_COUNT (to QGC) → report number of waypoints
  - MISSION_REQUEST (from QGC) → request specific waypoint
  - MISSION_ITEM (bidirectional) → waypoint data
  - MISSION_ACK (to QGC) → mission accepted/rejected
  - MISSION_CURRENT (to QGC) → current waypoint index
  - MISSION_ITEM_REACHED (to QGC) → waypoint reached event

Architecture:
  QGroundControl
    ↓ (MAVLink UDP 14550)
  mission_control_bridge (ROS 2 node)
    ├→ Receives MISSION_* messages from UDP
    ├→ Converts to ROS 2 topics/services
    ├→ Publishes to mission_executor_node
    └→ Receives mission progress
        ↓
    Publishes back to UDP as MAVLink

Author: Claude Code
Date: 2026-07-05
"""

import json
import struct
import socket
import threading
import time
from typing import Optional, List, Dict
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String


class MAVMissionType(IntEnum):
    """MAVLink mission item types."""
    NAV_WAYPOINT = 16
    NAV_LOITER_UNLIM = 17
    NAV_LOITER_TURNS = 18
    NAV_LOITER_TIME = 19
    NAV_RETURN_TO_LAUNCH = 20
    NAV_LAND = 21
    NAV_TAKEOFF = 22
    DO_CHANGE_SPEED = 178


class MAVMissionResult(IntEnum):
    """MAVLink mission acknowledgement codes."""
    ACCEPTED = 0
    ERROR = 1
    UNSUPPORTED_FRAME = 2
    UNSUPPORTED_COMMAND = 3


class MissionControlBridge(Node):
    """
    Bidirectional bridge between QGroundControl and SAS mission executor.

    Responsibilities:
    1. Receive MAVLink mission messages from QGC (UDP)
    2. Convert to ROS 2 mission format
    3. Send to mission_executor_node
    4. Track mission progress
    5. Send mission status back to QGC
    """

    # MAVLink message IDs
    MAVLINK_MSG_ID_MISSION_REQUEST_LIST = 43
    MAVLINK_MSG_ID_MISSION_COUNT = 44
    MAVLINK_MSG_ID_MISSION_REQUEST = 40
    MAVLINK_MSG_ID_MISSION_ITEM = 39
    MAVLINK_MSG_ID_MISSION_ACK = 47
    MAVLINK_MSG_ID_MISSION_CURRENT = 42
    MAVLINK_MSG_ID_MISSION_ITEM_REACHED = 61

    def __init__(self):
        super().__init__('mission_control_bridge')

        # Parameters
        self.declare_parameter('system_id', 1)
        self.declare_parameter('component_id', 1)
        self.declare_parameter('mavlink_host', 'localhost')
        self.declare_parameter('mavlink_port', 14550)
        self.declare_parameter('drone_id', '')

        self.system_id = self.get_parameter('system_id').value
        self.component_id = self.get_parameter('component_id').value
        mavlink_host = self.get_parameter('mavlink_host').value
        mavlink_port = self.get_parameter('mavlink_port').value
        self.drone_id = self.get_parameter('drone_id').value
        self.topic_prefix = f'/{self.drone_id}' if self.drone_id else ''

        self.get_logger().info(
            f'Mission Control Bridge initialized: '
            f'system_id={self.system_id}, component_id={self.component_id}, '
            f'target={mavlink_host}:{mavlink_port}'
        )

        # UDP socket
        self._socket: Optional[socket.socket] = None
        self._sequence = 0
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._socket.bind((mavlink_host, mavlink_port))
            self._socket.settimeout(0.5)
            self.get_logger().info(f'Listening for MAVLink mission messages on {mavlink_host}:{mavlink_port}')
        except OSError as e:
            self.get_logger().error(f'Failed to bind UDP socket: {e}')
            self._socket = None

        # Mission state
        self._mission_items: List[Dict] = []
        self._current_waypoint = 0
        self._mission_in_progress = False

        # QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self._mission_upload_pub = self.create_publisher(
            String, f'{self.topic_prefix}/mission_executor/load_mission', qos
        )

        self._mission_status_sub = self.create_subscription(
            String,
            f'{self.topic_prefix}/mission_executor/status',
            self._cb_mission_status,
            qos
        )

        # UDP receiver thread
        self._receiver_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._receiver_thread.start()

        # Status publisher timer
        self.create_timer(1.0, self._publish_mission_current)

        self.get_logger().info('Mission Control Bridge started')

    def _receive_loop(self):
        """Background thread: receive MAVLink messages from QGC."""
        while rclpy.ok():
            try:
                if self._socket is None:
                    time.sleep(0.1)
                    continue

                data, addr = self._socket.recvfrom(1024)
                self._handle_mavlink_message(data)
            except socket.timeout:
                pass
            except OSError:
                break
            except Exception as e:
                self.get_logger().warn(f'Error in receiver loop: {e}')

    def _handle_mavlink_message(self, data: bytes):
        """Process incoming MAVLink message."""
        if len(data) < 10:
            return

        # Parse minimal header to identify message type
        try:
            stx = data[0]
            if stx != 0xFD:
                return

            msg_id = data[3]
            system_id = data[4]
            component_id = data[5]

            # Route to handler
            if msg_id == self.MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
                self._handle_mission_request_list()
            elif msg_id == self.MAVLINK_MSG_ID_MISSION_ITEM:
                self._handle_mission_item(data)
            elif msg_id == self.MAVLINK_MSG_ID_MISSION_REQUEST:
                self._handle_mission_request(data)
        except Exception as e:
            self.get_logger().warn(f'Error handling MAVLink message: {e}')

    def _handle_mission_request_list(self):
        """QGC is requesting the list of waypoints."""
        self.get_logger().info('MISSION_REQUEST_LIST received from QGC')
        # Send MISSION_COUNT with current mission size
        self._send_mission_count(len(self._mission_items))

    def _handle_mission_item(self, data: bytes):
        """QGC is uploading a waypoint."""
        try:
            payload = data[7:-2]  # Extract payload (skip header and CRC)

            # Parse MISSION_ITEM (39 bytes minimum)
            if len(payload) < 37:
                return

            # Extract key fields
            seq = struct.unpack('<H', payload[0:2])[0]
            frame = payload[2]
            command = struct.unpack('<H', payload[3:5])[0]
            current = payload[5]
            autocontinue = payload[6]
            param1 = struct.unpack('<f', payload[7:11])[0]
            param2 = struct.unpack('<f', payload[11:15])[0]
            param3 = struct.unpack('<f', payload[15:19])[0]
            param4 = struct.unpack('<f', payload[19:23])[0]
            x = struct.unpack('<i', payload[23:27])[0]  # lat in 1e7 degrees
            y = struct.unpack('<i', payload[27:31])[0]  # lon in 1e7 degrees
            z = struct.unpack('<f', payload[31:35])[0]  # altitude in meters

            # Convert MAVLink mission item to SAS format
            mission_item = {
                'sequence': seq,
                'frame': frame,  # 0=MAV_FRAME_GLOBAL, 3=MAV_FRAME_GLOBAL_RELATIVE_ALT
                'command': command,
                'current': current,
                'autocontinue': autocontinue,
                'params': [param1, param2, param3, param4],
                'position': {
                    'latitude': x / 1e7,
                    'longitude': y / 1e7,
                    'altitude': z
                }
            }

            # Store or update waypoint
            while len(self._mission_items) <= seq:
                self._mission_items.append(None)
            self._mission_items[seq] = mission_item

            self.get_logger().info(f'Received waypoint {seq}: ({mission_item["position"]["latitude"]}, {mission_item["position"]["longitude"]}, {mission_item["position"]["altitude"]}m)')

            # Acknowledge receipt
            self._send_mission_ack(MAVMissionResult.ACCEPTED)
        except Exception as e:
            self.get_logger().error(f'Error parsing MISSION_ITEM: {e}')
            self._send_mission_ack(MAVMissionResult.ERROR)

    def _handle_mission_request(self, data: bytes):
        """QGC is requesting a specific waypoint."""
        try:
            payload = data[7:-2]
            seq = struct.unpack('<H', payload[0:2])[0]

            if seq < len(self._mission_items) and self._mission_items[seq] is not None:
                self._send_mission_item(seq, self._mission_items[seq])
            else:
                self.get_logger().warn(f'Waypoint {seq} not found')
        except Exception as e:
            self.get_logger().error(f'Error handling MISSION_REQUEST: {e}')

    def _cb_mission_status(self, msg: String):
        """Receive mission status from mission_executor_node."""
        try:
            status = json.loads(msg.data)
            self._current_waypoint = status.get('current_waypoint', 0)
            self._mission_in_progress = status.get('in_progress', False)
        except json.JSONDecodeError:
            pass

    def _publish_mission_current(self):
        """Publish current waypoint to QGC."""
        if len(self._mission_items) == 0:
            return

        payload = struct.pack('<H I', self._current_waypoint, int(time.time() * 1000))
        self._send_mavlink_frame(self.MAVLINK_MSG_ID_MISSION_CURRENT, payload)

    def _send_mission_count(self, count: int):
        """Send MISSION_COUNT to QGC."""
        payload = struct.pack('<H I', count, 0)
        self._send_mavlink_frame(self.MAVLINK_MSG_ID_MISSION_COUNT, payload)

    def _send_mission_ack(self, result: int):
        """Send MISSION_ACK to QGC."""
        payload = struct.pack('<H I', result, 0)
        self._send_mavlink_frame(self.MAVLINK_MSG_ID_MISSION_ACK, payload)

    def _send_mission_item(self, seq: int, item: Dict):
        """Send a waypoint to QGC."""
        frame = item.get('frame', 3)  # MAV_FRAME_GLOBAL_RELATIVE_ALT
        command = item.get('command', 16)  # NAV_WAYPOINT
        current = item.get('current', 0)
        autocontinue = item.get('autocontinue', 1)
        params = item.get('params', [0, 0, 0, 0])
        pos = item.get('position', {})

        lat = int(pos.get('latitude', 0) * 1e7)
        lon = int(pos.get('longitude', 0) * 1e7)
        alt = float(pos.get('altitude', 0))

        payload = struct.pack('<H B H B B f f f f i i f',
            seq, frame, command, current, autocontinue,
            params[0], params[1], params[2], params[3],
            lat, lon, alt
        )

        self._send_mavlink_frame(self.MAVLINK_MSG_ID_MISSION_ITEM, payload)

    def _send_mavlink_frame(self, msg_id: int, payload: bytes):
        """Send MAVLink frame to QGC."""
        if self._socket is None:
            return

        seq = self._sequence % 256
        self._sequence += 1

        stx = 0xFD
        payload_len = len(payload)
        incomp_flags = 0x00

        frame_data = struct.pack('<BBBBBBB',
            stx, payload_len, incomp_flags, msg_id & 0xFF,
            self.system_id, self.component_id, seq
        ) + payload

        crc = self._compute_crc(frame_data[1:], msg_id)
        frame = frame_data + struct.pack('<H', crc)

        try:
            self._socket.sendto(frame, ('localhost', 14550))
        except OSError as e:
            self.get_logger().warn(f'Failed to send MAVLink frame: {e}')

    @staticmethod
    def _compute_crc(data: bytes, msg_id: int) -> int:
        """Compute MAVLink CRC16-CCITT."""
        CRC_INIT = 0xFFFF
        CRC_EXTRA_MAP = {
            39: 191,  # MISSION_ITEM
            40: 230,  # MISSION_REQUEST
            42: 4,    # MISSION_CURRENT
            43: 33,   # MISSION_REQUEST_LIST
            44: 142,  # MISSION_COUNT
            47: 153,  # MISSION_ACK
            61: 16,   # MISSION_ITEM_REACHED
        }

        crc_extra = CRC_EXTRA_MAP.get(msg_id, 0)
        crc = CRC_INIT

        for byte in data:
            tmp = byte ^ (crc & 0xFF)
            tmp = (tmp ^ (tmp << 4)) & 0xFF
            crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
            crc &= 0xFFFF

        tmp = crc_extra ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        crc &= 0xFFFF

        return crc


def main(args=None):
    """Entry point for mission control bridge."""
    rclpy.init(args=args)
    node = MissionControlBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
