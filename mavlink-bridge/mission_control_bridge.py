#!/usr/bin/env python3

"""
Mission Control Bridge: QGroundControl ↔ SAS Mission Executor

Bridges MAVLink mission commands from QGC to the SAS mission_executor_node,
enabling real-time mission upload, progress tracking, and waypoint download.

MAVLink Messages Handled:
  - MISSION_REQUEST_LIST (from QGC) → QGC asking to download our mission
  - MISSION_COUNT (bidirectional) → announces how many items follow
  - MISSION_REQUEST_INT / MISSION_REQUEST (from QGC) → request a specific waypoint
  - MISSION_ITEM_INT (bidirectional) → waypoint data (lat/lon scaled by 1e7)
  - MISSION_ACK (to QGC) → mission accepted/rejected
  - MISSION_CURRENT (to QGC) → current waypoint index + total
  - MISSION_ITEM_REACHED (to QGC) → waypoint reached event

Upload handshake (QGC → vehicle), per the real MAVLink mission protocol:
  1. QGC sends MISSION_COUNT announcing N items
  2. We request each item in sequence via MISSION_REQUEST_INT
  3. QGC responds with MISSION_ITEM_INT for each requested seq
  4. After the last item, we send a single MISSION_ACK and publish the
     assembled mission to mission_executor_node

Download handshake (vehicle → QGC) — unchanged in spirit from before, now
using the corrected wire format:
  1. QGC sends MISSION_REQUEST_LIST
  2. We respond with MISSION_COUNT
  3. QGC requests each item via MISSION_REQUEST_INT / MISSION_REQUEST
  4. We respond with MISSION_ITEM_INT for each

All frame encoding/decoding goes through mavlink_v2.py, which is verified
byte-for-byte against pymavlink — this bridge previously built a
non-standard 7-byte frame header (STX, LEN, INCOMPAT_FLAGS, MSG_ID, SYSID,
COMPID, SEQ) instead of the real 10-byte MAVLink 2.0 header, and used ad hoc
payload layouts for MISSION_ACK/MISSION_CURRENT/MISSION_COUNT that didn't
match the spec at all. None of it was parseable by a real MAVLink peer.

Author: Claude Code
Date: 2026-07-09
"""

import json
import socket
import threading
import time
from typing import Optional, List, Dict
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String

import mavlink_v2 as mav


class MAVMissionType(IntEnum):
    """MAV_CMD values carried in a mission item's `command` field."""
    NAV_WAYPOINT = 16
    NAV_LOITER_UNLIM = 17
    NAV_LOITER_TURNS = 18
    NAV_LOITER_TIME = 19
    NAV_RETURN_TO_LAUNCH = 20
    NAV_LAND = 21
    NAV_TAKEOFF = 22
    DO_CHANGE_SPEED = 178


class MissionControlBridge(Node):
    """
    Bidirectional bridge between QGroundControl and SAS mission executor.

    Responsibilities:
    1. Receive MAVLink mission messages from QGC (UDP)
    2. Run the real mission-protocol upload handshake and assemble the
       received waypoints into the QGC .plan JSON format mission_executor_node
       already parses
    3. Publish the assembled mission to mission_executor_node
    4. Track mission progress and report it back to QGC
    5. Serve mission download requests from QGC
    """

    def __init__(self):
        super().__init__('mission_control_bridge')

        # Parameters
        self.declare_parameter('system_id', 1)
        self.declare_parameter('component_id', 1)
        self.declare_parameter('mavlink_host', 'localhost')
        self.declare_parameter('mavlink_bind_host', '0.0.0.0')
        self.declare_parameter('mavlink_port', 14550)
        self.declare_parameter('drone_id', '')

        self.system_id = self.get_parameter('system_id').value
        self.component_id = self.get_parameter('component_id').value
        mavlink_host = self.get_parameter('mavlink_host').value
        mavlink_bind_host = self.get_parameter('mavlink_bind_host').value
        mavlink_port = self.get_parameter('mavlink_port').value
        self.drone_id = self.get_parameter('drone_id').value
        self.topic_prefix = f'/{self.drone_id}' if self.drone_id else ''

        self.get_logger().info(
            f'Mission Control Bridge initialized: '
            f'system_id={self.system_id}, component_id={self.component_id}, '
            f'bind={mavlink_bind_host}:{mavlink_port}, target={mavlink_host}:{mavlink_port}'
        )

        # UDP socket. Bound to mavlink_bind_host (default 0.0.0.0), NOT
        # mavlink_host -- the two differ whenever QGC runs on a different
        # host/network namespace than this node (e.g. QGC on native Windows,
        # this node inside WSL2 NAT), where mavlink_host is the Windows-side
        # address outbound bridges send to and is never a locally-assignable
        # bind address here.
        self._socket: Optional[socket.socket] = None
        self._sequence = 0
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._socket.bind((mavlink_bind_host, mavlink_port))
            self._socket.settimeout(0.5)
            self.get_logger().info(f'Listening for MAVLink mission messages on {mavlink_bind_host}:{mavlink_port}')
        except OSError as e:
            self.get_logger().error(f'Failed to bind UDP socket: {e}')
            self._socket = None

        # Fallback destination for outbound frames sent before any inbound
        # packet has been seen (e.g. the periodic MISSION_CURRENT timer).
        # Once a real sender is observed, replies target that address instead
        # -- see _handle_mavlink_message / _send_mavlink_frame. Previously
        # every outbound frame was hardcoded to ('localhost', 14550)
        # regardless of this configuration, which silently broke the
        # documented multi-drone setup (different mavlink_port per drone).
        self._reply_addr = (mavlink_host, mavlink_port)

        # Last-seen GCS identity, learned from incoming frames, used as the
        # target_system/target_component of our outbound targeted messages.
        # Defaults to QGC's conventional system_id (255) until we hear from it.
        self._gcs_system_id = 255
        self._gcs_component_id = 0

        # Mission state (download direction: what we report back to QGC on request)
        self._mission_items: List[Optional[Dict]] = []
        self._current_waypoint = 0
        self._mission_in_progress = False

        # Upload state (QGC -> vehicle): items being received one at a time via
        # the MISSION_COUNT -> MISSION_REQUEST_INT -> MISSION_ITEM_INT handshake,
        # pending assembly until every item 0..count-1 has arrived.
        self._upload_items: List[Optional[Dict]] = []
        self._upload_expected_count = 0
        self._upload_in_progress = False

        # QoS for the assembled-mission publisher. RELIABLE, not BEST_EFFORT:
        # an uploaded mission is a single discrete payload that must not be
        # dropped, and mission_executor_node subscribes to load_mission with
        # default QoS (RELIABLE). Under DDS compatibility rules a RELIABLE
        # subscriber never matches a BEST_EFFORT publisher, so the previous
        # BEST_EFFORT setting meant a fully-received mission was published
        # into the void -- rclpy logged "requesting incompatible QoS. No
        # messages will be sent to it. Last incompatible policy: RELIABILITY"
        # and the executor never saw a single upload. TRANSIENT_LOCAL is kept
        # so an executor that starts late still receives the last mission
        # (TRANSIENT_LOCAL publisher -> VOLATILE subscriber is compatible).
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self._mission_upload_pub = self.create_publisher(
            String, f'{self.topic_prefix}/mission_executor/load_mission', qos
        )

        # mission_executor_node publishes its status with default (VOLATILE) QoS;
        # a TRANSIENT_LOCAL subscriber would never match a VOLATILE publisher under
        # DDS QoS-compatibility rules, so this subscription intentionally does not
        # reuse the TRANSIENT_LOCAL `qos` profile above.
        self._mission_status_sub = self.create_subscription(
            String,
            f'{self.topic_prefix}/mission_executor/status',
            self._cb_mission_status,
            10
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
                self._handle_mavlink_message(data, addr)
            except socket.timeout:
                pass
            except OSError:
                break
            except Exception as e:
                self.get_logger().warn(f'Error in receiver loop: {e}')

    def _handle_mavlink_message(self, data: bytes, addr=None):
        """Process every MAVLink message packed into one incoming datagram.

        QGC frequently bundles several outgoing messages (e.g. its own
        HEARTBEAT ahead of a MISSION_COUNT) into a single UDP write -- using
        parse_frame() alone here would silently process only the first one
        and drop the rest with no error, which is exactly what broke mission
        upload before this fix. See parse_frames()'s docstring for the full
        explanation.
        """
        frames = mav.parse_frames(data)
        if not frames:
            self.get_logger().warn(
                f'parse_frames() found NO frames in {len(data)} bytes from {addr}: '
                f'{data[:20].hex()}')
        for parsed in frames:
            if parsed.valid:
                self._handle_one_message(parsed, addr)
            else:
                self.get_logger().warn(f'Invalid/unparseable frame from {addr}: {data[:20]!r}')

    def _handle_one_message(self, parsed, addr=None):
        """Process a single already-parsed MAVLink message."""
        self._gcs_system_id = parsed.system_id
        self._gcs_component_id = parsed.component_id
        if addr is not None:
            self._reply_addr = addr

        try:
            if parsed.msg_id == mav.MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
                self._handle_mission_request_list()
            elif parsed.msg_id == mav.MAVLINK_MSG_ID_MISSION_COUNT:
                self._handle_mission_count(parsed.payload)
            elif parsed.msg_id == mav.MAVLINK_MSG_ID_MISSION_ITEM_INT:
                self._handle_mission_item_int(parsed.payload)
            elif parsed.msg_id in (mav.MAVLINK_MSG_ID_MISSION_REQUEST,
                                    mav.MAVLINK_MSG_ID_MISSION_REQUEST_INT):
                self._handle_mission_request(parsed.payload)
        except Exception as e:
            self.get_logger().warn(f'Error handling MAVLink message (id={parsed.msg_id}): {e}')

    # ===== Upload handshake: QGC -> vehicle =====

    def _handle_mission_request_list(self):
        """QGC wants to download our current mission (download direction)."""
        self.get_logger().info('MISSION_REQUEST_LIST received from QGC')
        self._send_mission_count(len(self._mission_items))

    def _handle_mission_count(self, payload: bytes):
        """QGC is announcing it wants to upload a mission of this size.

        This is the message that actually starts an upload; the previous
        implementation had no handler for it and instead reacted to
        unsolicited MISSION_ITEM messages that a real QGC never sends
        without being asked first.
        """
        parsed = mav.parse_mission_count(payload)
        if parsed is None:
            return

        count = parsed['count']
        self.get_logger().info(f'MISSION_COUNT received from QGC: {count} item(s) incoming')

        self._upload_items = [None] * count
        self._upload_expected_count = count
        self._upload_in_progress = count > 0

        if count == 0:
            # Empty mission upload (e.g. "clear mission") — nothing to request.
            self._send_mission_ack(mav.MAVMissionResult.ACCEPTED)
            return

        self._send_mission_request_int(0)

    def _handle_mission_item_int(self, payload: bytes):
        """QGC is sending a waypoint we requested during an upload."""
        item = mav.parse_mission_item_int(payload)
        if item is None:
            self._send_mission_ack(mav.MAVMissionResult.ERROR)
            return

        seq = item['sequence']
        if not self._upload_in_progress or seq >= self._upload_expected_count:
            self.get_logger().warn(
                f'Unexpected MISSION_ITEM_INT seq={seq} (no upload in progress)')
            return

        self._upload_items[seq] = item
        self.get_logger().info(
            f'Received waypoint {seq}/{self._upload_expected_count - 1}: '
            f'({item["position"]["latitude"]}, {item["position"]["longitude"]}, '
            f'{item["position"]["altitude"]}m)'
        )

        next_seq = seq + 1
        if next_seq < self._upload_expected_count:
            self._send_mission_request_int(next_seq)
            return

        # All items received. The real mission protocol ACKs once, after the
        # last item — the previous implementation incorrectly ACKed every
        # individual item as it arrived.
        self._complete_upload()

    def _complete_upload(self):
        """Finish an upload: adopt the received items, ACK, and publish to
        mission_executor_node so it actually loads and can execute it."""
        self._mission_items = list(self._upload_items)
        self._upload_in_progress = False
        self._send_mission_ack(mav.MAVMissionResult.ACCEPTED)
        self._publish_mission_to_executor(self._mission_items)

    def _publish_mission_to_executor(self, items: List[Optional[Dict]]):
        """Assemble received waypoints into the QGC .plan JSON format that
        mission_executor_node.parse_qgc_mission() already understands, and
        publish it. This publisher previously existed but was never called —
        a successfully received mission never reached the executor at all.
        """
        plan_items = []
        for item in items:
            if item is None:
                continue
            # MAVLink mission item params are param1-4 followed by lat/lon/alt
            # (param5-7); mission_executor_node's QGC-plan parser reads
            # lat/lon/alt from params[4]/[5]/[6] accordingly.
            params = list(item['params']) + [
                item['position']['latitude'],
                item['position']['longitude'],
                item['position']['altitude'],
            ]
            plan_items.append({'command': item['command'], 'params': params})

        mission_plan = {
            'fileType': 'Plan',
            'groundStation': 'mission_control_bridge',
            'mission': {'items': plan_items},
        }

        msg = String()
        msg.data = json.dumps(mission_plan)
        self._mission_upload_pub.publish(msg)
        self.get_logger().info(
            f'Published {len(plan_items)}-waypoint mission to mission_executor_node')

    # ===== Download handshake: vehicle -> QGC =====

    def _handle_mission_request(self, payload: bytes):
        """QGC is requesting a specific waypoint (MISSION_REQUEST or
        MISSION_REQUEST_INT — both share the same wire layout)."""
        parsed = mav.parse_mission_request(payload)
        if parsed is None:
            return

        seq = parsed['sequence']
        if seq < len(self._mission_items) and self._mission_items[seq] is not None:
            self._send_mission_item_int(seq, self._mission_items[seq])
        else:
            self.get_logger().warn(f'Waypoint {seq} not found')

    # ===== Status / progress =====

    def _cb_mission_status(self, msg: String):
        """Receive mission status from mission_executor_node."""
        try:
            status = json.loads(msg.data)
            self._current_waypoint = status.get('current_waypoint', 0)
            # mission_executor_node publishes a 'state' field (idle/loading/executing/
            # paused/completed/interrupted/error), not 'in_progress' — derive it here.
            state = status.get('state', '')
            self._mission_in_progress = state in ('executing', 'paused', 'loading')
        except json.JSONDecodeError:
            pass

    def _publish_mission_current(self):
        """Publish current waypoint to QGC."""
        if len(self._mission_items) == 0:
            return

        payload = mav.build_mission_current(
            seq=self._current_waypoint,
            total=len(self._mission_items),
        )
        self._send_mavlink_frame(mav.MAVLINK_MSG_ID_MISSION_CURRENT, payload)

    # ===== Outbound message builders =====

    def _send_mission_count(self, count: int):
        payload = mav.build_mission_count(
            count, target_system=self._gcs_system_id, target_component=self._gcs_component_id)
        self._send_mavlink_frame(mav.MAVLINK_MSG_ID_MISSION_COUNT, payload)

    def _send_mission_ack(self, result: int):
        payload = mav.build_mission_ack(
            result, target_system=self._gcs_system_id, target_component=self._gcs_component_id)
        self._send_mavlink_frame(mav.MAVLINK_MSG_ID_MISSION_ACK, payload)

    def _send_mission_request_int(self, seq: int):
        payload = mav.build_mission_request_int(
            seq, target_system=self._gcs_system_id, target_component=self._gcs_component_id)
        self._send_mavlink_frame(mav.MAVLINK_MSG_ID_MISSION_REQUEST_INT, payload)

    def _send_mission_item_int(self, seq: int, item: Dict):
        """Send a waypoint to QGC as MISSION_ITEM_INT (scaled-int lat/lon)."""
        frame = item.get('frame', 3)  # MAV_FRAME_GLOBAL_RELATIVE_ALT
        command = item.get('command', MAVMissionType.NAV_WAYPOINT)
        current = item.get('current', 0)
        autocontinue = item.get('autocontinue', 1)
        params = item.get('params', [0.0, 0.0, 0.0, 0.0])
        pos = item.get('position', {})

        lat = int(pos.get('latitude', 0) * 1e7)
        lon = int(pos.get('longitude', 0) * 1e7)
        alt = float(pos.get('altitude', 0))

        payload = mav.build_mission_item_int(
            seq=seq, frame=frame, command=command, current=current,
            autocontinue=autocontinue,
            param1=params[0], param2=params[1], param3=params[2], param4=params[3],
            x=lat, y=lon, z=alt,
            target_system=self._gcs_system_id, target_component=self._gcs_component_id,
        )
        self._send_mavlink_frame(mav.MAVLINK_MSG_ID_MISSION_ITEM_INT, payload)

    def _send_mavlink_frame(self, msg_id: int, payload: bytes):
        """Send a spec-compliant MAVLink 2.0 frame to QGC."""
        if self._socket is None:
            return

        seq = self._sequence % 256
        self._sequence += 1

        frame = mav.build_frame(msg_id, seq, payload, self.system_id, self.component_id)

        try:
            self._socket.sendto(frame, self._reply_addr)
        except OSError as e:
            self.get_logger().warn(f'Failed to send MAVLink frame: {e}')


def main(args=None):
    """Entry point for mission control bridge."""
    rclpy.init(args=args)
    node = MissionControlBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
