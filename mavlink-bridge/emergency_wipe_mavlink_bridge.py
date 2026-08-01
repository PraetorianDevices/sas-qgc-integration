#!/usr/bin/env python3

"""
Emergency Wipe Bridge: MAVLink COMMAND_LONG (from QGC) → /emergency_wipe/execute

Receives a MAVLink COMMAND_LONG from QGroundControl and, if it passes a
deliberate two-factor safety gate, triggers the SAS emergency-wipe service.
Acknowledges every gated command back to QGC via COMMAND_ACK, and forwards
wipe progress/results from /emergency_wipe/status to QGC as STATUSTEXT.

Direction: this is the only MAVLink→ROS bridge among the SAS bridges (the
others are ROS→MAVLink). It binds a UDP socket and runs a background receiver
thread, mirroring mission_control_bridge.py's inbound pattern.

Safety gate (why it lives here): emergency_wipe_node's service is
std_srvs/Trigger — an EMPTY request with no field to carry a confirmation
token — and the node itself has no auth/arming gate. A destructive,
irreversible wipe must therefore be gated by the bridge. We require BOTH:
  1. param1 == wipe_magic_param1 (a magic confirm value, default 1.0), and
  2. confirmation >= 1 (COMMAND_LONG's confirmation byte).
Anything failing the gate is answered COMMAND_ACK=DENIED and no service call is
made. This is defense-in-depth on top of QGC's own confirmation dialog.

The command id defaults to MAV_CMD_USER_1 (31010); there is no standardized
MAV_CMD for "wipe onboard data", and USER_1..5 (31010-31013) are reserved for
exactly this kind of vendor-specific command. Both the command id and the magic
value are ROS parameters.

All frame encoding/decoding goes through mavlink_v2.py (verified byte-for-byte
against pymavlink).
"""

import json
import socket
import threading
import time
from typing import Optional
from enum import IntEnum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

import mavlink_v2 as mav

MAV_CMD_USER_1 = 31010


class MAVSeverity(IntEnum):
    """MAV_SEVERITY, spec-correct values (see fleet_manager_mavlink_bridge)."""
    EMERGENCY = 0
    ALERT = 1
    CRITICAL = 2
    ERROR = 3
    WARNING = 4
    NOTICE = 5
    INFO = 6
    DEBUG = 7


class EmergencyWipeMAVLinkBridge(Node):
    """Bridges a gated MAVLink COMMAND_LONG to the emergency-wipe service."""

    def __init__(self):
        super().__init__('emergency_wipe_mavlink_bridge')

        self.declare_parameter('system_id', 1)
        self.declare_parameter('component_id', 1)
        self.declare_parameter('mavlink_host', 'localhost')
        self.declare_parameter('mavlink_bind_host', '0.0.0.0')
        self.declare_parameter('mavlink_port', 14550)
        self.declare_parameter('drone_id', '')
        self.declare_parameter('wipe_command_id', MAV_CMD_USER_1)
        self.declare_parameter('wipe_magic_param1', 1.0)

        self.system_id = self.get_parameter('system_id').value
        self.component_id = self.get_parameter('component_id').value
        mavlink_host = self.get_parameter('mavlink_host').value
        mavlink_bind_host = self.get_parameter('mavlink_bind_host').value
        mavlink_port = self.get_parameter('mavlink_port').value
        self.drone_id = self.get_parameter('drone_id').value
        self.topic_prefix = f'/{self.drone_id}' if self.drone_id else ''
        self.wipe_command_id = self.get_parameter('wipe_command_id').value
        self.wipe_magic_param1 = self.get_parameter('wipe_magic_param1').value

        self.get_logger().info(
            f'Emergency Wipe MAVLink Bridge initialized: '
            f'system_id={self.system_id}, component_id={self.component_id}, '
            f'listen={mavlink_bind_host}:{mavlink_port} (QGC target={mavlink_host}:{mavlink_port}), '
            f'wipe_command_id={self.wipe_command_id}'
        )

        # UDP socket (inbound from QGC). Bound to mavlink_bind_host (default
        # 0.0.0.0), NOT mavlink_host -- the two differ whenever QGC runs on a
        # different host/network namespace than this node (e.g. QGC on native
        # Windows, this node inside WSL2 NAT), where mavlink_host is the
        # Windows-side address outbound bridges send to and is never a
        # locally-assignable bind address here.
        self._socket: Optional[socket.socket] = None
        self._sequence = 0
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._socket.bind((mavlink_bind_host, mavlink_port))
            self._socket.settimeout(0.5)
            self.get_logger().info(f'Listening for MAVLink commands on {mavlink_bind_host}:{mavlink_port}')
        except OSError as e:
            self.get_logger().error(f'Failed to bind UDP socket: {e}')
            self._socket = None

        # Reply target, learned from inbound frames (see mission_control_bridge).
        self._reply_addr = (mavlink_host, mavlink_port)

        # Service client for the wipe trigger.
        self._wipe_client = self.create_client(
            Trigger, f'{self.topic_prefix}/emergency_wipe/execute')

        # Forward wipe status/results to QGC as STATUSTEXT.
        self.create_subscription(
            String, f'{self.topic_prefix}/emergency_wipe/status',
            self._cb_wipe_status, 10)

        # UDP receiver thread
        self._receiver_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._receiver_thread.start()

        self.get_logger().info('Emergency Wipe MAVLink Bridge started')

    # ===== Inbound MAVLink =====

    def _receive_loop(self):
        """Background thread: receive MAVLink commands from QGC."""
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
        """Process one inbound frame; act only on our configured command id."""
        parsed = mav.parse_frame(data)
        if parsed is None or not parsed.valid:
            return
        if addr is not None:
            self._reply_addr = addr

        if parsed.msg_id != mav.MAVLINK_MSG_ID_COMMAND_LONG:
            return

        cmd = mav.parse_command_long(parsed.payload)
        if cmd is None or cmd['command'] != self.wipe_command_id:
            return  # not ours — do not ACK commands addressed elsewhere

        # Target filtering: honor commands addressed to us or broadcast (0).
        target = cmd['target_system']
        if target not in (0, self.system_id):
            return

        self._handle_wipe_command(cmd)

    def _handle_wipe_command(self, cmd: dict):
        """Apply the two-factor gate and, if it passes, invoke the wipe service."""
        param1 = cmd['params'][0]
        confirmation = cmd['confirmation']

        magic_ok = abs(param1 - float(self.wipe_magic_param1)) < 1e-6
        confirm_ok = confirmation >= 1

        if not (magic_ok and confirm_ok):
            self.get_logger().warn(
                f'Rejected emergency-wipe COMMAND_LONG '
                f'(param1={param1}, confirmation={confirmation}); '
                f'need param1=={self.wipe_magic_param1} and confirmation>=1')
            self._send_command_ack(self.wipe_command_id, mav.MAVResult.DENIED)
            return

        if not self._wipe_client.service_is_ready():
            self.get_logger().error('Emergency-wipe service not available')
            self._send_command_ack(self.wipe_command_id, mav.MAVResult.TEMPORARILY_REJECTED)
            self._send_statustext('EMERGENCY WIPE: service unavailable', int(MAVSeverity.CRITICAL))
            return

        self.get_logger().warn('Emergency-wipe command ACCEPTED — invoking wipe service')
        self._send_command_ack(self.wipe_command_id, mav.MAVResult.ACCEPTED)
        self._send_statustext('EMERGENCY WIPE: triggered', int(MAVSeverity.CRITICAL))

        future = self._wipe_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_wipe_response)

    def _on_wipe_response(self, future):
        """Report the service result back to QGC as STATUSTEXT."""
        try:
            response = future.result()
        except Exception as e:
            self._send_statustext(f'EMERGENCY WIPE: call failed: {e}', int(MAVSeverity.CRITICAL))
            return
        severity = MAVSeverity.INFO if response.success else MAVSeverity.CRITICAL
        self._send_statustext(f'EMERGENCY WIPE: {response.message}', int(severity))

    def _cb_wipe_status(self, msg: String):
        """Forward /emergency_wipe/status JSON to QGC as a STATUSTEXT summary."""
        try:
            status = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        message = status.get('message') or status.get('error') or json.dumps(status)
        severity = MAVSeverity.CRITICAL if status.get('error') else MAVSeverity.INFO
        self._send_statustext(f'WIPE STATUS: {message}', int(severity))

    # ===== Outbound MAVLink =====

    def _send_command_ack(self, command: int, result: int):
        payload = mav.build_command_ack(
            command, result, target_system=self.system_id, target_component=self.component_id)
        self._send_mavlink_frame(mav.MAVLINK_MSG_ID_COMMAND_ACK, payload)

    def _send_statustext(self, text: str, severity: int):
        payload = mav.build_statustext(text, severity)
        self._send_mavlink_frame(mav.MAVLINK_MSG_ID_STATUSTEXT, payload)

    def _send_mavlink_frame(self, msg_id: int, payload: bytes):
        """Send a spec-compliant MAVLink 2.0 frame to QGC's last-seen address."""
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
    """Entry point for emergency wipe MAVLink bridge."""
    rclpy.init(args=args)
    node = EmergencyWipeMAVLinkBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
