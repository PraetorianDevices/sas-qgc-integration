#!/usr/bin/env python3

"""
Fleet Manager Bridge: /fleet/status → MAVLink STATUSTEXT for QGroundControl

Subscribes to fleet_manager_node's aggregated `/fleet/status` topic and surfaces
each drone's mission state and waypoint progress to the operator as MAVLink
STATUSTEXT messages in QGC's message panel.

Why STATUSTEXT and not per-vehicle HEARTBEAT/telemetry: `/fleet/status` carries
only mission state and waypoint progress — no position, attitude, or battery
(those never flow through fleet_manager; see fleet_manager_node.py and
mission_executor_node.publish_mission_status). A STATUSTEXT summary is honest to
the data actually available and is always visible in QGC without a plugin.

Input topic: /fleet/status  (std_msgs/String, RELIABLE/VOLATILE)
  Payload is a JSON object keyed by drone_id. Each value is either the literal
  string "unknown" (drone hasn't reported yet) or a JSON *string* (double
  encoded) with the mission_executor status keys: state, message, mission_name,
  total_waypoints, current_waypoint, timestamp.

Output: MAVLink STATUSTEXT frames over UDP to QGC, one per drone whose summary
changed. `/fleet/status` re-emits the full fleet snapshot on every single
per-drone update, so we de-duplicate: a STATUSTEXT is sent for a drone only when
its summary line actually changes, otherwise the panel would flood with
identical lines.

All frame encoding goes through mavlink_v2.py (verified byte-for-byte against
pymavlink).
"""

import json
import socket
from typing import Optional, Dict
from enum import IntEnum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import mavlink_v2 as mav


class MAVSeverity(IntEnum):
    """MAV_SEVERITY, per the MAVLink common spec (lower = more severe).

    NB: these are the spec-correct values (EMERGENCY=0 .. DEBUG=7). This differs
    from gps_spoof_mavlink_bridge.py's MAVSeverity, whose numbering is
    transposed (its INFO=0 is really EMERGENCY, its CRITICAL=5 is really
    NOTICE) — a latent bug in that file, not something to replicate here.
    """
    EMERGENCY = 0
    ALERT = 1
    CRITICAL = 2
    ERROR = 3
    WARNING = 4
    NOTICE = 5
    INFO = 6
    DEBUG = 7


# Mission states that warrant a more prominent QGC severity than plain INFO.
_STATE_SEVERITY = {
    'error': MAVSeverity.CRITICAL,
    'interrupted': MAVSeverity.WARNING,
    'paused': MAVSeverity.WARNING,
}


class FleetManagerMAVLinkBridge(Node):
    """Forwards per-drone fleet status to QGroundControl as STATUSTEXT."""

    def __init__(self):
        super().__init__('fleet_manager_mavlink_bridge')

        self.declare_parameter('system_id', 1)
        self.declare_parameter('component_id', 200)  # SAS custom component
        self.declare_parameter('mavlink_host', 'localhost')
        self.declare_parameter('mavlink_port', 14550)

        self.system_id = self.get_parameter('system_id').value
        self.component_id = self.get_parameter('component_id').value
        mavlink_host = self.get_parameter('mavlink_host').value
        mavlink_port = self.get_parameter('mavlink_port').value

        self.get_logger().info(
            f'Fleet Manager MAVLink Bridge initialized: '
            f'system_id={self.system_id}, component_id={self.component_id}, '
            f'target={mavlink_host}:{mavlink_port}'
        )

        # UDP socket (outbound to QGC)
        self._socket: Optional[socket.socket] = None
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.connect((mavlink_host, mavlink_port))
            self.get_logger().info(f'Connected to MAVLink endpoint {mavlink_host}:{mavlink_port}')
        except OSError as e:
            self.get_logger().error(f'Failed to connect UDP socket: {e}')
            self._socket = None

        self._sequence = 0

        # Last summary line emitted per drone, so we only send a STATUSTEXT when
        # a drone's summary actually changes (see module docstring).
        self._last_summary: Dict[str, str] = {}

        # fleet_manager_node publishes /fleet/status with default (RELIABLE/
        # VOLATILE) QoS; match it.
        self.create_subscription(String, '/fleet/status', self._cb_fleet_status, 10)

        self.get_logger().info('Fleet Manager MAVLink Bridge started')

    def _cb_fleet_status(self, msg: String):
        """Parse the aggregated fleet snapshot and emit a STATUSTEXT for each
        drone whose summary has changed since we last reported it."""
        try:
            fleet = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warn(f'Malformed /fleet/status payload: {msg.data!r}')
            return

        if not isinstance(fleet, dict):
            self.get_logger().warn('/fleet/status payload is not a JSON object')
            return

        for drone_id, raw in fleet.items():
            summary, severity = self._summarize(drone_id, raw)
            if self._last_summary.get(drone_id) == summary:
                continue  # unchanged since last snapshot — don't re-spam QGC
            self._last_summary[drone_id] = summary
            self._send_statustext(summary, int(severity))

    def _summarize(self, drone_id: str, raw) -> tuple:
        """Build a (text, severity) summary line for one drone from its
        `/fleet/status` value. `raw` is either 'unknown' or a JSON string
        (double-encoded by fleet_manager) of the mission_executor status."""
        if raw == 'unknown' or raw is None:
            return (f'[FLEET] {drone_id}: unknown', MAVSeverity.NOTICE)

        try:
            status = json.loads(raw) if isinstance(raw, str) else raw
        except (json.JSONDecodeError, TypeError):
            return (f'[FLEET] {drone_id}: unparseable', MAVSeverity.WARNING)

        state = str(status.get('state', 'unknown'))
        cur = status.get('current_waypoint', 0)
        total = status.get('total_waypoints', 0)
        name = str(status.get('mission_name', '') or '')

        severity = _STATE_SEVERITY.get(state, MAVSeverity.INFO)

        text = f'[FLEET] {drone_id}: {state} {cur}/{total}'
        if name:
            text += f' ({name})'
        return (text[:50], severity)

    def _send_statustext(self, text: str, severity: int):
        """Send a MAVLink 2.0 STATUSTEXT to QGC."""
        if self._socket is None:
            return

        seq = self._sequence % 256
        self._sequence += 1

        payload = mav.build_statustext(text, severity)
        frame = mav.build_frame(mav.MAVLINK_MSG_ID_STATUSTEXT, seq, payload,
                                 self.system_id, self.component_id)
        try:
            self._socket.send(frame)
        except OSError as e:
            self.get_logger().warn(f'Failed to send MAVLink frame: {e}')

def main(args=None):
    """Entry point for fleet manager MAVLink bridge."""
    rclpy.init(args=args)
    node = FleetManagerMAVLinkBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
