#!/usr/bin/env python3

"""
Collision Avoidance Bridge: px4_msgs/ObstacleDistance → MAVLink for QGroundControl

Subscribes to the SF45 LiDAR sweep that sf45_px4_node publishes and forwards it
to QGroundControl as MAVLink OBSTACLE_DISTANCE (id 330), so the operator sees a
live 360° proximity/collision-risk view in QGC's proximity widget.

Input topic:  {prefix}/fmu/in/obstacle_distance  (px4_msgs/ObstacleDistance)
  NB: the topic is /fmu/IN/... — that is where sf45_px4_node actually publishes
  the sweep (companion → FMU direction). Earlier docs called it /fmu/out/...,
  a name nothing in SAS ever publishes; do not use it.

Output: MAVLink OBSTACLE_DISTANCE frames over UDP to QGC.

The px4_msgs ObstacleDistance maps essentially 1:1 onto the MAVLink message —
both use a 72-sector u16 distance array in centimetres (65535 = no obstacle),
a body-FRD frame (12), degrees-per-sector increment, and an angle offset.
All frame encoding goes through mavlink_v2.py (verified byte-for-byte against
pymavlink); the OBSTACLE_DISTANCE builder was added and verified the same way.
"""

import socket
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import ObstacleDistance

import mavlink_v2 as mav


class CollisionMAVLinkBridge(Node):
    """Forwards SF45 obstacle-distance sweeps to QGroundControl as MAVLink
    OBSTACLE_DISTANCE."""

    def __init__(self):
        super().__init__('collision_mavlink_bridge')

        self.declare_parameter('system_id', 1)
        self.declare_parameter('component_id', 1)  # MAV_COMP_ID_AUTOPILOT
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
            f'Collision MAVLink Bridge initialized: '
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

        # sf45_px4_node publishes with default (RELIABLE/VOLATILE) QoS; a
        # BEST_EFFORT subscriber matches a RELIABLE publisher under DDS rules,
        # and BEST_EFFORT is the right choice for high-rate sensor data.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            ObstacleDistance,
            f'{self.topic_prefix}/fmu/in/obstacle_distance',
            self._cb_obstacle_distance,
            qos,
        )

        self.get_logger().info('Collision MAVLink Bridge started')

    def _cb_obstacle_distance(self, msg: ObstacleDistance):
        """Translate a PX4 ObstacleDistance sweep into MAVLink OBSTACLE_DISTANCE
        and forward it to QGC."""
        # increment is float degrees in px4_msgs; MAVLink carries both an
        # integer `increment` (u8) and a float `increment_f` override. Populate
        # both consistently so QGC uses the precise float value.
        increment_f = float(msg.increment)
        increment = int(round(increment_f))

        payload = mav.build_obstacle_distance(
            time_usec=int(msg.timestamp),
            distances=list(msg.distances),
            increment=increment,
            min_distance=int(msg.min_distance),
            max_distance=int(msg.max_distance),
            increment_f=increment_f,
            angle_offset=float(msg.angle_offset),
            sensor_type=int(msg.sensor_type),
            frame=int(msg.frame),
        )
        self._send_mavlink_frame(mav.MAVLINK_MSG_ID_OBSTACLE_DISTANCE, payload)

    def _send_mavlink_frame(self, msg_id: int, payload: bytes):
        """Send a spec-compliant MAVLink 2.0 frame to QGC."""
        if self._socket is None:
            return

        seq = self._sequence % 256
        self._sequence += 1

        frame = mav.build_frame(msg_id, seq, payload, self.system_id, self.component_id)

        try:
            self._socket.send(frame)
        except OSError as e:
            self.get_logger().warn(f'Failed to send MAVLink frame: {e}')


def main(args=None):
    """Entry point for collision MAVLink bridge."""
    rclpy.init(args=args)
    node = CollisionMAVLinkBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
