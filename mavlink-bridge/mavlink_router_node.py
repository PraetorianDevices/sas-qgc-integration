#!/usr/bin/env python3

"""
MAVLink Router: fans one external QGC UDP link out to multiple inbound bridges

Resolves the inbound single-UDP-port limitation: QGC uses one UDP comm link
per vehicle, but mission_control_bridge and emergency_wipe_mavlink_bridge each
need to BIND a socket to receive, and two processes cannot cleanly bind the
same UDP port. Previously they were parked on separate ports (14550 and
14556), which QGC's single link can't reach at once.

This node binds the single EXTERNAL port QGC actually connects to, and fans
every inbound datagram out, byte-for-byte, to a configurable list of internal
downstream targets (one per inbound bridge, each on its own port). It also
relays anything the bridges send back on its downstream-facing socket out to
whichever address last contacted the external socket.

No MAVLink parsing happens here -- it is a pure byte relay, exactly matching
how a real MAVLink bus works (every node sees every packet; each bridge
already filters for the message types it cares about and ignores the rest, so
broadcasting every inbound frame to every downstream bridge is safe).

Why neither mission_control_bridge.py nor emergency_wipe_mavlink_bridge.py
needed any code changes for this: both already (a) bind whatever `mavlink_port`
they are configured with, rather than assuming a fixed external port, and (b)
track their outbound reply address dynamically from the sender of the last
packet they received (see each file's `_reply_addr` / `_handle_mavlink_message`).
Pointed at the router's downstream socket instead of directly at QGC, that
existing mechanism keeps working unmodified: each bridge learns to reply to the
router, and the router relays the reply on to the real QGC address it most
recently heard from.
"""

import socket
import threading
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node


def parse_targets(raw: List[str]) -> List[Tuple[str, int]]:
    """Parse ['host:port', ...] downstream target strings into (host, port)
    tuples, skipping (with a warning left to the caller) any entry that isn't
    parseable rather than crashing node startup over one bad entry."""
    targets = []
    for entry in raw:
        host, sep, port_str = str(entry).rpartition(':')
        if not sep:
            continue
        try:
            targets.append((host, int(port_str)))
        except ValueError:
            continue
    return targets


class MAVLinkRouterNode(Node):
    """Fans one external QGC-facing UDP socket out to N internal inbound
    bridges, and relays their replies back to QGC."""

    def __init__(self):
        super().__init__('mavlink_router_node')

        self.declare_parameter('mavlink_host', 'localhost')
        self.declare_parameter('mavlink_bind_host', '0.0.0.0')
        self.declare_parameter('mavlink_port', 14550)
        self.declare_parameter('downstream_bind_host', 'localhost')
        self.declare_parameter('downstream_bind_port', 14559)
        self.declare_parameter('downstream_targets', ['localhost:14551', 'localhost:14556'])

        mavlink_host = self.get_parameter('mavlink_host').value
        mavlink_bind_host = self.get_parameter('mavlink_bind_host').value
        mavlink_port = self.get_parameter('mavlink_port').value
        downstream_bind_host = self.get_parameter('downstream_bind_host').value
        downstream_bind_port = self.get_parameter('downstream_bind_port').value
        targets_raw = self.get_parameter('downstream_targets').get_parameter_value().string_array_value

        self._downstream_targets = parse_targets(targets_raw)
        if not self._downstream_targets:
            self.get_logger().warn('No valid downstream_targets configured; router will forward nowhere')

        self.get_logger().info(
            f'MAVLink Router initialized: external bind={mavlink_bind_host}:{mavlink_port} '
            f'(QGC target={mavlink_host}:{mavlink_port}), '
            f'downstream_bind={downstream_bind_host}:{downstream_bind_port}, '
            f'targets={self._downstream_targets}'
        )

        # External (QGC-facing) socket: bound once, used for both receiving
        # from QGC and relaying replies back to it. Bound to mavlink_bind_host
        # (default 0.0.0.0), NOT mavlink_host -- the two are different
        # addresses whenever QGC runs on a different host/network namespace
        # than this node (e.g. QGC on native Windows, this node inside WSL2
        # NAT: mavlink_host is the Windows-side gateway IP outbound bridges
        # send to, which is never a locally-assignable bind address here).
        self._external_socket: Optional[socket.socket] = None
        try:
            self._external_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._external_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._external_socket.bind((mavlink_bind_host, mavlink_port))
            self._external_socket.settimeout(0.5)
        except OSError as e:
            self.get_logger().error(f'Failed to bind external UDP socket: {e}')
            self._external_socket = None

        # Downstream (bridge-facing) socket: bound once, used for both
        # forwarding inbound frames to every bridge and receiving their
        # outbound replies.
        self._downstream_socket: Optional[socket.socket] = None
        try:
            self._downstream_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._downstream_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._downstream_socket.bind((downstream_bind_host, downstream_bind_port))
            self._downstream_socket.settimeout(0.5)
        except OSError as e:
            self.get_logger().error(f'Failed to bind downstream UDP socket: {e}')
            self._downstream_socket = None

        # Address that last contacted the external socket -- where replies get
        # relayed back to. None until the first inbound packet arrives.
        self._last_qgc_addr: Optional[Tuple[str, int]] = None

        self._external_thread = threading.Thread(target=self._external_receive_loop, daemon=True)
        self._downstream_thread = threading.Thread(target=self._downstream_receive_loop, daemon=True)
        self._external_thread.start()
        self._downstream_thread.start()

        self.get_logger().info('MAVLink Router started')

    # ===== Background receive loops =====

    def _external_receive_loop(self):
        """Background thread: receive from QGC, forward to every downstream bridge."""
        while rclpy.ok():
            try:
                if self._external_socket is None:
                    time.sleep(0.1)
                    continue
                data, addr = self._external_socket.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            except Exception as e:
                self.get_logger().warn(f'Error in external receive loop: {e}')
                continue

            if addr != self._last_qgc_addr:
                self.get_logger().info(f'External socket now receiving from {addr}')
            self._last_qgc_addr = addr
            self._forward_to_downstream(data)

    def _downstream_receive_loop(self):
        """Background thread: receive a bridge's reply, relay it back to QGC."""
        while rclpy.ok():
            try:
                if self._downstream_socket is None:
                    time.sleep(0.1)
                    continue
                data, _ = self._downstream_socket.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            except Exception as e:
                self.get_logger().warn(f'Error in downstream receive loop: {e}')
                continue

            self._relay_to_qgc(data)

    # ===== Relay logic (pure, testable without real sockets) =====

    def _forward_to_downstream(self, data: bytes):
        """Fan `data` out, unchanged, to every configured downstream target."""
        if self._downstream_socket is None:
            return
        for target in self._downstream_targets:
            try:
                self._downstream_socket.sendto(data, target)
            except OSError as e:
                self.get_logger().warn(f'Failed to forward to {target}: {e}')

    def _relay_to_qgc(self, data: bytes):
        """Relay `data`, unchanged, back to the last address that contacted us."""
        if self._external_socket is None or self._last_qgc_addr is None:
            return
        try:
            self._external_socket.sendto(data, self._last_qgc_addr)
        except OSError as e:
            self.get_logger().warn(f'Failed to relay to QGC: {e}')

    def destroy_node(self):
        if self._external_socket is not None:
            self._external_socket.close()
        if self._downstream_socket is not None:
            self._downstream_socket.close()
        super().destroy_node()


def main(args=None):
    """Entry point for the MAVLink router node."""
    rclpy.init(args=args)
    node = MAVLinkRouterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
