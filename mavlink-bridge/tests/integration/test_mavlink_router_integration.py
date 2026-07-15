#!/usr/bin/env python3
"""
Integration Test: MAVLink Router -- real UDP sockets, simulated endpoints

Spins up the REAL MAVLinkRouterNode via its actual __init__ (real bound
external + downstream sockets, real background threads), then drives it from
plain UDP sockets standing in for QGC and for the inbound bridges behind it.
Verifies the core relay behavior end to end over real sockets: one inbound
frame fans out to every downstream target, and any downstream reply relays
back to whichever address most recently contacted the external socket.

The crown-jewel test -- the REAL mission_control_bridge and REAL
emergency_wipe_mavlink_bridge sharing this router simultaneously, resolving
the originally-flagged "QGC has one UDP link but two bridges need to bind"
limitation -- lives in test_router_with_real_bridges_integration.py.

rclpy stub + the ros_params fixture come from tests/conftest.py.
"""

import socket
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import mavlink_v2 as mav
from mavlink_router_node import MAVLinkRouterNode


@pytest.fixture
def qgc_socket():
    """Simulates QGroundControl's single UDP endpoint."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)
    yield sock
    sock.close()


@pytest.fixture
def downstream_sockets():
    """Two plain sockets standing in for the two inbound bridges (e.g.
    mission_control_bridge, emergency_wipe_mavlink_bridge) behind the router."""
    a = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    a.bind(('localhost', 0))
    a.settimeout(2.0)
    b = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    b.bind(('localhost', 0))
    b.settimeout(2.0)
    yield a, b
    a.close()
    b.close()


@pytest.fixture
def router(downstream_sockets, ros_params):
    # rclpy is a single shared stub module for the whole test session (see
    # tests/conftest.py); an earlier test file's teardown may have left
    # rclpy.ok() returning False, which would make this router's background
    # receive loops exit immediately without ever listening. Every fixture
    # that starts rclpy.ok()-gated background threads must set this itself
    # rather than assume a fresh True default -- same pattern as
    # test_mission_control_integration.py's and
    # test_emergency_wipe_integration.py's live_bridge fixtures.
    import rclpy
    rclpy.ok.return_value = True

    a, b = downstream_sockets
    ros_params.update({
        'mavlink_host': 'localhost',
        'mavlink_port': 0,  # ephemeral -- avoids port collisions between runs
        'downstream_bind_host': 'localhost',
        'downstream_bind_port': 0,
        'downstream_targets': [
            f'localhost:{a.getsockname()[1]}',
            f'localhost:{b.getsockname()[1]}',
        ],
    })
    r = MAVLinkRouterNode()
    yield r
    rclpy.ok.return_value = False
    r.destroy_node()


def _external_addr(router):
    return router._external_socket.getsockname()


def test_inbound_frame_fans_out_to_both_downstream_targets(router, qgc_socket, downstream_sockets):
    a, b = downstream_sockets
    frame = mav.build_frame(mav.MAVLINK_MSG_ID_HEARTBEAT, 0,
                             mav.build_heartbeat(2, 4, 0, 0, 4), 255, 0)
    qgc_socket.sendto(frame, _external_addr(router))

    data_a, _ = a.recvfrom(1024)
    data_b, _ = b.recvfrom(1024)
    assert data_a == frame
    assert data_b == frame


def test_downstream_reply_relays_back_to_qgc(router, qgc_socket, downstream_sockets):
    a, b = downstream_sockets
    qgc_socket.bind(('localhost', 0))  # claim a known ephemeral port to reply to

    # Establish "last QGC address" by sending an inbound frame first.
    trigger = mav.build_frame(mav.MAVLINK_MSG_ID_MISSION_COUNT, 0,
                              mav.build_mission_count(0), 255, 0)
    qgc_socket.sendto(trigger, _external_addr(router))
    a.recvfrom(1024)
    b.recvfrom(1024)

    reply = mav.build_frame(mav.MAVLINK_MSG_ID_MISSION_ACK, 1,
                            mav.build_mission_ack(mav.MAVMissionResult.ACCEPTED), 1, 1)
    a.sendto(reply, router._downstream_socket.getsockname())

    data, _ = qgc_socket.recvfrom(1024)
    assert data == reply


def test_no_relay_before_any_inbound_frame_seen(router, downstream_sockets):
    a, b = downstream_sockets
    reply = mav.build_frame(mav.MAVLINK_MSG_ID_STATUSTEXT, 0,
                            mav.build_statustext('too early', 6), 1, 1)
    a.sendto(reply, router._downstream_socket.getsockname())
    time.sleep(0.3)
    assert router._last_qgc_addr is None


def test_multiple_frames_all_fan_out_in_order(router, qgc_socket, downstream_sockets):
    a, b = downstream_sockets
    frames = [
        mav.build_frame(mav.MAVLINK_MSG_ID_HEARTBEAT, seq,
                        mav.build_heartbeat(2, 4, 0, 0, 4), 255, 0)
        for seq in range(5)
    ]
    for frame in frames:
        qgc_socket.sendto(frame, _external_addr(router))

    received_a = [a.recvfrom(1024)[0] for _ in frames]
    received_b = [b.recvfrom(1024)[0] for _ in frames]
    assert received_a == frames
    assert received_b == frames


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
