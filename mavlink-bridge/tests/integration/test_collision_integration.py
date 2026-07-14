#!/usr/bin/env python3
"""
Integration Test: Collision Bridge -> QGC (real UDP)

Constructs the REAL CollisionMAVLinkBridge via its actual __init__ (a real UDP
socket connected to a real listening socket standing in for QGC), drives its
real _cb_obstacle_distance callback, and verifies genuine bytes arrive and
parse as valid MAVLink 2.0 OBSTACLE_DISTANCE.

rclpy/std_msgs/px4_msgs stubs + the ros_params fixture come from tests/conftest.py.
"""

import socket
import sys
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import mavlink_v2 as mav
from collision_mavlink_bridge import CollisionMAVLinkBridge


@pytest.fixture
def qgc_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('localhost', 0))
    sock.settimeout(2.0)
    yield sock
    sock.close()


@pytest.fixture
def bridge(qgc_listener, ros_params):
    ros_params.update({
        'system_id': 1,
        'component_id': 1,
        'mavlink_host': 'localhost',
        'mavlink_port': qgc_listener.getsockname()[1],
        'drone_id': '',
    })
    b = CollisionMAVLinkBridge()
    yield b
    if b._socket is not None:
        b._socket.close()


def _sweep(distances=None):
    if distances is None:
        distances = [65535] * 72
        distances[0] = 250
        distances[18] = 500
    return types.SimpleNamespace(
        timestamp=999, frame=12, sensor_type=0, distances=distances,
        increment=5.0, angle_offset=0.0, min_distance=20, max_distance=5000)


def test_obstacle_distance_arrives_as_valid_frame(bridge, qgc_listener):
    bridge._cb_obstacle_distance(_sweep())
    data, _ = qgc_listener.recvfrom(1024)
    parsed = mav.parse_frame(data)
    assert parsed is not None
    assert parsed.valid
    assert parsed.msg_id == mav.MAVLINK_MSG_ID_OBSTACLE_DISTANCE


def test_distances_survive_the_wire(bridge, qgc_listener):
    pytest.importorskip('pymavlink.dialects.v20.common')
    from pymavlink.dialects.v20 import common as m2
    distances = [65535] * 72
    distances[0] = 123
    distances[36] = 456
    bridge._cb_obstacle_distance(_sweep(distances))
    data, _ = qgc_listener.recvfrom(1024)
    decoded = m2.MAVLink(None).decode(bytearray(data))
    assert decoded.get_type() == 'OBSTACLE_DISTANCE'
    assert list(decoded.distances) == distances


def test_high_rate_sweeps_all_arrive_valid(bridge, qgc_listener):
    for i in range(10):
        d = [65535] * 72
        d[i] = 100 + i
        bridge._cb_obstacle_distance(_sweep(d))
    received = 0
    for _ in range(10):
        data, _ = qgc_listener.recvfrom(1024)
        assert mav.parse_frame(data).valid
        received += 1
    assert received == 10


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
