#!/usr/bin/env python3
"""
Integration Test: Fleet Manager Bridge -> QGC (real UDP)

Constructs the REAL FleetManagerMAVLinkBridge via its actual __init__ (real UDP
socket to a real listener standing in for QGC), drives its real
_cb_fleet_status callback with the double-encoded /fleet/status shape
fleet_manager_node produces, and verifies real STATUSTEXT bytes arrive.

rclpy/std_msgs stubs + the ros_params fixture come from tests/conftest.py.
"""

import json
import socket
import sys
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import mavlink_v2 as mav
from fleet_manager_mavlink_bridge import FleetManagerMAVLinkBridge


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
        'component_id': 200,
        'mavlink_host': 'localhost',
        'mavlink_port': qgc_listener.getsockname()[1],
    })
    b = FleetManagerMAVLinkBridge()
    yield b
    if b._socket is not None:
        b._socket.close()


def _fleet_msg(fleet: dict):
    m = types.SimpleNamespace()
    encoded = {did: (val if val == 'unknown' else json.dumps(val))
               for did, val in fleet.items()}
    m.data = json.dumps(encoded)
    return m


def _status(state='executing', cur=1, total=5, name='QGC Mission'):
    return {'state': state, 'message': '', 'mission_name': name,
            'total_waypoints': total, 'current_waypoint': cur, 'timestamp': 0}


def test_fleet_status_arrives_as_valid_statustext(bridge, qgc_listener):
    bridge._cb_fleet_status(_fleet_msg({'drone_1': _status('executing', 2, 5)}))
    data, _ = qgc_listener.recvfrom(1024)
    parsed = mav.parse_frame(data)
    assert parsed.valid
    assert parsed.msg_id == mav.MAVLINK_MSG_ID_STATUSTEXT
    text = parsed.payload[1:51].rstrip(b'\x00').decode('ascii', 'ignore')
    assert 'drone_1' in text and 'executing 2/5' in text


def test_two_drones_produce_two_frames(bridge, qgc_listener):
    bridge._cb_fleet_status(_fleet_msg({
        'drone_1': _status('executing', 1, 5),
        'drone_2': _status('idle', 0, 0, name=''),
    }))
    seen = []
    for _ in range(2):
        data, _ = qgc_listener.recvfrom(1024)
        p = mav.parse_frame(data)
        seen.append(p.payload[1:51].rstrip(b'\x00').decode('ascii', 'ignore'))
    assert any('drone_1' in t for t in seen)
    assert any('drone_2' in t for t in seen)


def test_unchanged_snapshot_sends_nothing_second_time(bridge, qgc_listener):
    snap = _fleet_msg({'drone_1': _status('executing', 1, 5)})
    bridge._cb_fleet_status(snap)
    qgc_listener.recvfrom(1024)  # first one arrives
    bridge._cb_fleet_status(snap)  # identical -> no send
    with pytest.raises(socket.timeout):
        qgc_listener.recvfrom(1024)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
