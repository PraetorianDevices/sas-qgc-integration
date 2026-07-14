#!/usr/bin/env python3
"""
Unit Test: Collision Avoidance Bridge (collision_mavlink_bridge.py)

Constructs the real CollisionMAVLinkBridge (bypassing __init__'s socket/ROS
setup) and drives its real _cb_obstacle_distance callback with realistic
px4_msgs/ObstacleDistance-shaped data, then validates the emitted bytes decode
as a valid MAVLink 2.0 OBSTACLE_DISTANCE via the real mavlink_v2 parser (and,
where installed, pymavlink).

rclpy/std_msgs/px4_msgs stubs come from tests/conftest.py.
"""

import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import mavlink_v2 as mav
from collision_mavlink_bridge import CollisionMAVLinkBridge


def _make_bridge(system_id=1, component_id=1):
    bridge = CollisionMAVLinkBridge.__new__(CollisionMAVLinkBridge)
    bridge.system_id = system_id
    bridge.component_id = component_id
    bridge._sequence = 0
    bridge.get_logger = lambda: MagicMock()
    return bridge


def _capture_socket(bridge):
    sent = []
    bridge._socket = types.SimpleNamespace(send=lambda f: sent.append(f))
    return sent


def _sweep(distances=None, increment=5.0, angle_offset=0.0, min_d=20, max_d=5000,
           sensor_type=0, frame=12, timestamp=123456):
    if distances is None:
        distances = [65535] * 72
    return types.SimpleNamespace(
        timestamp=timestamp, frame=frame, sensor_type=sensor_type,
        distances=distances, increment=increment, angle_offset=angle_offset,
        min_distance=min_d, max_distance=max_d)


class TestObstacleDistanceForwarding:

    def test_emits_one_valid_obstacle_distance_frame(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        bridge._cb_obstacle_distance(_sweep())

        assert len(sent) == 1
        parsed = mav.parse_frame(sent[0])
        assert parsed is not None
        assert parsed.valid
        assert parsed.msg_id == mav.MAVLINK_MSG_ID_OBSTACLE_DISTANCE

    def test_distances_and_metadata_roundtrip_via_pymavlink(self):
        pytest.importorskip('pymavlink.dialects.v20.common')
        from pymavlink.dialects.v20 import common as m2

        distances = [65535] * 72
        distances[0] = 150
        distances[18] = 300
        distances[36] = 4999

        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        bridge._cb_obstacle_distance(_sweep(distances=distances))

        decoded = m2.MAVLink(None).decode(bytearray(sent[0]))
        assert decoded.get_type() == 'OBSTACLE_DISTANCE'
        assert list(decoded.distances) == distances
        assert decoded.min_distance == 20
        assert decoded.max_distance == 5000
        assert decoded.frame == 12
        assert decoded.angle_offset == pytest.approx(0.0)

    def test_frame_carries_configured_system_and_component_id(self):
        bridge = _make_bridge(system_id=3, component_id=200)
        sent = _capture_socket(bridge)
        bridge._cb_obstacle_distance(_sweep())
        parsed = mav.parse_frame(sent[0])
        assert parsed.system_id == 3
        assert parsed.component_id == 200

    def test_float_increment_populates_increment_and_increment_f(self):
        pytest.importorskip('pymavlink.dialects.v20.common')
        from pymavlink.dialects.v20 import common as m2

        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        bridge._cb_obstacle_distance(_sweep(increment=5.0))
        decoded = m2.MAVLink(None).decode(bytearray(sent[0]))
        assert decoded.increment == 5           # integer u8
        assert decoded.increment_f == pytest.approx(5.0)  # float override

    def test_sequence_increments_across_sweeps(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        for _ in range(4):
            bridge._cb_obstacle_distance(_sweep())
        sequences = [mav.parse_frame(f).sequence for f in sent]
        assert sequences == [0, 1, 2, 3]

    def test_shorter_distances_array_padded_to_72(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        bridge._cb_obstacle_distance(_sweep(distances=[100, 200, 300]))
        parsed = mav.parse_frame(sent[0])
        assert parsed.valid
        assert parsed.msg_id == mav.MAVLINK_MSG_ID_OBSTACLE_DISTANCE

    def test_no_socket_does_not_raise(self):
        bridge = _make_bridge()
        bridge._socket = None
        bridge._cb_obstacle_distance(_sweep())  # must not raise


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
