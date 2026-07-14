#!/usr/bin/env python3
"""
Unit Test: Fleet Manager Bridge (fleet_manager_mavlink_bridge.py)

Constructs the real FleetManagerMAVLinkBridge (bypassing __init__) and drives
its real _cb_fleet_status callback with realistic /fleet/status payloads --
including the double-encoded JSON-string-per-drone shape fleet_manager_node
actually produces -- then validates the emitted STATUSTEXT frames via the real
mavlink_v2 parser.

rclpy/std_msgs stubs come from tests/conftest.py.
"""

import json
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import mavlink_v2 as mav
from fleet_manager_mavlink_bridge import FleetManagerMAVLinkBridge, MAVSeverity


class _DummyString:
    def __init__(self):
        self.data = ''


def _make_bridge(system_id=1, component_id=200):
    bridge = FleetManagerMAVLinkBridge.__new__(FleetManagerMAVLinkBridge)
    bridge.system_id = system_id
    bridge.component_id = component_id
    bridge._sequence = 0
    bridge._last_summary = {}
    bridge.get_logger = lambda: MagicMock()
    return bridge


def _capture_socket(bridge):
    sent = []
    bridge._socket = types.SimpleNamespace(send=lambda f: sent.append(f))
    return sent


def _fleet_msg(fleet: dict):
    """Build a /fleet/status String the way fleet_manager_node does: the outer
    dict's values are themselves JSON strings (double-encoded), or the literal
    'unknown'."""
    m = _DummyString()
    encoded = {did: (val if val == 'unknown' else json.dumps(val))
               for did, val in fleet.items()}
    m.data = json.dumps(encoded)
    return m


def _status(state='executing', cur=0, total=0, name='QGC Mission'):
    return {'state': state, 'message': '', 'mission_name': name,
            'total_waypoints': total, 'current_waypoint': cur, 'timestamp': 0}


def _texts(sent):
    out = []
    for f in sent:
        p = mav.parse_frame(f)
        out.append((p.payload[0], p.payload[1:51].rstrip(b'\x00').decode('ascii', 'ignore')))
    return out


class TestFleetStatusToStatustext:

    def test_emits_statustext_per_drone(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        bridge._cb_fleet_status(_fleet_msg({
            'drone_1': _status('executing', 2, 5),
            'drone_2': _status('idle', 0, 0, name=''),
        }))
        assert len(sent) == 2
        texts = [t for _, t in _texts(sent)]
        assert any('drone_1' in t and 'executing 2/5' in t for t in texts)
        assert any('drone_2' in t and 'idle 0/0' in t for t in texts)

    def test_all_frames_are_valid_statustext(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        bridge._cb_fleet_status(_fleet_msg({'drone_1': _status()}))
        parsed = mav.parse_frame(sent[0])
        assert parsed.valid
        assert parsed.msg_id == mav.MAVLINK_MSG_ID_STATUSTEXT

    def test_unknown_drone_reported_as_unknown(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        bridge._cb_fleet_status(_fleet_msg({'drone_1': 'unknown'}))
        _, text = _texts(sent)[0]
        assert 'drone_1' in text and 'unknown' in text

    def test_error_state_maps_to_critical_severity(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        bridge._cb_fleet_status(_fleet_msg({'drone_1': _status('error')}))
        severity, _ = _texts(sent)[0]
        assert severity == int(MAVSeverity.CRITICAL)

    def test_interrupted_state_maps_to_warning_severity(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        bridge._cb_fleet_status(_fleet_msg({'drone_1': _status('interrupted')}))
        severity, _ = _texts(sent)[0]
        assert severity == int(MAVSeverity.WARNING)

    def test_executing_state_maps_to_info_severity(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        bridge._cb_fleet_status(_fleet_msg({'drone_1': _status('executing')}))
        severity, _ = _texts(sent)[0]
        assert severity == int(MAVSeverity.INFO)


class TestDeduplication:
    """`/fleet/status` re-emits the whole snapshot on every per-drone update, so
    the bridge must only send a STATUSTEXT when a drone's summary changes."""

    def test_identical_snapshot_sends_nothing_the_second_time(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        snap = _fleet_msg({'drone_1': _status('executing', 1, 5)})
        bridge._cb_fleet_status(snap)
        assert len(sent) == 1
        bridge._cb_fleet_status(snap)  # unchanged
        assert len(sent) == 1

    def test_progress_change_triggers_new_statustext(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        bridge._cb_fleet_status(_fleet_msg({'drone_1': _status('executing', 1, 5)}))
        bridge._cb_fleet_status(_fleet_msg({'drone_1': _status('executing', 2, 5)}))
        assert len(sent) == 2

    def test_only_changed_drone_reemitted_in_mixed_snapshot(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        bridge._cb_fleet_status(_fleet_msg({
            'drone_1': _status('executing', 1, 5),
            'drone_2': _status('idle', 0, 0, name=''),
        }))
        assert len(sent) == 2
        # drone_1 advances, drone_2 unchanged -> exactly one new frame
        bridge._cb_fleet_status(_fleet_msg({
            'drone_1': _status('executing', 2, 5),
            'drone_2': _status('idle', 0, 0, name=''),
        }))
        assert len(sent) == 3
        _, text = _texts(sent)[-1]
        assert 'drone_1' in text and '2/5' in text


class TestMalformedInput:

    def test_malformed_outer_json_does_not_raise_or_send(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        bad = _DummyString()
        bad.data = '{not valid json'
        bridge._cb_fleet_status(bad)
        assert sent == []

    def test_non_object_payload_does_not_raise_or_send(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        arr = _DummyString()
        arr.data = '[1, 2, 3]'
        bridge._cb_fleet_status(arr)
        assert sent == []

    def test_unparseable_inner_value_reported_not_crashed(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        m = _DummyString()
        # outer is valid; inner value is a non-'unknown' string that isn't JSON
        m.data = json.dumps({'drone_1': 'garbage-not-json'})
        bridge._cb_fleet_status(m)
        severity, text = _texts(sent)[0]
        assert 'drone_1' in text and 'unparseable' in text

    def test_long_drone_summary_truncated_over_the_wire(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        bridge._cb_fleet_status(_fleet_msg({'drone_1': _status(name='X' * 100)}))
        parsed = mav.parse_frame(sent[0])
        text = parsed.payload[1:51].rstrip(b'\x00')
        assert len(text) <= 50


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
