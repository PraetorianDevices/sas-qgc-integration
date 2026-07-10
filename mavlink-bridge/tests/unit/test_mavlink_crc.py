#!/usr/bin/env python3
"""
Unit Test: GPS Spoofing Bridge MAVLink Frame Generation (real module)

Tests the real GPSSpoofMAVLinkBridge's STATUSTEXT frame generation, via
mavlink_v2's verified codec. A previous version of this file called
bridge._build_statustext_payload/_build_mavlink_frame/_compute_mavlink_crc --
those methods were removed when gps_spoof_mavlink_bridge.py was migrated to
mavlink_v2.py, so that version would now raise AttributeError outright. Its
assertions were also written against the old 7-byte frame header (e.g.
`frame[3] == msg_id`), which never matched real MAVLink 2.0 in the first
place.

This file also absorbs the previously-separate test_mavlink_frame_generation.py,
which reimplemented the identical (broken) frame logic in a standalone
MAVLinkFrameBuilder class and never imported real code at all -- keeping two
parallel test files for the same functionality, one fake and one real, was
worse than consolidating into one that's real.
"""

import json
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def _install_stubs():
    if 'gps_spoof_mavlink_bridge' in sys.modules:
        return
    rclpy_mock = MagicMock()
    rclpy_mock.node.Node = object
    sys.modules['rclpy'] = rclpy_mock
    sys.modules['rclpy.node'] = rclpy_mock.node
    sys.modules['rclpy.qos'] = MagicMock()
    sys.modules['std_msgs'] = MagicMock()
    sys.modules['std_msgs.msg'] = MagicMock()


_install_stubs()

import mavlink_v2 as mav
from gps_spoof_mavlink_bridge import GPSSpoofMAVLinkBridge, MAVSeverity


class _DummyString:
    def __init__(self):
        self.data = ''


def _make_bridge(system_id=1, component_id=200):
    bridge = GPSSpoofMAVLinkBridge.__new__(GPSSpoofMAVLinkBridge)
    bridge.system_id = system_id
    bridge.component_id = component_id
    bridge._sequence = 0
    bridge.get_logger = lambda: MagicMock()
    return bridge


def _capture_socket(bridge):
    sent = []
    bridge._socket = types.SimpleNamespace(send=lambda f: sent.append(f))
    return sent


def _alert_msg(alert_id=1, level='WARNING', strategy='HEADING', state='SUSPICIOUS', detail=None):
    m = _DummyString()
    m.data = json.dumps({
        'alert_id': alert_id, 'level': level, 'strategy': strategy,
        'state': state, 'detail': detail or {},
    })
    return m


class TestStatustextFrameStructure:
    """Real GPSSpoofMAVLinkBridge._send_statustext output, validated via the
    real mavlink_v2 parser -- not byte-offset assumptions about a 7-byte
    header."""

    def test_frame_passes_crc_validation(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)

        bridge._send_statustext("GPS SPOOF DETECTED: heading divergence", int(MAVSeverity.CRITICAL))

        assert len(sent) == 1
        parsed = mav.parse_frame(sent[0])
        assert parsed is not None
        assert parsed.valid

    def test_frame_identifies_as_statustext(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        bridge._send_statustext("Test message", int(MAVSeverity.WARNING))

        parsed = mav.parse_frame(sent[0])
        assert parsed.msg_id == mav.MAVLINK_MSG_ID_STATUSTEXT

    def test_frame_carries_configured_system_and_component_id(self):
        bridge = _make_bridge(system_id=1, component_id=200)
        sent = _capture_socket(bridge)
        bridge._send_statustext("Test message", int(MAVSeverity.WARNING))

        parsed = mav.parse_frame(sent[0])
        assert parsed.system_id == 1
        assert parsed.component_id == 200

    def test_statustext_payload_roundtrips_severity_and_text(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        bridge._send_statustext("GPS altitude spoofed", int(MAVSeverity.CRITICAL))

        parsed = mav.parse_frame(sent[0])
        # severity(u8) + text(char[50], truncated) -- decode directly, same
        # layout mavlink_v2.build_statustext produces.
        severity = parsed.payload[0]
        text = parsed.payload[1:51].rstrip(b'\x00').decode('ascii', errors='ignore')
        assert severity == int(MAVSeverity.CRITICAL)
        assert text == "GPS altitude spoofed"

    def test_long_message_truncated_to_50_chars(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        long_text = "A" * 100
        bridge._send_statustext(long_text, int(MAVSeverity.WARNING))

        parsed = mav.parse_frame(sent[0])
        text = parsed.payload[1:51].rstrip(b'\x00').decode('ascii', errors='ignore')
        assert len(text) == 50
        assert text == "A" * 50

    def test_sequence_increments_across_calls(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        for _ in range(5):
            bridge._send_statustext("test", int(MAVSeverity.INFO))

        sequences = [mav.parse_frame(f).sequence for f in sent]
        assert sequences == [0, 1, 2, 3, 4]

    def test_sequence_wraps_at_256(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        for _ in range(260):
            bridge._send_statustext("test", int(MAVSeverity.INFO))

        sequences = [mav.parse_frame(f).sequence for f in sent]
        assert sequences[255] == 255
        assert sequences[256] == 0
        assert sequences[259] == 3

    def test_no_socket_does_not_raise(self):
        bridge = _make_bridge()
        bridge._socket = None
        bridge._send_statustext("test", int(MAVSeverity.WARNING))  # must not raise


class TestGpsSpoofAlertPipelineReal:
    """End-to-end: JSON alert in -> real MAVLink STATUSTEXT frame out, via
    the real _cb_gps_spoof_alert callback."""

    def test_spoofing_detected_maps_to_critical_severity(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)

        bridge._cb_gps_spoof_alert(_alert_msg(
            level='CRITICAL', strategy='HEADING', state='SPOOFING_DETECTED',
            detail={'description': 'EKF2 heading diverging'}))

        parsed = mav.parse_frame(sent[0])
        assert parsed.payload[0] == int(MAVSeverity.CRITICAL)

    def test_suspicious_maps_to_warning_severity(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)

        bridge._cb_gps_spoof_alert(_alert_msg(
            level='WARNING', strategy='ALTITUDE', state='SUSPICIOUS',
            detail={'description': 'GPS/baro altitude mismatch'}))

        parsed = mav.parse_frame(sent[0])
        assert parsed.payload[0] == int(MAVSeverity.WARNING)

    def test_nominal_maps_to_info_severity(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)

        bridge._cb_gps_spoof_alert(_alert_msg(
            level='INFO', strategy='HEADING', state='NOMINAL', detail={}))

        parsed = mav.parse_frame(sent[0])
        assert parsed.payload[0] == int(MAVSeverity.INFO)

    def test_malformed_json_does_not_raise(self):
        bridge = _make_bridge()
        _capture_socket(bridge)
        bad_msg = _DummyString()
        bad_msg.data = '{"invalid": json}'
        bridge._cb_gps_spoof_alert(bad_msg)  # must not raise

    def test_alert_text_includes_strategy_and_description(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)

        bridge._cb_gps_spoof_alert(_alert_msg(
            level='CRITICAL', strategy='HEADING', state='SPOOFING_DETECTED',
            detail={'description': 'diverging'}))

        parsed = mav.parse_frame(sent[0])
        text = parsed.payload[1:51].rstrip(b'\x00').decode('ascii', errors='ignore')
        assert 'HEADING' in text
        assert 'SPOOF DETECTED' in text

    def test_multiple_alerts_produce_valid_sequential_frames(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)

        alerts = [
            ('INFO', 'HEADING', 'NOMINAL'),
            ('WARNING', 'HEADING', 'SUSPICIOUS'),
            ('CRITICAL', 'HEADING', 'SPOOFING_DETECTED'),
            ('INFO', 'HEADING', 'NOMINAL'),
        ]
        for level, strategy, state in alerts:
            bridge._cb_gps_spoof_alert(_alert_msg(level=level, strategy=strategy, state=state))

        assert len(sent) == 4
        for frame in sent:
            assert mav.parse_frame(frame).valid


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
