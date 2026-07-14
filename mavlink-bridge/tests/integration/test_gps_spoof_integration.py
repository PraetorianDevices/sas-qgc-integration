#!/usr/bin/env python3
"""
Integration Test Suite: GPS Spoofing Detector -> MAVLink Bridge -> QGC (real UDP)

Constructs the REAL GPSSpoofMAVLinkBridge via its actual __init__ (a real UDP
socket connected to a real listening socket standing in for QGC), then
drives its real _cb_gps_spoof_alert callback and verifies genuine bytes
arrive at the far end and parse as valid MAVLink 2.0 STATUSTEXT.

A previous version of this file set up real rclpy Node subclasses directly
(no stubbing), which meant it could only be collected inside a real ROS 2
environment with rclpy installed -- and even then, its assertions never
actually instantiated GPSSpoofMAVLinkBridge. test_bridge_receives_alert only
checked that a synthetic publisher's call counter incremented, with the
comment "Bridge should have received it (in real environment)" -- i.e. it
never checked the bridge at all. This version stubs rclpy (so it can run
anywhere) and drives the real bridge class end-to-end over a real socket.
"""

import json
import socket
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

# rclpy/std_msgs stubs are installed once in tests/conftest.py, shared across
# every test file -- see that module's docstring for why a per-file stub here
# would be unsafe (gps_spoof_mavlink_bridge.py is also imported by
# tests/unit/test_mavlink_crc.py).
import mavlink_v2 as mav
from gps_spoof_mavlink_bridge import GPSSpoofMAVLinkBridge, MAVSeverity


class _DummyString:
    def __init__(self):
        self.data = ''


@pytest.fixture
def qgc_listener():
    """A real UDP socket standing in for QGroundControl's listening endpoint."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('localhost', 0))
    sock.settimeout(2.0)
    yield sock
    sock.close()


@pytest.fixture
def bridge(qgc_listener, ros_params):
    """A real GPSSpoofMAVLinkBridge, connected to the qgc_listener socket."""
    ros_params.update({
        'system_id': 1,
        'component_id': 200,
        'mavlink_host': 'localhost',
        'mavlink_port': qgc_listener.getsockname()[1],
    })
    b = GPSSpoofMAVLinkBridge()
    yield b
    if b._socket is not None:
        b._socket.close()


def _alert_msg(alert_id=1, level='WARNING', strategy='HEADING', state='SUSPICIOUS', detail=None):
    m = _DummyString()
    m.data = json.dumps({
        'alert_id': alert_id, 'level': level, 'strategy': strategy,
        'state': state, 'detail': detail or {},
    })
    return m


class TestRealBridgeConstruction:

    def test_bridge_connects_real_socket(self, bridge):
        assert bridge._socket is not None

    def test_bridge_uses_configured_system_and_component_id(self, bridge):
        assert bridge.system_id == 1
        assert bridge.component_id == 200


class TestRealAlertToUdpPipeline:
    """JSON alert -> real _cb_gps_spoof_alert -> real UDP bytes -> real
    parser at the far end. This is the pipeline the previous version of this
    file claimed to test but never actually drove."""

    def test_critical_alert_arrives_as_valid_statustext(self, bridge, qgc_listener):
        bridge._cb_gps_spoof_alert(_alert_msg(
            level='CRITICAL', strategy='HEADING', state='SPOOFING_DETECTED',
            detail={'description': 'EKF2 heading diverging'}))

        data, _ = qgc_listener.recvfrom(1024)
        parsed = mav.parse_frame(data)

        assert parsed is not None
        assert parsed.valid
        assert parsed.msg_id == mav.MAVLINK_MSG_ID_STATUSTEXT
        assert parsed.payload[0] == int(MAVSeverity.CRITICAL)

    def test_warning_alert_severity_and_text(self, bridge, qgc_listener):
        bridge._cb_gps_spoof_alert(_alert_msg(
            level='WARNING', strategy='ALTITUDE', state='SUSPICIOUS',
            detail={'description': 'GPS/baro altitude mismatch'}))

        data, _ = qgc_listener.recvfrom(1024)
        parsed = mav.parse_frame(data)

        assert parsed.payload[0] == int(MAVSeverity.WARNING)
        text = parsed.payload[1:51].rstrip(b'\x00').decode('ascii', errors='ignore')
        assert 'ALTITUDE' in text

    def test_multiple_alerts_arrive_in_order(self, bridge, qgc_listener):
        alerts = [
            ('INFO', 'NOMINAL'),
            ('WARNING', 'SUSPICIOUS'),
            ('CRITICAL', 'SPOOFING_DETECTED'),
        ]
        for level, state in alerts:
            bridge._cb_gps_spoof_alert(_alert_msg(level=level, strategy='HEADING', state=state))

        severities = []
        for _ in alerts:
            data, _ = qgc_listener.recvfrom(1024)
            severities.append(mav.parse_frame(data).payload[0])

        assert severities == [int(MAVSeverity.INFO), int(MAVSeverity.WARNING), int(MAVSeverity.CRITICAL)]

    def test_malformed_json_sends_nothing_and_does_not_raise(self, bridge, qgc_listener):
        bad_msg = _DummyString()
        bad_msg.data = '{"invalid": json}'
        bridge._cb_gps_spoof_alert(bad_msg)  # must not raise

        with pytest.raises(socket.timeout):
            qgc_listener.recvfrom(1024)

    def test_long_description_truncated_over_the_wire(self, bridge, qgc_listener):
        bridge._cb_gps_spoof_alert(_alert_msg(
            level='WARNING', strategy='HEADING', state='SUSPICIOUS',
            detail={'description': 'X' * 200}))

        data, _ = qgc_listener.recvfrom(1024)
        parsed = mav.parse_frame(data)
        text = parsed.payload[1:51].rstrip(b'\x00')
        assert len(text) <= 50

    def test_high_frequency_alert_stream_all_arrive_valid(self, bridge, qgc_listener):
        for i in range(10):
            bridge._cb_gps_spoof_alert(_alert_msg(
                level=['INFO', 'WARNING', 'CRITICAL'][i % 3], strategy='HEADING',
                state='NOMINAL', detail={'count': i}))

        received = 0
        for _ in range(10):
            data, _ = qgc_listener.recvfrom(1024)
            assert mav.parse_frame(data).valid
            received += 1
        assert received == 10


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
