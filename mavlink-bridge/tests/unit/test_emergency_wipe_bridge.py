#!/usr/bin/env python3
"""
Unit Test: Emergency Wipe Bridge (emergency_wipe_mavlink_bridge.py)

Constructs the real EmergencyWipeMAVLinkBridge (bypassing __init__) and drives
its real _handle_mavlink_message / _handle_wipe_command path with genuine
MAVLink COMMAND_LONG frames built by the real mavlink_v2 codec. The focus is
the two-factor safety gate: a destructive wipe must fire ONLY when both the
magic param1 and the confirmation byte are correct, and must be answered with
the right COMMAND_ACK result in every case.

rclpy/std_msgs/std_srvs stubs come from tests/conftest.py.
"""

import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import mavlink_v2 as mav
from emergency_wipe_mavlink_bridge import EmergencyWipeMAVLinkBridge, MAV_CMD_USER_1


def _make_bridge(command_id=MAV_CMD_USER_1, magic=1.0, system_id=1,
                 component_id=1, service_ready=True):
    bridge = EmergencyWipeMAVLinkBridge.__new__(EmergencyWipeMAVLinkBridge)
    bridge.system_id = system_id
    bridge.component_id = component_id
    bridge._sequence = 0
    bridge._reply_addr = ('localhost', 14550)
    bridge.wipe_command_id = command_id
    bridge.wipe_magic_param1 = magic
    bridge.get_logger = lambda: MagicMock()

    client = MagicMock()
    client.service_is_ready.return_value = service_ready
    bridge._wipe_client = client
    return bridge


def _capture_socket(bridge):
    sent = []
    bridge._socket = types.SimpleNamespace(sendto=lambda f, addr: sent.append(f))
    return sent


def _command_long(command=MAV_CMD_USER_1, param1=1.0, confirmation=1,
                  target_system=1, target_component=1, params_rest=None):
    rest = params_rest or [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    payload = _pack_command_long(param1, rest, command, target_system,
                                 target_component, confirmation)
    return mav.build_frame(mav.MAVLINK_MSG_ID_COMMAND_LONG, 0, payload, 255, 0)


def _pack_command_long(param1, rest, command, ts, tc, confirmation):
    import struct
    p = [param1] + list(rest)
    return struct.pack('<fffffffHBBB', *p, command, ts, tc, confirmation)


def _acks(sent):
    """Return list of (command, result) from COMMAND_ACK frames in `sent`."""
    out = []
    for f in sent:
        parsed = mav.parse_frame(f)
        if parsed.msg_id == mav.MAVLINK_MSG_ID_COMMAND_ACK:
            import struct
            payload = parsed.payload.ljust(3, b'\x00')
            command, result = struct.unpack('<HB', payload[:3])
            out.append((command, result))
    return out


class TestSafetyGate:

    def test_valid_command_accepted_and_service_called(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        bridge._handle_mavlink_message(_command_long(param1=1.0, confirmation=1))

        assert bridge._wipe_client.call_async.called
        acks = _acks(sent)
        assert (MAV_CMD_USER_1, int(mav.MAVResult.ACCEPTED)) in acks

    def test_wrong_magic_param1_denied_and_service_not_called(self):
        bridge = _make_bridge(magic=1.0)
        sent = _capture_socket(bridge)
        bridge._handle_mavlink_message(_command_long(param1=0.0, confirmation=1))

        assert not bridge._wipe_client.call_async.called
        assert (MAV_CMD_USER_1, int(mav.MAVResult.DENIED)) in _acks(sent)

    def test_zero_confirmation_denied_and_service_not_called(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        bridge._handle_mavlink_message(_command_long(param1=1.0, confirmation=0))

        assert not bridge._wipe_client.call_async.called
        assert (MAV_CMD_USER_1, int(mav.MAVResult.DENIED)) in _acks(sent)

    def test_both_factors_required(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        # magic ok but confirmation 0
        bridge._handle_mavlink_message(_command_long(param1=1.0, confirmation=0))
        # confirmation ok but magic wrong
        bridge._handle_mavlink_message(_command_long(param1=2.0, confirmation=1))
        assert not bridge._wipe_client.call_async.called
        results = [r for _, r in _acks(sent)]
        assert results == [int(mav.MAVResult.DENIED), int(mav.MAVResult.DENIED)]

    def test_custom_magic_value_respected(self):
        bridge = _make_bridge(magic=42.0)
        sent = _capture_socket(bridge)
        bridge._handle_mavlink_message(_command_long(param1=42.0, confirmation=1))
        assert bridge._wipe_client.call_async.called
        assert (MAV_CMD_USER_1, int(mav.MAVResult.ACCEPTED)) in _acks(sent)


class TestCommandFiltering:

    def test_non_wipe_command_id_ignored_no_ack(self):
        bridge = _make_bridge(command_id=31010)
        sent = _capture_socket(bridge)
        bridge._handle_mavlink_message(
            _command_long(command=400, param1=1.0, confirmation=1))  # MAV_CMD_COMPONENT_ARM_DISARM
        assert not bridge._wipe_client.call_async.called
        assert sent == []  # we do not ACK commands that aren't ours

    def test_command_addressed_to_other_system_ignored(self):
        bridge = _make_bridge(system_id=1)
        sent = _capture_socket(bridge)
        bridge._handle_mavlink_message(
            _command_long(param1=1.0, confirmation=1, target_system=99))
        assert not bridge._wipe_client.call_async.called
        assert sent == []

    def test_broadcast_target_system_zero_is_honored(self):
        bridge = _make_bridge(system_id=1)
        sent = _capture_socket(bridge)
        bridge._handle_mavlink_message(
            _command_long(param1=1.0, confirmation=1, target_system=0))
        assert bridge._wipe_client.call_async.called

    def test_non_command_long_frame_ignored(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        # a STATUSTEXT frame is not a COMMAND_LONG
        frame = mav.build_frame(mav.MAVLINK_MSG_ID_STATUSTEXT, 0,
                                 mav.build_statustext('hi', 6), 255, 0)
        bridge._handle_mavlink_message(frame)
        assert not bridge._wipe_client.call_async.called
        assert sent == []

    def test_invalid_crc_frame_ignored(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        frame = bytearray(_command_long(param1=1.0, confirmation=1))
        frame[-1] ^= 0xFF  # corrupt CRC
        bridge._handle_mavlink_message(bytes(frame))
        assert not bridge._wipe_client.call_async.called
        assert sent == []


class TestServiceAvailability:

    def test_service_not_ready_temporarily_rejected(self):
        bridge = _make_bridge(service_ready=False)
        sent = _capture_socket(bridge)
        bridge._handle_mavlink_message(_command_long(param1=1.0, confirmation=1))
        assert not bridge._wipe_client.call_async.called
        assert (MAV_CMD_USER_1, int(mav.MAVResult.TEMPORARILY_REJECTED)) in _acks(sent)


class TestWipeStatusForwarding:

    def test_status_message_forwarded_as_statustext(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        msg = types.SimpleNamespace(data='{"message": "wipe complete", "all_succeeded": true}')
        bridge._cb_wipe_status(msg)
        parsed = mav.parse_frame(sent[0])
        assert parsed.msg_id == mav.MAVLINK_MSG_ID_STATUSTEXT
        text = parsed.payload[1:51].rstrip(b'\x00').decode('ascii', 'ignore')
        assert 'wipe complete' in text

    def test_error_status_uses_critical_severity(self):
        from emergency_wipe_mavlink_bridge import MAVSeverity
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        msg = types.SimpleNamespace(data='{"error": "disk unreadable"}')
        bridge._cb_wipe_status(msg)
        parsed = mav.parse_frame(sent[0])
        assert parsed.payload[0] == int(MAVSeverity.CRITICAL)

    def test_malformed_status_json_ignored(self):
        bridge = _make_bridge()
        sent = _capture_socket(bridge)
        bridge._cb_wipe_status(types.SimpleNamespace(data='not json'))
        assert sent == []


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
