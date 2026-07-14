#!/usr/bin/env python3
"""
Integration Test: Emergency Wipe Bridge -- real UDP sockets + real receiver thread

Spins up the REAL EmergencyWipeMAVLinkBridge with its actual __init__ (real
bound UDP socket, real background receiver thread), then sends genuine MAVLink
COMMAND_LONG frames from a second UDP socket simulating QGC, and reads the real
COMMAND_ACK that comes back over the wire. This exercises the full inbound path
-- socket bind, the receive thread, frame parse, the two-factor gate, and the
outbound ACK addressing -- not just a direct method call.

The wipe service itself is the conftest rclpy-stub's MagicMock client (no real
Trigger server), so this test targets the MAVLink wire + gate + threading;
the service-call wiring is covered by the unit tests.

rclpy/std_msgs/std_srvs stubs + ros_params come from tests/conftest.py.
"""

import socket
import struct
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import mavlink_v2 as mav
from emergency_wipe_mavlink_bridge import EmergencyWipeMAVLinkBridge, MAV_CMD_USER_1


@pytest.fixture
def live_bridge(ros_params):
    import rclpy
    rclpy.ok.return_value = True

    ros_params.update({
        'system_id': 1,
        'component_id': 1,
        'mavlink_host': 'localhost',
        'mavlink_port': 0,  # ephemeral -- avoids port collisions between runs
        'drone_id': '',
        'wipe_command_id': MAV_CMD_USER_1,
        'wipe_magic_param1': 1.0,
    })

    bridge = EmergencyWipeMAVLinkBridge()
    bridge_port = bridge._socket.getsockname()[1]

    yield bridge, ('localhost', bridge_port)

    rclpy.ok.return_value = False
    if bridge._socket is not None:
        try:
            bridge._socket.close()
        except OSError:
            pass
    bridge._receiver_thread.join(timeout=2.0)


@pytest.fixture
def qgc_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)
    yield sock
    sock.close()


def _command_long(command=MAV_CMD_USER_1, param1=1.0, confirmation=1,
                  target_system=1, target_component=1):
    payload = struct.pack('<fffffffHBBB', param1, 0, 0, 0, 0, 0, 0,
                          command, target_system, target_component, confirmation)
    return mav.build_frame(mav.MAVLINK_MSG_ID_COMMAND_LONG, 0, payload, 255, 0)


def _recv_ack(sock):
    data, _ = sock.recvfrom(1024)
    parsed = mav.parse_frame(data)
    assert parsed.msg_id == mav.MAVLINK_MSG_ID_COMMAND_ACK
    payload = parsed.payload.ljust(3, b'\x00')
    command, result = struct.unpack('<HB', payload[:3])
    return command, result


def test_valid_command_acked_accepted_over_real_socket(live_bridge, qgc_socket):
    bridge, addr = live_bridge
    qgc_socket.sendto(_command_long(param1=1.0, confirmation=1), addr)
    command, result = _recv_ack(qgc_socket)
    assert command == MAV_CMD_USER_1
    assert result == int(mav.MAVResult.ACCEPTED)


def test_bad_gate_acked_denied_over_real_socket(live_bridge, qgc_socket):
    bridge, addr = live_bridge
    qgc_socket.sendto(_command_long(param1=0.0, confirmation=1), addr)
    command, result = _recv_ack(qgc_socket)
    assert result == int(mav.MAVResult.DENIED)


def test_service_unavailable_acked_temporarily_rejected(live_bridge, qgc_socket):
    bridge, addr = live_bridge
    bridge._wipe_client.service_is_ready.return_value = False
    qgc_socket.sendto(_command_long(param1=1.0, confirmation=1), addr)
    command, result = _recv_ack(qgc_socket)
    assert result == int(mav.MAVResult.TEMPORARILY_REJECTED)


def test_reply_addressed_to_actual_sender(live_bridge, qgc_socket):
    """The ACK must come back to the real ephemeral port our socket sent from,
    which only works if the bridge learns the reply address from the inbound
    packet rather than assuming a fixed port."""
    bridge, addr = live_bridge
    qgc_socket.bind(('localhost', 0))  # claim a specific ephemeral port
    qgc_socket.sendto(_command_long(param1=1.0, confirmation=1), addr)
    command, result = _recv_ack(qgc_socket)  # must arrive back here
    assert command == MAV_CMD_USER_1


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
