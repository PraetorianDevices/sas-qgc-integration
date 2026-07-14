#!/usr/bin/env python3
"""
Live demo: the REAL GPSSpoofMAVLinkBridge (mavlink-bridge/gps_spoof_mavlink_bridge.py)
builds and sends genuine MAVLink 2.0 STATUSTEXT frames over a real UDP socket.
A receiver decodes them with pymavlink -- the same reference MAVLink 2.0
implementation QGroundControl itself is built on -- standing in for QGC's
parser. If frames decode cleanly here, QGC would accept them too; this is
exactly the check that would have failed before the Phase 0 protocol fix
(the old 7-byte header was not valid MAVLink 2.0 at all).

Run: python demo_qgc_wire_protocol.py
"""

import socket
import sys
import threading
import time
from pathlib import Path
from unittest.mock import MagicMock

sys.path.insert(0, str(Path(__file__).resolve().parent))

# Stub rclpy/std_msgs so gps_spoof_mavlink_bridge.py imports cleanly without a
# real ROS 2 environment -- same pattern the test suite's conftest.py uses.
rclpy_mock = MagicMock()
rclpy_mock.node.Node = object
sys.modules['rclpy'] = rclpy_mock
sys.modules['rclpy.node'] = rclpy_mock.node
sys.modules['rclpy.qos'] = MagicMock()


class _DummyString:
    def __init__(self):
        self.data = ''


std_msgs_mock = MagicMock()
std_msgs_mock.String = _DummyString
sys.modules['std_msgs'] = MagicMock()
sys.modules['std_msgs.msg'] = std_msgs_mock

from gps_spoof_mavlink_bridge import GPSSpoofMAVLinkBridge, MAVSeverity  # noqa: E402

PORT = 14550


def make_bridge() -> GPSSpoofMAVLinkBridge:
    """Real bridge, constructed the same way the unit tests do (bypassing
    __init__'s ROS 2 setup) but with a real, connected UDP socket."""
    bridge = GPSSpoofMAVLinkBridge.__new__(GPSSpoofMAVLinkBridge)
    bridge.system_id = 1
    bridge.component_id = 200
    bridge._sequence = 0
    bridge.get_logger = lambda: MagicMock()
    bridge._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    bridge._socket.connect(('localhost', PORT))
    return bridge


def run_receiver(stop_event: threading.Event, decoded_count: list):
    """Stands in for QGroundControl's parser: real pymavlink decoding real
    bytes arriving over a real socket."""
    from pymavlink.dialects.v20 import common as mavlink2

    mav = mavlink2.MAVLink(None)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('localhost', PORT))
    sock.settimeout(0.5)
    print(f"[receiver] bound to localhost:{PORT}, decoding with pymavlink "
          f"(the reference MAVLink 2.0 implementation)\n")

    while not stop_event.is_set():
        try:
            data, _ = sock.recvfrom(1024)
        except socket.timeout:
            continue
        try:
            msg = mav.decode(bytearray(data))
        except Exception as e:
            print(f"[receiver] FAILED TO DECODE: {e}  raw={data.hex()}")
            continue
        decoded_count[0] += 1
        text = msg.text.rstrip('\x00')
        print(f"[receiver] #{decoded_count[0]} decoded {msg.get_type()}  "
              f"sysid={msg.get_srcSystem()} compid={msg.get_srcComponent()}  "
              f"severity={msg.severity}  text={text!r}")

    sock.close()


def main():
    stop_event = threading.Event()
    decoded_count = [0]
    receiver = threading.Thread(target=run_receiver, args=(stop_event, decoded_count), daemon=True)
    receiver.start()
    time.sleep(0.3)  # let the receiver bind before we send

    bridge = make_bridge()
    print("[sender] frames built by the real GPSSpoofMAVLinkBridge._send_statustext\n")

    alerts = [
        (MAVSeverity.INFO, "GPS nominal - no spoofing detected"),
        (MAVSeverity.WARNING, "GPS/baro altitude mismatch - SUSPICIOUS"),
        (MAVSeverity.CRITICAL, "GPS SPOOFING DETECTED - heading divergence"),
    ]
    for severity, text in alerts:
        print(f"[sender] sending {severity.name}: {text!r}")
        bridge._send_statustext(text, int(severity))
        time.sleep(1.0)

    time.sleep(0.5)
    stop_event.set()
    receiver.join(timeout=2.0)
    bridge._socket.close()

    print(f"\n{decoded_count[0]}/{len(alerts)} frames sent were successfully "
          f"decoded as valid MAVLink 2.0 by pymavlink.")


if __name__ == '__main__':
    main()
