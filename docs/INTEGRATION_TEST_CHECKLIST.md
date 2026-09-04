# GPS Spoofing Detector → QGC Integration — Test Checklist

## Status: Phase 5 (live QGC) has since been run — ✅ passed

This checklist originally predated the Phase 0 MAVLink protocol audit, and warned that its
Phase 5 (real QGroundControl connection) could not have passed, because the bridge was
emitting structurally invalid MAVLink 2.0 frames that QGC's parser would never recognise.
That is long fixed, and **Phase 5 has now actually been run against real QGroundControl and
real PX4 SITL, successfully** — see `IMPLEMENTATION_STATUS.md` Part 5.

Doing so found three further bugs on the GPS-spoofing/inbound path that this checklist's
steps do not probe, and which are worth knowing about before you re-run it:

- **QGC speaks MAVLink 1.0 on link-up**, and the parser accepted only 2.0, so every inbound
  message was discarded silently. Fixed — the parser now accepts both.
- **QGC must target the WSL2 interface IP, not `localhost`.** WSL2's localhost forwarding
  drops UDP. If Phase 5 appears to do nothing, check this first.
- **PX4's own mavlink instance on port 18570 contends with `mavlink_router_node`** on 14550.
  Run `mavlink stop -u 18570` at the `pxh>` prompt after every fresh SITL boot.

Earlier revisions of this checklist also fixed:
- **Phase 2/3.3**: test names and file paths updated to match the current (real-module,
  real-socket) test suite — the old ones referenced a deleted file (`test_integration.py`)
  and tests that never actually drove the bridge.
- **Phase 4.3**: the manual packet-parsing script used the old, wrong byte offsets (`msg_id`
  at byte 3, `seq` at byte 6), fixed to the real 10-byte header layout — otherwise every
  field it reports would be silently wrong. Note it decodes **v2** frames; QGC's own
  outbound traffic may be v1, with a 6-byte header.

`mavlink-bridge/test_gps_spoof_alert_generator.py` exists, so Phases 5.2/5.4/7.1 can use
`ros2 run mavlink-bridge test_gps_spoof_alert_generator [--count N] [--rate HZ] [--level LEVEL]`
instead of the inline snippets. The inline snippets remain below as a no-build fallback.

---

## Phase 1: Environment Setup

- [ ] ROS 2 Jazzy installed and sourced
- [ ] SAS package built: `colcon build --packages-select my_python_package`
- [ ] mavlink-bridge package built: `colcon build --packages-select mavlink-bridge`
- [ ] QGroundControl installed (for end-to-end test)
- [ ] Port 14550 is available (check with `lsof -i :14550` or `netstat -an | grep 14550`)
- [ ] `pymavlink` installed if you want to cross-check captured packets against a reference decoder: `pip install pymavlink`

## Phase 2: Unit Tests (Already Passing ✓ as of this session — re-run to confirm in your environment)

Run: `cd mavlink-bridge && python -m pytest tests/unit/test_mavlink_v2.py tests/unit/test_mavlink_crc.py -v`

- [x] `test_mavlink_v2.py` (22 tests) — every MAVLink message type the bridges use, byte-for-byte identical to `pymavlink`'s own encoder output; frame header structure (10-byte header, 24-bit message ID); CRC corruption detection; MAVLink 2 trailing-zero-truncation edge cases
- [x] `test_mavlink_crc.py` (14 tests, real `GPSSpoofMAVLinkBridge`) — frame CRC validity, STATUSTEXT severity/text roundtrip, 50-char truncation, sequence increment and 256-wraparound, full alert-JSON-to-frame pipeline (INFO/WARNING/CRITICAL severity mapping)

Both files stub `rclpy` so they run without a ROS 2 environment — useful for quick local iteration before doing the ROS 2/UDP-level phases below.

## Phase 3: ROS 2 Topic Integration Tests

### 3.1 Verify Detector Publishing

```bash
# Terminal 1: Launch detector
ros2 run my_python_package gps_spoof_detector_node --ros-args -p system_id:=1

# Terminal 2: Monitor /gps_spoof_alert topic
ros2 topic echo /gps_spoof_alert

# Expected output: JSON alert messages appearing at ~10 Hz when detector is active
```

**Validation:**
- [ ] `/gps_spoof_alert` topic exists (check: `ros2 topic list | grep gps_spoof`)
- [ ] Alerts are JSON-formatted (check: `ros2 topic echo /gps_spoof_alert --once`)
- [ ] Alert has all required fields:
  - [ ] `alert_id` (integer)
  - [ ] `level` (INFO | WARNING | CRITICAL)
  - [ ] `strategy` (HEADING | ALTITUDE | PX4_INTERNAL)
  - [ ] `state` (NOMINAL | SUSPICIOUS | SPOOFING_DETECTED)
  - [ ] `detail` (dictionary with strategy-specific data)
  - [ ] `timestamp_us` (microsecond timestamp)

### 3.2 Verify Bridge Subscription

```bash
# Terminal 1: Launch full pipeline
ros2 launch mavlink-bridge launch_gps_spoof_qgc.py system_id:=1

# Terminal 2: Check nodes
ros2 node list

# Terminal 3: Monitor bridge log
ros2 node info /gps_spoof_mavlink_bridge

# Terminal 4: Verify subscriptions
ros2 topic info /gps_spoof_alert
```

**Validation:**
- [ ] `/gps_spoof_detector_node` is listed in `ros2 node list`
- [ ] `/gps_spoof_mavlink_bridge` is listed in `ros2 node list`
- [ ] Bridge logs show "GPS Spoof MAVLink Bridge initialized"
- [ ] Bridge logs show "Connected to MAVLink endpoint localhost:14550"
- [ ] `/gps_spoof_alert` has 1 subscriber (the bridge)

### 3.3 Run Integration Test Suite

```bash
cd mavlink-bridge
python -m pytest tests/integration/test_gps_spoof_integration.py -v
```

This file constructs the real `GPSSpoofMAVLinkBridge` (with a real UDP socket connected to a real listener socket standing in for QGC) and drives its real `_cb_gps_spoof_alert` callback directly — it does not require ROS 2 topics to actually be wired up, since that layer is stubbed, but it does exercise every byte the bridge would put on the wire.

**Validation:**
- [ ] `TestRealBridgeConstruction::test_bridge_connects_real_socket` PASSES
- [ ] `TestRealBridgeConstruction::test_bridge_uses_configured_system_and_component_id` PASSES
- [ ] `TestRealAlertToUdpPipeline::test_critical_alert_arrives_as_valid_statustext` PASSES
- [ ] `TestRealAlertToUdpPipeline::test_warning_alert_severity_and_text` PASSES
- [ ] `TestRealAlertToUdpPipeline::test_multiple_alerts_arrive_in_order` PASSES
- [ ] `TestRealAlertToUdpPipeline::test_malformed_json_sends_nothing_and_does_not_raise` PASSES
- [ ] `TestRealAlertToUdpPipeline::test_long_description_truncated_over_the_wire` PASSES
- [ ] `TestRealAlertToUdpPipeline::test_high_frequency_alert_stream_all_arrive_valid` PASSES
- [ ] All 8 integration tests PASS

## Phase 4: UDP Network Layer Tests

### 4.1 Verify UDP Socket Binding

```bash
# Terminal 1: Start bridge
ros2 run mavlink-bridge gps_spoof_mavlink_bridge \
  --ros-args -p mavlink_port:=14550

# Terminal 2: Check port is open
lsof -i :14550

# OR on Windows (PowerShell)
netstat -an | findstr 14550
```

**Validation:**
- [ ] Bridge is listening on UDP port 14550 (state: LISTEN or similar)
- [ ] Port shows component_id=200 (SAS custom component)

### 4.2 Capture UDP Packets

```bash
# Terminal 1: Start capture (Linux/WSL)
sudo tcpdump -i lo udp port 14550 -A -n

# OR use Wireshark (GUI)
# Filter: udp.dstport == 14550
# Capture on: Loopback

# Terminal 2: Launch pipeline
ros2 launch mavlink-bridge launch_gps_spoof_qgc.py system_id:=1

# Trigger alerts by running detector
ros2 run my_python_package gps_spoof_detector_node
```

**Validation (tcpdump output):**
- [ ] Packets appear on localhost:14550 (source port varies, dest 14550)
- [ ] Packet contents start with `0xfd` (MAVLink 2.0 STX byte)
- [ ] Packets have the real MAVLink 2.0 structure: `[STX][LEN][INCOMPAT_FLAGS][COMPAT_FLAGS][SEQ][SYSID][COMPID][MSGID×3 bytes][PAYLOAD][CRC×2 bytes]` — **10-byte header**, not the old 7-byte one
- [ ] Message ID (bytes 7-9, 24-bit little-endian) decodes to 253 (STATUSTEXT) — i.e. bytes 7,8,9 = `fd 00 00`
- [ ] SYSID byte (byte 5) is `0x01` (system_id=1)
- [ ] COMPID byte (byte 6) is `0xc8` (component_id=200 in hex)
- [ ] SEQ byte (byte 4) increments: 0x00, 0x01, 0x02, etc.

**Validation (Wireshark output — requires a MAVLink-aware Wireshark dissector):**
- [ ] Frame tree shows MAVLink 2.0 protocol recognized (it would **not** have been recognized at all before the Phase 0 fix — a non-recognized/malformed frame here means the fix regressed)
- [ ] Message Type: STATUSTEXT (id 253)
- [ ] System ID: 1
- [ ] Component ID: 200
- [ ] Text field contains alert message (first 50 chars)
- [ ] Severity field: 0 (INFO), 4 (WARNING), or 5 (CRITICAL)

### 4.3 Verify CRC and Field Layout in Captured Packets

```bash
# Use script to parse captured packets -- byte offsets match the REAL
# 10-byte MAVLink 2.0 header (see mavlink-bridge/mavlink_v2.py), not the
# old 7-byte layout this script used before the Phase 0 fix.
python3 << 'EOF'
import socket
import struct

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('localhost', 14550))

print("Listening for MAVLink packets...")
for i in range(5):  # Capture 5 packets
    data, addr = sock.recvfrom(1024)
    print(f"\nPacket {i+1}:")
    print(f"  Length: {len(data)} bytes")
    print(f"  Hex: {data[:16].hex()}...")

    # Parse frame (real MAVLink 2.0 header, 10 bytes before the payload)
    stx = data[0]
    payload_len = data[1]
    incompat_flags = data[2]
    compat_flags = data[3]
    seq = data[4]
    sysid = data[5]
    compid = data[6]
    msg_id = data[7] | (data[8] << 8) | (data[9] << 16)
    crc = struct.unpack('<H', data[-2:])[0]

    print(f"  STX: 0x{stx:02x} (expect 0xfd)")
    print(f"  Payload len: {payload_len}")
    print(f"  Sequence: {seq}")
    print(f"  SYSID: {sysid} (expect 1)")
    print(f"  COMPID: {compid} (expect 200)")
    print(f"  Message ID: {msg_id} (expect 253)")
    print(f"  CRC: 0x{crc:04x}")

sock.close()
EOF
```

**Optional cross-check against a reference decoder** (stronger than hand-parsing, catches anything the manual script above still gets wrong):
```bash
python3 << 'EOF'
from pymavlink.dialects.v20 import common as mavlink2
import socket

mav = mavlink2.MAVLink(None)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('localhost', 14550))

data, _ = sock.recvfrom(1024)
msg = mav.decode(bytearray(data))
print(msg)  # a real STATUSTEXT object if the frame is genuinely valid MAVLink 2.0
sock.close()
EOF
```

`mavlink-bridge/demo_qgc_wire_protocol.py` packages this same real-bridge-sends → pymavlink-decodes check into a standalone, no-ROS-2-required script — run it directly (`python demo_qgc_wire_protocol.py`) for a quick end-to-end sanity check before or instead of this manual snippet.

**Validation:**
- [ ] All packets start with STX=0xfd
- [ ] Payload length is consistent (STATUSTEXT payload is up to 54 bytes before MAVLink 2's trailing-zero truncation, often shorter in practice)
- [ ] Sequence (byte 4) increments or wraps correctly
- [ ] SYSID (byte 5) and COMPID (byte 6) match configured values
- [ ] Message ID (bytes 7-9) is 253 (STATUSTEXT)
- [ ] CRC values are non-zero and vary per packet
- [ ] If using the `pymavlink` cross-check: `mav.decode()` successfully returns a `STATUSTEXT` message object without raising

## Phase 5: QGroundControl Integration

**This phase is the highest-priority validation remaining in the whole project.** Everything above it (unit tests, byte-for-byte pymavlink comparison, real-socket simulation) has been verified; this is the first time the bridge's output will be checked against the actual external system it exists to talk to.

### 5.1 QGC Connection Setup

```bash
# Terminal 1: Launch pipeline
ros2 launch mavlink-bridge launch_gps_spoof_qgc.py system_id:=1

# Terminal 2: Start QGroundControl
# (or if using snap: snap run qgroundcontrol)
qgroundcontrol

# In QGC UI:
# 1. Click the "Q" icon (hamburger menu) at top-left
# 2. Select "Settings" (gear icon, top-right)
# 3. Go to "Comm Links" tab
# 4. Click "Add" button
# 5. Configure:
#    - Type: UDP
#    - Host: localhost (or 127.0.0.1)
#    - Port: 14550
#    - Baudrate: (N/A for UDP)
# 6. Click "Save"
# 7. Select the new link and click "Connect"
```

**Validation:**
- [ ] QGC shows "Listening" status on the UDP connection
- [ ] QGC does NOT show "Waiting for connection" indefinitely
- [ ] QGC status bar updates (usually shows signal bars)

### 5.2 Verify STATUSTEXT Reception in QGC

With QGC connected to the bridge:

```bash
# Preferred: ros2 run mavlink-bridge test_gps_spoof_alert_generator
# The inline publisher below is an equivalent fallback that needs no build:
python3 << 'EOF'
import json, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init()
node = Node('test_alert_publisher')
pub = node.create_publisher(String, '/gps_spoof_alert', 10)

for i, (level, state) in enumerate([
    ('INFO', 'NOMINAL'), ('WARNING', 'SUSPICIOUS'), ('CRITICAL', 'SPOOFING_DETECTED')
]):
    alert = {
        'alert_id': i, 'level': level, 'strategy': 'HEADING', 'state': state,
        'detail': {'description': f'{level} test alert'}, 'timestamp_us': 0,
    }
    msg = String(); msg.data = json.dumps(alert)
    pub.publish(msg)
    print(f"Published {level} alert")
    time.sleep(1.0)

rclpy.spin_once(node, timeout_sec=1.0)
EOF
```

**Validation (visual inspection in QGC):**
- [ ] Status text appears in the vehicle status area (top of Fly View)
- [ ] Alert text is readable (first 50 chars)
- [ ] Alert severity is color-coded:
  - [ ] INFO alerts appear in **green** or neutral color
  - [ ] WARNING alerts appear in **yellow**
  - [ ] CRITICAL alerts appear in **red** with alert banner
- [ ] Multiple alerts appear in sequence as they're published
- [ ] Alert count increments in vehicle health panel
- [ ] Alerts remain visible for several seconds (not flickering)

### 5.3 Test CRITICAL Alert Behavior

```bash
python3 << 'EOF'
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init()
node = Node('test_critical')
pub = node.create_publisher(String, '/gps_spoof_alert', 10)

alert = {
    'alert_id': 1,
    'level': 'CRITICAL',
    'strategy': 'HEADING',
    'state': 'SPOOFING_DETECTED',
    'detail': {
        'description': 'GPS SPOOFING DETECTED - multiple strategies active'
    },
    'timestamp_us': 0
}

msg = String()
msg.data = json.dumps(alert)
pub.publish(msg)
print("CRITICAL alert published")

rclpy.spin_once(node, timeout_sec=1.0)
EOF
```

**Validation:**
- [ ] QGC displays red alert banner with STATUSTEXT
- [ ] Sound/notification alert triggers (if enabled in QGC)
- [ ] "Abort Mission" or "Return to Launch" options appear
- [ ] Operator can confirm abort without unintended engagement

### 5.4 Test Multiple Alerts (Stress)

```bash
ros2 run mavlink-bridge test_gps_spoof_alert_generator --count 10 --rate 5
```

**Validation:**
- [ ] QGC receives all alerts (no dropped messages)
- [ ] Alert count in health panel reaches at least 5
- [ ] Bridge does not crash or disconnect
- [ ] QGC does not lag or become unresponsive
- [ ] Sequence numbers in captured UDP packets are continuous

## Phase 6: Error & Recovery Tests

### 6.1 Detector Crash Recovery

```bash
# Terminal 1: Launch bridge
ros2 run mavlink-bridge gps_spoof_mavlink_bridge \
  --ros-args -p system_id:=1

# Terminal 2: Start detector
ros2 run my_python_package gps_spoof_detector_node

# Verify alerts flowing (Terminal 3)
ros2 topic echo /gps_spoof_alert --once

# Kill detector (Ctrl+C in Terminal 2)
# Wait 5 seconds
# Restart detector

# Verify bridge is still running and reconnects
```

**Validation:**
- [ ] Bridge remains active after detector crashes
- [ ] Bridge resumes processing alerts when detector restarts
- [ ] No dropped UDP packets during detector downtime
- [ ] Sequence numbers resume correctly

### 6.2 Network Interrupt

```bash
# Terminal 1: tcpdump to log packets
tcpdump -i lo udp port 14550 -w /tmp/packets.pcap

# Terminal 2: Launch pipeline
ros2 launch mavlink-bridge launch_gps_spoof_qgc.py

# Terminal 3: Monitor topics
ros2 topic echo /gps_spoof_alert

# Simulate packet loss by blocking port (optional)
# iptables -A INPUT -p udp --dport 14550 -j DROP  (Linux sudo)

# Publish alerts during disruption
# Check that bridge doesn't crash
```

**Validation:**
- [ ] Bridge continues running even if UDP delivery fails
- [ ] No exceptions in bridge logs
- [ ] When network is restored, packets resume flowing
- [ ] QGC shows alerts once connection is restored

### 6.3 Malformed JSON Handling

```bash
python3 << 'EOF'
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init()
node = Node('test_malformed')
pub = node.create_publisher(String, '/gps_spoof_alert', 10)

# Publish invalid JSON
msg = String()
msg.data = '{"invalid": json without quotes}'
pub.publish(msg)
print("Malformed JSON published")

# Wait and observe bridge doesn't crash
import time
time.sleep(1)

rclpy.spin_once(node, timeout_sec=1.0)
EOF
```

**Validation:**
- [ ] Bridge logs error but continues running
- [ ] Bridge doesn't crash or disconnect
- [ ] Next valid alert is processed correctly

## Phase 7: Performance & Scalability

### 7.1 High-Frequency Alert Stream

```bash
# Preferred: ros2 run mavlink-bridge test_gps_spoof_alert_generator --count 100 --rate 100
# Inline equivalent:
python3 << 'EOF'
import json
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init()
node = Node('test_highfreq')
pub = node.create_publisher(String, '/gps_spoof_alert', 10)

print("Publishing 100 alerts at 100 Hz...")
start = time.time()

for i in range(100):
    alert = {
        'alert_id': i,
        'level': ['INFO', 'WARNING', 'CRITICAL'][i % 3],
        'strategy': 'HEADING',
        'state': 'NOMINAL',
        'detail': {'index': i},
        'timestamp_us': 0
    }
    msg = String()
    msg.data = json.dumps(alert)
    pub.publish(msg)
    time.sleep(0.01)  # 100 Hz

elapsed = time.time() - start
print(f"Completed in {elapsed:.2f}s")
EOF
```

**Validation:**
- [ ] Bridge processes all 100 alerts without dropping any
- [ ] UDP packets are delivered for all alerts
- [ ] No exceptions in bridge logs
- [ ] QGC displays final alert count as 100 (or continues counting)
- [ ] System remains responsive

### 7.2 Verify Sequence Number Continuity

```bash
python3 << 'EOF'
import socket
import struct

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('localhost', 14550))
sock.settimeout(5.0)

sequences = []
try:
    while len(sequences) < 20:
        data, _ = sock.recvfrom(1024)
        seq = data[4]  # Sequence is byte 4 in the real 10-byte MAVLink 2.0 header
        sequences.append(seq)
except socket.timeout:
    pass

sock.close()

print(f"Captured {len(sequences)} packets")
print(f"Sequences: {sequences}")

# Check for gaps
for i in range(1, len(sequences)):
    expected = (sequences[i-1] + 1) % 256
    if sequences[i] != expected:
        print(f"  GAP at index {i}: expected {expected}, got {sequences[i]}")
    else:
        print(f"  OK: {sequences[i-1]} -> {sequences[i]}")
EOF
```

**Validation:**
- [ ] Sequence numbers increment correctly
- [ ] No gaps in sequence (except wrap-around at 255->0)
- [ ] All packets are delivered (no drops)

## Phase 8: Deploy Readiness

### 8.1 Documentation Check

- [ ] README.md has quickstart section
- [ ] MAVLINK_BRIDGES_QGC_INTEGRATION.md covers all integration points
- [ ] Launch file is well-documented with examples
- [ ] Parameters are documented with defaults
- [ ] Error messages in bridge are clear and actionable

### 8.2 Configuration Validation

```bash
# Test with various parameter combinations
ros2 launch mavlink-bridge launch_gps_spoof_qgc.py \
  system_id:=2 \
  mavlink_port:=14551

# Verify bridge accepts parameters
ros2 node info /gps_spoof_mavlink_bridge
```

**Validation:**
- [ ] Bridge accepts system_id parameter (1-255)
- [ ] Bridge accepts component_id parameter
- [ ] Bridge accepts mavlink_port parameter (any available port)
- [ ] Bridge accepts mavlink_host parameter

### 8.3 Build & Package Validation

```bash
cd SAS
colcon clean packages --packages-select mavlink-bridge
colcon build --packages-select mavlink-bridge --symlink-install
colcon test --packages-select mavlink-bridge
```

**Validation:**
- [ ] Package builds without errors
- [ ] Package builds without warnings (treat as errors)
- [ ] All tests pass
- [ ] Package can be sourced and run from any directory
- [ ] `setup.py`'s console_scripts all point at files that actually exist

### 8.4 Integration with CI/CD

- [ ] Unit tests run in CI pipeline
- [ ] Integration test checklist is automated (where possible)
- [ ] Build artifacts are uploaded to deployment storage
- [ ] Documentation is generated from code comments

## Sign-Off

**Test Lead:** _________________
**Date:** _________________

**System Readiness Assessment:**

- [ ] All Phase tests completed
- [ ] No critical issues remaining
- [ ] All blockers resolved
- [ ] Documentation is complete and accurate
- [ ] Team is trained on deployment and troubleshooting

**Deployment Decision:**

- [ ] **APPROVED FOR DEPLOYMENT** — All tests passing, no open issues
- [ ] **APPROVED WITH CAVEATS** — Known limitations documented
- [ ] **NOT APPROVED** — Issues remain, re-test after fixes

**Notes:**

_____________________________________________________________________

_____________________________________________________________________

## References

- [MAVLINK_BRIDGES_QGC_INTEGRATION.md](MAVLINK_BRIDGES_QGC_INTEGRATION.md) — Architecture and usage guide (GPS spoofing, mission control, telemetry)
- [IMPLEMENTATION_STATUS.md](IMPLEMENTATION_STATUS.md) — Current implementation status, including the Phase 0 MAVLink protocol audit that motivated this checklist's revision
- [IMPLEMENTATION_ROADMAP.md](IMPLEMENTATION_ROADMAP.md) — Full project roadmap
- `mavlink-bridge/mavlink_v2.py` — Verified MAVLink 2.0 codec shared by all bridges
- `mavlink-bridge/gps_spoof_mavlink_bridge.py` — Bridge implementation
- `mavlink-bridge/tests/unit/test_mavlink_crc.py`, `mavlink-bridge/tests/integration/test_gps_spoof_integration.py` — Current test suite for this bridge
- `SAS/my_python_package/gps_spoof_detector_node.py` — Detector implementation
