# GPS Spoofing Detector → QGC Integration — Test Checklist

## Phase 1: Environment Setup

- [ ] ROS 2 Jazzy installed and sourced
- [ ] SAS package built: `colcon build --packages-select my_python_package`
- [ ] mavlink-bridge package built: `colcon build --packages-select mavlink-bridge`
- [ ] QGroundControl installed (for end-to-end test)
- [ ] Port 14550 is available (check with `lsof -i :14550` or `netstat -an | grep 14550`)

## Phase 2: Unit Tests (Already Passing ✓)

- [x] MAVLink frame structure validation
- [x] CRC16-CCITT computation
- [x] STATUSTEXT payload formatting
- [x] Message truncation (100 chars → 50 chars)
- [x] Sequence numbering and wrap-around
- [x] Alert level mapping (INFO, WARNING, CRITICAL)
- [x] Multiple frame generation

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

### 3.3 Run Unit Test Suite

```bash
cd mavlink-bridge
python -m pytest test_integration.py -v
```

**Validation:**
- [ ] Test discovers ROS 2 environment
- [ ] `test_detector_publishes_alert` PASSES
- [ ] `test_bridge_receives_alert` PASSES
- [ ] `test_mavlink_frame_parsing` PASSES
- [ ] `test_alert_level_to_severity_mapping` PASSES
- [ ] `test_json_alert_format_compatibility` PASSES
- [ ] `test_sequence_numbering_across_alerts` PASSES
- [ ] `test_message_truncation_in_frame` PASSES
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
- [ ] Packets have consistent structure: `[0xfd] [LEN] [INV] [MSGID] [SYSID] [COMPID] [SEQ] [PAYLOAD] [CRC]`
- [ ] MSGID byte is `0xfd` (253 for STATUSTEXT)
- [ ] SYSID byte is `0x01` (system_id=1)
- [ ] COMPID byte is `0xc8` (component_id=200 in hex)
- [ ] SEQ byte increments: 0x00, 0x01, 0x02, etc.

**Validation (Wireshark output):**
- [ ] Frame tree shows MAVLink protocol recognized
- [ ] Message Type: STATUSTEXT (id 253)
- [ ] System ID: 1
- [ ] Component ID: 200
- [ ] Text field contains alert message (first 50 chars)
- [ ] Severity field: 0 (INFO), 4 (WARNING), or 5 (CRITICAL)

### 4.3 Verify CRC in Captured Packets

```bash
# Use script to parse captured packets
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
    
    # Parse frame
    stx = data[0]
    payload_len = data[1]
    msg_id = data[3]
    seq = data[6]
    crc = struct.unpack('<H', data[-2:])[0]
    
    print(f"  STX: 0x{stx:02x} (expect 0xfd)")
    print(f"  Payload len: {payload_len}")
    print(f"  Message ID: {msg_id} (expect 253)")
    print(f"  Sequence: {seq}")
    print(f"  CRC: 0x{crc:04x}")

sock.close()
EOF
```

**Validation:**
- [ ] All packets start with STX=0xfd
- [ ] Payload length is consistent (~54 bytes)
- [ ] Message ID is 253 (STATUSTEXT)
- [ ] Sequence numbers increment or wrap correctly
- [ ] CRC values are non-zero and vary per packet

## Phase 5: QGroundControl Integration

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
# Terminal 2: Publish test alerts
ros2 run mavlink-bridge test_gps_spoof_alert_generator
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
# Publish a CRITICAL alert
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
# Publish 10 rapid alerts
ros2 run mavlink-bridge test_gps_spoof_alert_generator
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
        seq = data[6]  # Sequence at byte 6
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
- [ ] GPS_SPOOFING_QGC_INTEGRATION.md covers all integration points
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

- [GPS_SPOOFING_QGC_INTEGRATION.md](GPS_SPOOFING_QGC_INTEGRATION.md) — Architecture and usage guide
- [SAS_QGC_Integration_Plan.md](SAS_QGC_Integration_Plan.md) — High-level integration plan
- `mavlink-bridge/gps_spoof_mavlink_bridge.py` — Bridge implementation
- `SAS/my_python_package/gps_spoof_detector_node.py` — Detector implementation
