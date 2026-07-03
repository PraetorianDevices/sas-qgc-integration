# GPS Spoofing Detector → QGroundControl Integration

## Overview

The GPS spoofing detector node (`gps_spoof_detector_node.py`) detects GPS spoofing attacks using three independent strategies:
1. **Heading cross-check** — EKF2 heading vs. raw magnetometer
2. **Altitude cross-check** — GPS altitude delta vs. barometer
3. **PX4 internal flags** — u-blox M8P/F9P hardware anti-spoofing

This integration bridges spoofing alerts from the SAS ROS 2 system to QGroundControl via MAVLink STATUSTEXT messages, providing real-time operator awareness of GPS threats.

---

## Architecture

```
gps_spoof_detector_node (ROS 2)
  │
  └─→ /gps_spoof_alert (String, JSON)
        │
        └─→ gps_spoof_mavlink_bridge (ROS 2)
              │
              └─→ MAVLink STATUSTEXT (UDP 14550)
                    │
                    └─→ QGroundControl
                          │
                          ├─ Status bar alert (color-coded)
                          ├─ Vehicle health panel
                          └─ Mission abort option (CRITICAL level)
```

---

## Component: gps_spoof_mavlink_bridge

**File:** `mavlink-bridge/gps_spoof_mavlink_bridge.py`

**Purpose:** Convert ROS 2 GPS spoofing alerts to MAVLink STATUSTEXT messages for QGC.

### Inputs

- **Topic:** `/gps_spoof_alert` (std_msgs/String)
- **Format:** JSON with fields:
  ```json
  {
    "alert_id": <int>,
    "level": "INFO" | "WARNING" | "CRITICAL",
    "strategy": "HEADING" | "ALTITUDE" | "PX4_INTERNAL",
    "state": "NOMINAL" | "SUSPICIOUS" | "SPOOFING_DETECTED",
    "detail": { <strategy-specific fields> },
    "timestamp_us": <int>
  }
  ```

### Outputs

- **Protocol:** MAVLink 2.0 STATUSTEXT (msg ID 253)
- **Transport:** UDP (default: `localhost:14550`)
- **Fields:**
  - `severity` (uint8): MAV_SEVERITY_INFO (0), WARNING (4), or CRITICAL (5)
  - `text` (char[50]): Alert message (truncated to 50 chars)
  - `id` (uint16): Unique message ID
  - `chunk_seq` (uint8): For multi-part messages (not used here)

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `system_id` | int | 1 | MAVLink system ID for this vehicle |
| `component_id` | int | 200 | MAVLink component ID (custom for SAS) |
| `mavlink_host` | str | localhost | UDP hostname/IP for QGC connection |
| `mavlink_port` | int | 14550 | UDP port for QGC telemetry |

---

## Alert Level Mapping

| SAS Alert Level | MAVLink Severity | QGC Display | Operator Action |
|---|---|---|---|
| **INFO** | 0 (INFO) | Green text | Monitor |
| **WARNING** | 4 (WARNING) | Yellow text | Prepare to abort |
| **CRITICAL** | 5 (CRITICAL) | Red text, alert sound | Abort mission |

---

## Usage

### Prerequisites

1. **ROS 2 Jazzy** installed
2. **SAS repository** with `gps_spoof_detector_node` built
3. **mavlink-bridge** package built
4. **QGroundControl** running on target host

### Step 1: Build the Integration

```bash
cd d:/praetoriandevices

# Build SAS (if not already done)
cd SAS && colcon build --packages-select my_python_package
source install/setup.sh

# Build mavlink-bridge
colcon build --packages-select mavlink-bridge
source install/setup.sh
```

### Step 2: Launch Detector + Bridge

**Option A: Using launch file (recommended)**

```bash
ros2 launch mavlink-bridge launch_gps_spoof_qgc.py system_id:=1
```

**Option B: Manual startup (two terminals)**

Terminal 1:
```bash
ros2 run my_python_package gps_spoof_detector_node --ros-args -p system_id:=1
```

Terminal 2:
```bash
ros2 run mavlink-bridge gps_spoof_mavlink_bridge \
  --ros-args \
  -p system_id:=1 \
  -p component_id:=200 \
  -p mavlink_host:=localhost \
  -p mavlink_port:=14550
```

### Step 3: Connect QGroundControl

1. **Open QGC**
2. Go to **Application Settings** (gear icon, top-right)
3. Select **Comm Links** tab
4. Click **Add** to create a new link
5. Configure:
   - **Type:** UDP
   - **Host:** `localhost` (or IP where ROS 2 is running)
   - **Port:** `14550`
6. Click **Connect**

Expected result: QGC connects and begins receiving spoofing alerts as STATUSTEXT messages.

---

## Testing

### Test 1: Synthetic Alerts (No Detector Needed)

Run the test alert generator to verify the bridge is working:

```bash
ros2 run mavlink-bridge test_gps_spoof_alert_generator
```

This publishes synthetic spoofing alerts in sequence:
- INFO (nominal)
- WARNING (heading divergence)
- CRITICAL (altitude spoofing)
- etc.

**In QGC:** You should see color-coded STATUSTEXT messages appear in the vehicle status bar.

### Test 2: Real Detector Alerts

Run the full detector stack with the bridge:

```bash
ros2 launch mavlink-bridge launch_gps_spoof_qgc.py system_id:=1
```

Monitor the ROS 2 log:
```bash
ros2 node list
```

You should see:
- `/gps_spoof_detector_node` (publishing `/gps_spoof_alert`)
- `/gps_spoof_mavlink_bridge` (subscribing to alerts, publishing MAVLink)

### Test 3: Verify UDP Packets (Advanced)

Capture MAVLink packets on port 14550:

```bash
# Linux/WSL
tcpdump -i lo udp port 14550 -A

# Wireshark (GUI)
# Filter: "udp.dstport == 14550"
```

Expected packet format:
```
[0xFD] [LEN] [INV] [MSG_ID=253] [SYS_ID] [COMP_ID] [SEQ] [PAYLOAD] [CRC]
```

---

## ROS 2 Topics

### Subscribed

| Topic | Type | Source | Purpose |
|-------|------|--------|---------|
| `/gps_spoof_alert` | std_msgs/String | gps_spoof_detector_node | GPS spoofing detection events |

### Published

- **None** (UDP output via socket; not a ROS 2 topic)

---

## QGroundControl Display

### STATUSTEXT Message Display

When a spoofing alert is published:

1. **Status Bar:** Red/yellow/green alert banner with text (first 50 chars)
2. **Vehicle Health Panel:** Alert count increments
3. **Message Log:** Full alert JSON logged in flight logs

### Example Alert Text

```
[GPS DETECTED] HEADING: EKF2 heading diverging...
[GPS SUSPICIOUS] ALTITUDE: GPS altitude delta diverging...
[GPS OK] HEADING: Nominal operation
```

### Response Options

**When CRITICAL alert is active:**
- QGC displays **"Abort Mission"** button in emergency panel
- Operator must confirm before continuing
- MAVLink COMMAND_LONG with MAV_CMD_DO_RETURN_TO_LAUNCH can be triggered

---

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| No STATUSTEXT in QGC | Bridge not running | Check `ros2 node list` — should show `/gps_spoof_mavlink_bridge` |
| Bridge connects but no messages | Detector not publishing | Check `/gps_spoof_alert` topic: `ros2 topic echo /gps_spoof_alert` |
| UDP socket error | Port 14550 in use | Change port via `mavlink_port` parameter |
| MAVLink packets not recognized | CRC error | Verify CRC computation — use Wireshark to inspect packets |
| QGC shows different vehicle | system_id mismatch | Ensure QGC and bridge use same `system_id` parameter |

---

## Architecture Notes

### Why MAVLink STATUSTEXT?

STATUSTEXT (msg ID 253) is the standard MAVLink message for text alerts with severity levels. It:
- Integrates cleanly with QGC's alert system
- Supports color-coding (green/yellow/red)
- Can trigger operator actions (abort, RTL, etc.)
- Is part of the minimal MAVLink implementation

### Sequence Numbering

The bridge maintains a sequence counter (0-255) for MAVLink packet ordering. Each STATUSTEXT increments the sequence, allowing QGC to detect dropped packets.

### CRC Computation

MAVLink uses CRC16-CCITT with a message-specific CRC_EXTRA byte. For STATUSTEXT (msg 253), the CRC_EXTRA is **83**. The bridge computes this correctly to ensure QGC accepts packets.

---

## Integration with QGC Plugin (Future)

Once the custom QGC plugin is implemented (SASPlugin.cc), it can:

1. **Monitor `/gps_spoof_status`** directly (ROS 2 topic subscription via plugin interface)
2. **Display spoofing state indicator** in Fly View (red = critical, yellow = warning, green = nominal)
3. **Trigger mission abort** on operator command with 2-factor confirmation
4. **Log all alerts** with full JSON detail in QGC flight logs

For now, STATUSTEXT provides basic alert functionality until the plugin is ready.

---

## References

- **MAVLink Spec:** https://mavlink.io/en/messages/common.html#STATUSTEXT
- **QGC Plugin Architecture:** `QGroundControl/src/API/QGCCorePlugin.h`
- **GPS Spoofing Detector:** `SAS/my_python_package/gps_spoof_detector_node.py`
- **MAVLink Bridge:** `mavlink-bridge/gps_spoof_mavlink_bridge.py`
