# MAVLink Bridges → QGroundControl Integration

Reference for the three MAVLink bridges that connect SAS to QGroundControl: GPS spoofing
alerts, mission upload/download, and vehicle telemetry. All three, plus the remaining four
bridges (`fleet_manager`, `collision`, `emergency_wipe`, and the `mavlink_router` that fans
QGC's single UDP link out to the inbound ones), have been run live against real PX4 SITL and
real QGroundControl — see [IMPLEMENTATION_STATUS.md](IMPLEMENTATION_STATUS.md) for what was
verified and the bugs that live testing found.

All wire formats below come from `mavlink-bridge/mavlink_v2.py`, the shared codec verified
byte-for-byte against `pymavlink`. Where this document previously described a different (now
superseded) format, that has been corrected here.

- [Running the Integration](#running-the-integration) — shared setup for all three bridges
- [GPS Spoofing Alert Bridge](#gps-spoofing-alert-bridge)
- [Mission Control Bridge](#mission-control-bridge)
- [Telemetry Bridge](#telemetry-bridge)
- [References](#references)

---

## Running the Integration

One launch file starts all seven MAVLink bridges together (see
[IMPLEMENTATION_STATUS.md](IMPLEMENTATION_STATUS.md) for the full stack, including PX4 SITL
and the SAS nodes). Two settings below are easy to get wrong and fail **silently** if missed.

```bash
# 1. PX4 SITL, then at the pxh> prompt stop PX4's own mavlink instance
#    (it contends with our router on port 14550):
#       mavlink stop -u 18570

# 2. Micro XRCE-DDS agent
MicroXRCEAgent udp4 -p 8888

# 3. SAS control nodes (runs them under the /drone_1 namespace)
ros2 launch my_python_package single_drone.launch.py

# 4. All seven MAVLink bridges
ros2 launch mavlink-bridge launch_sas_qgc_integration.py \
  system_id:=1 \
  sas_namespace:=drone_1 \
  mavlink_host:=<address QGC is reachable at>
```

- **`sas_namespace:=drone_1` is required** alongside `single_drone.launch.py`. Without it,
  `mission_control_bridge`/`emergency_wipe_mavlink_bridge` publish/serve on unprefixed
  topics while the SAS nodes listen under `/drone_1/...` — mission upload and emergency wipe
  succeed at the MAVLink layer and are then silently dropped, with no error. `drone_id`
  (separate from `sas_namespace`) prefixes **PX4** DDS topics instead, and should stay empty
  for a single SITL instance, which publishes `/fmu/...` unprefixed.
- **QGC must target the address this stack is actually reachable at**, not `localhost`.
  Running under WSL2, that means the WSL2 interface IP (`ip addr show eth0`) —
  **WSL2's localhost-forwarding silently drops UDP**, so `localhost` looks like it should
  work and doesn't. The IP changes when WSL restarts.

In QGC: **Application Settings → Comm Links → Add → UDP**, host = the address above, port
`14550` → **Connect**. `mavlink_port`/`mavlink_host`/`mavlink_bind_host` are shared launch
arguments across all bridges; see each bridge's parameter table below for the rest.

**Multi-drone:** run one instance of this launch file per drone, with distinct `system_id`,
`drone_id`, `sas_namespace`, and `mavlink_port`:

```bash
# Drone 1
ros2 launch mavlink-bridge launch_sas_qgc_integration.py \
  system_id:=1 drone_id:=drone_1 sas_namespace:=drone_1

# Drone 2 (different UDP port)
ros2 launch mavlink-bridge launch_sas_qgc_integration.py \
  system_id:=2 drone_id:=drone_2 sas_namespace:=drone_2 mavlink_port:=14551
```

---

## GPS Spoofing Alert Bridge

**File:** `mavlink-bridge/gps_spoof_mavlink_bridge.py`
**Verified live:** spoofing alerts arriving in QGC as color-coded `STATUSTEXT`.

### Overview

`gps_spoof_detector_node` detects GPS spoofing using three independent strategies:
1. **Heading cross-check** — EKF2 heading vs. raw magnetometer
2. **Altitude cross-check** — GPS altitude delta vs. barometer
3. **PX4 internal flags** — u-blox M8P/F9P hardware anti-spoofing

This bridge converts its alerts to MAVLink `STATUSTEXT`, giving the operator real-time
awareness in QGC.

### Architecture

```
gps_spoof_detector_node (ROS 2)
  └─→ /gps_spoof_alert (String, JSON)
        └─→ gps_spoof_mavlink_bridge
              └─→ MAVLink STATUSTEXT (UDP 14550)
                    └─→ QGroundControl (status bar alert, vehicle health panel)
```

### Input

- **Topic:** `/gps_spoof_alert` (`std_msgs/String`)
- **Format:**
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

### Output

- **Message:** `STATUSTEXT` (id 253), `CRC_EXTRA` 83
- **Fields:** `severity` (u8), `text` (char[50], truncated), `id` (u16), `chunk_seq` (u8, unused)

### Alert Level Mapping

The MAV_SEVERITY values below are the real spec values (lower = more severe) — a prior
revision of this bridge had them transposed (INFO=0, CRITICAL=5), which would have sent a
genuine CRITICAL spoofing alert at NOTICE priority. That is fixed; these are the current,
correct values.

| SAS Alert Level | MAV_SEVERITY | Value | QGC Display |
|---|---|---|---|
| **INFO** | `MAV_SEVERITY_INFO` | 6 | Green text |
| **WARNING** | `MAV_SEVERITY_WARNING` | 4 | Yellow text |
| **CRITICAL** | `MAV_SEVERITY_CRITICAL` | 2 | Red text, alert sound |

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `system_id` | int | 1 | MAVLink system ID |
| `component_id` | int | 200 | MAVLink component ID (custom for SAS) |
| `mavlink_host` | str | localhost | Where QGC is reachable |
| `mavlink_port` | int | 14550 | UDP port |

### Testing

**Synthetic alerts** (no detector needed) — publishes a sequence of INFO/WARNING/CRITICAL
alerts to sanity-check the bridge in isolation:

```bash
ros2 run mavlink-bridge test_gps_spoof_alert_generator [--count N] [--rate HZ] [--level LEVEL]
```

**Real detector + bridge:**

```bash
ros2 launch mavlink-bridge launch_gps_spoof_qgc.py system_id:=1
ros2 node list   # expect /gps_spoof_detector_node and /gps_spoof_mavlink_bridge
```

**Verify UDP packets on the wire:**

```bash
# WSL/Linux
tcpdump -i eth0 udp port 14550 -A     # note: interface is eth0 under WSL2, not lo
```

A frame starts `0xFD [LEN] [INCOMPAT] [COMPAT] [SEQ] [SYS_ID] [COMP_ID] [MSG_ID×3]`
for MAVLink 2.0 — QGC's own outbound traffic may instead be MAVLink 1.0 (`0xFE`, 6-byte
header, 8-bit message id); both are accepted on receive by `mavlink_v2.parse_frames()`.

### Troubleshooting

| Issue | Cause | Solution |
|---|---|---|
| No `STATUSTEXT` in QGC | Bridge not running | `ros2 node list` should show `/gps_spoof_mavlink_bridge` |
| Bridge connects, no messages | Detector not publishing | `ros2 topic echo /gps_spoof_alert` |
| UDP socket error | Port 14550 in use | Change `mavlink_port` |
| QGC shows a different vehicle | `system_id` mismatch | Match QGC's and the bridge's `system_id` |

### Why STATUSTEXT

`STATUSTEXT` is the standard MAVLink text-alert message with severity levels: it integrates
with QGC's color-coded alert system, is part of the minimal MAVLink implementation, and can
be extended later to trigger operator actions (abort, RTL) via `COMMAND_LONG`.

---

## Mission Control Bridge

**File:** `mavlink-bridge/mission_control_bridge.py`
**Verified live:** a real QGC upload of 5 waypoints completing the full handshake and
reaching `mission_executor_node` ("Parsed QGC mission with 5 waypoints").

### Overview

Bidirectional mission control between QGroundControl and `mission_executor_node`: mission
upload (QGC → SAS), execution progress reporting (SAS → QGC), and mission download
(SAS → QGC).

### Architecture

```
QGroundControl (Plan view)
  │  MISSION_COUNT → MISSION_REQUEST_INT × N → MISSION_ITEM_INT × N → (ACK)
  ↓ MAVLink UDP, via mavlink_router_node
mission_control_bridge
  │  assembles waypoints into SAS's mission JSON format
  ↓ /mission_executor/load_mission (ROS 2)
mission_executor_node
  │  executes the sequence, reports progress
  ↑ /mission_executor/status (ROS 2)
mission_control_bridge
  │  MISSION_CURRENT (1 Hz)
  ↑ MAVLink UDP
QGroundControl (Fly view: "Executing waypoint N of M")
```

### MAVLink Messages Handled

Upload uses **`MISSION_ITEM_INT`** (id 73, integer lat/lon ×1e7) — not the older float-native
`MISSION_ITEM` (id 39) that early drafts of this bridge, and this document, used to describe.

| Direction | Message | ID | Purpose |
|---|---|---|---|
| QGC → SAS | `MISSION_COUNT` | 44 | Announces N items about to be uploaded |
| QGC → SAS | `MISSION_ITEM_INT` | 73 | One waypoint |
| QGC → SAS | `MISSION_REQUEST_LIST` | 43 | "How many waypoints do you have?" (download) |
| QGC → SAS | `MISSION_REQUEST` / `MISSION_REQUEST_INT` | 40 / 51 | Requests a specific waypoint |
| SAS → QGC | `MISSION_COUNT` | 44 | Reply to `MISSION_REQUEST_LIST`, or requesting upload item *i* |
| SAS → QGC | `MISSION_REQUEST_INT` | 51 | Pulls waypoint *i* during upload |
| SAS → QGC | `MISSION_ITEM_INT` | 73 | Reply during download |
| SAS → QGC | `MISSION_ACK` | 47 | Accepted/rejected, sent once after the last upload item |
| SAS → QGC | `MISSION_CURRENT` | 42 | Currently-executing waypoint index, 1 Hz |

### MISSION_ITEM_INT Wire Format

37 bytes (before MAVLink 2's trailing-zero truncation). Verified field **order** (not
declaration order) against `pymavlink`:

```
param1:f32  param2:f32  param3:f32  param4:f32
x:i32 (lat ×1e7)  y:i32 (lon ×1e7)  z:f32 (altitude)
seq:u16  command:u16
target_system:u8  target_component:u8  frame:u8  current:u8  autocontinue:u8
mission_type:u8   (extension field, defaults to 0/MISSION)
```

Example — a relative-altitude waypoint at San Francisco, 100 m up:

```json
{
  "sequence": 0, "frame": 3, "command": 16, "current": 0, "autocontinue": 1,
  "params": [0, 0, 0, 0],
  "position": {"latitude": 37.7749, "longitude": -122.4194, "altitude": 100.0}
}
```
(`frame` 3 = `MAV_FRAME_GLOBAL_RELATIVE_ALT`; `command` 16 = `MAV_CMD_NAV_WAYPOINT`.)

### Upload Handshake

```
QGC:    MISSION_COUNT(count=5)
Bridge: MISSION_REQUEST_INT(seq=0)
QGC:    MISSION_ITEM_INT(seq=0, ...)
Bridge: MISSION_REQUEST_INT(seq=1)
...
QGC:    MISSION_ITEM_INT(seq=4, ...)
Bridge: MISSION_ACK(ACCEPTED)          # once, after the last item
Bridge: publishes the assembled mission to mission_executor_node
```

### ROS 2 Integration

**Published:** `mission_executor/load_mission` (`std_msgs/String`, JSON: `{"waypoints": [...], "home": {...}}`) — `RELIABLE`/`TRANSIENT_LOCAL` QoS. This was previously `BEST_EFFORT`, which is
incompatible with `mission_executor_node`'s default `RELIABLE` subscription under DDS QoS
rules — a fully-assembled mission would be published and silently never delivered. Fixed.

**Subscribed:** `mission_executor/status` (`std_msgs/String`, JSON progress updates)

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `system_id` | int | 1 | MAVLink system ID |
| `component_id` | int | 1 | MAVLink component ID |
| `drone_id` | str | "" | **PX4** DDS topic namespace — leave empty for single SITL |
| `sas_namespace` | str | "" | **SAS** node namespace — pass `drone_1` to match `single_drone.launch.py` |
| `mavlink_host` | str | localhost | Where QGC is reachable |
| `mavlink_bind_host` | str | 0.0.0.0 | Local inbound bind address |
| `mavlink_port` | int | 14550 | External UDP port (behind `mavlink_router_node` in the full stack) |

### Coordinate Frames

| Frame ID | Name | Altitude Reference |
|---|---|---|
| 0 | `MAV_FRAME_GLOBAL` | MSL |
| 3 | `MAV_FRAME_GLOBAL_RELATIVE_ALT` | Relative to home (QGC's default) |
| 10 | `MAV_FRAME_LOCAL_NED` | Local NED origin |

### QGroundControl Display

- **Plan tab:** waypoint list, map, edit controls, Upload button
- **Fly tab:** mission progress bar ("Executing waypoint 3 of 5"), active/next waypoint on map

Clicking Upload may show *"This Plan was created for a different firmware or vehicle
type..."* — this is benign. It compares the `.plan` file's saved template against the
vehicle's reported type; our `HEARTBEAT` correctly reports `MAV_TYPE_QUADROTOR` /
`MAV_AUTOPILOT_PX4`. Click OK.

### Error Handling

| Error | MAVLink Response | QGC Display |
|---|---|---|
| Mission format invalid | `MISSION_ACK(ERROR)` | Red error message |
| Communication lost | No response (timeout) | "Lost vehicle connection" |

### Known Limitations

- No mid-flight mission editing (waypoints lock during execution)
- No geofence/rally-point support
- Uploaded missions are unsigned — the MAVLink mission protocol has no signature transport,
  so `load_mission_callback` accepts them without a signature by design (`strict=False`,
  in contrast to `.plan`-file missions loaded directly, which are verified if signed)

---

## Telemetry Bridge

**File:** `mavlink-bridge/telemetry_mavlink_bridge.py`
**Verified live:** real PX4 telemetry (HEARTBEAT, position, attitude, battery) received
by QGC during an armed flight (climb to 5 m, hold, commanded land, auto-disarm).

### Overview

Streams vehicle telemetry from PX4 (via `px4_msgs`) to QGC as standard MAVLink messages:
position/navigation, attitude, power, and system health.

### Architecture

```
PX4 (uXRCE-DDS)
  ├─→ /fmu/out/vehicle_local_position   ├─→ /fmu/out/battery_status
  ├─→ /fmu/out/vehicle_attitude          ├─→ /fmu/out/vehicle_angular_velocity
  ├─→ /fmu/out/vehicle_status            └─→ /fmu/out/cpuload
        │
        └─→ telemetry_mavlink_bridge
              └─→ MAVLink (UDP 14550): HEARTBEAT (1 Hz), GLOBAL_POSITION_INT (10 Hz),
                  ATTITUDE (10 Hz), SYS_STATUS (10 Hz), BATTERY_STATUS (10 Hz)
                        └─→ QGroundControl
```

`vehicle_angular_velocity` and `cpuload` were added after live testing against real
`px4_msgs` found that `VehicleAttitude.rollspeed/pitchspeed/yawspeed` and
`VehicleStatus.load`/`system_status` are not real fields — see
[IMPLEMENTATION_STATUS.md](IMPLEMENTATION_STATUS.md) Part 1 for the fix.

### Inputs → Outputs

| ROS 2 Topic | px4_msgs Type | Feeds |
|---|---|---|
| `/fmu/out/vehicle_local_position` | `VehicleLocalPosition` | `GLOBAL_POSITION_INT` |
| `/fmu/out/vehicle_attitude` | `VehicleAttitude` | `ATTITUDE` (quaternion) |
| `/fmu/out/vehicle_angular_velocity` | `VehicleAngularVelocity` | `ATTITUDE` (rates) |
| `/fmu/out/vehicle_status` | `VehicleStatus` | `HEARTBEAT` (armed, mode) |
| `/fmu/out/battery_status` | `BatteryStatus` | `SYS_STATUS`, `BATTERY_STATUS` |
| `/fmu/out/cpuload` | `Cpuload` | `SYS_STATUS` (CPU load) |

| MAVLink Message | ID | Rate | Content |
|---|---|---|---|
| `HEARTBEAT` | 0 | 1 Hz | Armed state, flight mode (arming-state-derived), autopilot=PX4, type=QUADROTOR |
| `GLOBAL_POSITION_INT` | 33 | 10 Hz | Lat/lon (×1e7), altitude (mm), velocity (cm/s), heading |
| `ATTITUDE` | 30 | 10 Hz | Roll/pitch/yaw (rad, from quaternion), angular rates |
| `SYS_STATUS` | 1 | 10 Hz | Battery voltage/current, CPU load, sensor health bitmask |
| `BATTERY_STATUS` | 147 | 10 Hz | Per-cell voltage, current, capacity, `energy_consumed=-1` (unknown sentinel — px4_msgs has no real equivalent field) |

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `system_id` | int | 1 | MAVLink system ID (1–255) |
| `component_id` | int | 1 | `MAV_COMP_ID_AUTOPILOT` |
| `drone_id` | str | "" | PX4 DDS topic namespace — empty for single SITL |
| `mavlink_host` | str | localhost | Where QGC is reachable |
| `mavlink_port` | int | 14550 | UDP port |

### Coordinate Frames & Units

Both PX4 and MAVLink use NED (North-East-Down; altitude is `-Z`), so no frame conversion is
needed — only unit conversion:

| Field | px4_msgs unit | MAVLink unit |
|---|---|---|
| Position (lat/lon) | degrees (float) | 1e-7 degrees (int) |
| Altitude | m | mm |
| Velocity | m/s | cm/s |
| Battery voltage | V | mV |
| Battery current | A | cA (centiamps) |
| Angles / angular rates | radians, rad/s | unchanged |

**Attitude:** quaternion → Euler via the ZYX aerospace sequence
(`roll = atan2(2(wx+yz), 1-2(x²+y²))`, `pitch = asin(clamp(2(wy-zx), -1, 1))`,
`yaw = atan2(2(wz+xy), 1-2(y²+z²))`).

### QGroundControl Display

- **Fly view:** armed/disarmed, flight mode, GPS fix, compass/attitude, altitude, speed, map trail
- **Vehicle health:** battery widget, link quality, sensor status (GPS/compass/IMU/baro), CPU load

### Troubleshooting

| Issue | Cause | Solution |
|---|---|---|
| No telemetry in QGC | Bridge not running, or PX4 topics empty | `ros2 topic list`; confirm PX4/uXRCE-DDS is up |
| Battery shows 0 V | `battery_status` not yet published | Confirm `/fmu/out/battery_status` is publishing |
| Altitude looks wrong | Home not set / GPS not locked | Set home in QGC; check GPS fix type ≥ 3 |

### Interaction with Other Bridges

All outbound bridges (`gps_spoof`, `telemetry`, `fleet_manager`, `collision`) share port
14550 via `connect()`-and-send — no conflict, since MAVLink multiplexes message types over
one link. `mission_control`/`emergency_wipe` (inbound) sit behind `mavlink_router_node`
instead, for the reason documented in that node's own module docstring.

---

## References

- MAVLink common message set: https://mavlink.io/en/messages/common.html
- MAVLink mission protocol: https://mavlink.io/en/services/mission.html
- QGC Plan view: https://docs.qgroundcontrol.com/master/en/PlanView/PlanView.html
- PX4 documentation: https://docs.px4.io/main/
- `px4_msgs`: https://github.com/PX4/px4_msgs
- Shared codec: `mavlink-bridge/mavlink_v2.py`
- Bridge sources: `mavlink-bridge/{gps_spoof,mission_control,telemetry}_mavlink_bridge.py`
- SAS counterparts: `SAS/my_python_package/{gps_spoof_detector_node,mission_executor_node}.py`
