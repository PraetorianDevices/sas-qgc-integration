# Offboard Controller → QGroundControl Integration

## Overview

The **Telemetry MAVLink Bridge** (`telemetry_mavlink_bridge.py`) streams vehicle telemetry from the offboard controller to QGroundControl in real-time. It converts PX4 messages (`px4_msgs`) to standard MAVLink messages, enabling QGC to display:

- **Position & Navigation:** GPS location, altitude, heading, speed
- **Attitude:** Roll, pitch, yaw angles and angular rates
- **Power Management:** Battery voltage, current, remaining capacity
- **System Health:** Flight mode, armed state, sensor status

---

## Architecture

```
offboard_controller_node (ROS 2 SAS)
  │
  ├─→ /fmu/out/vehicle_local_position (position, velocity, heading)
  ├─→ /fmu/out/vehicle_attitude (roll, pitch, yaw, angular rates)
  ├─→ /fmu/out/vehicle_status (armed state, flight mode, health)
  ├─→ /fmu/out/battery_status (voltage, current, capacity)
  └─→ /fmu/out/sensor_gps (GPS position, heading)
        │
        └─→ telemetry_mavlink_bridge (ROS 2 node)
              │
              └─→ MAVLink Messages (UDP 14550)
                    │
                    ├─ HEARTBEAT (msg 0, 1 Hz)
                    │   • Vehicle armed/disarmed
                    │   • Flight mode
                    │   • System health
                    │
                    ├─ GLOBAL_POSITION_INT (msg 33, 10 Hz)
                    │   • GPS latitude/longitude
                    │   • Altitude (absolute & relative)
                    │   • Heading
                    │   • Velocity (vx, vy, vz)
                    │
                    ├─ ATTITUDE (msg 30, 10 Hz)
                    │   • Roll, pitch, yaw angles
                    │   • Angular rates (p, q, r)
                    │
                    ├─ SYS_STATUS (msg 1, 10 Hz)
                    │   • Battery voltage & current
                    │   • CPU load
                    │   • Sensor health flags
                    │
                    └─ BATTERY_STATUS (msg 147, 10 Hz)
                        • Cell voltages
                        • Current
                        • Capacity
                        • Remaining percentage
```

---

## Component: telemetry_mavlink_bridge.py

**File:** `mavlink-bridge/telemetry_mavlink_bridge.py`

### Inputs

| ROS 2 Topic | Message Type | Rate | Content |
|---|---|---|---|
| `/fmu/out/vehicle_local_position` | VehicleLocalPosition | 50+ Hz | Position (x, y, z), velocity, heading, timestamp |
| `/fmu/out/vehicle_attitude` | VehicleAttitude | 50+ Hz | Quaternion (w, x, y, z), angular rates (p, q, r) |
| `/fmu/out/vehicle_status` | VehicleStatus | 50 Hz | Armed state, flight mode, system status |
| `/fmu/out/battery_status` | BatteryStatus | 1+ Hz | Voltage per cell, current, capacity, temperature |
| `/fmu/out/sensor_gps` | SensorGps | 5+ Hz | Latitude, longitude, altitude, heading, fix type |

### Outputs

| MAVLink Message | ID | Rate | Content |
|---|---|---|---|
| **HEARTBEAT** | 0 | 1 Hz | Vehicle state (armed, mode, health) |
| **GLOBAL_POSITION_INT** | 33 | 10 Hz | GPS, altitude, heading, velocity |
| **ATTITUDE** | 30 | 10 Hz | Roll/pitch/yaw, angular rates |
| **SYS_STATUS** | 1 | 10 Hz | Battery, CPU, sensor health |
| **BATTERY_STATUS** | 147 | 10 Hz | Detailed power info |

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `system_id` | int | 1 | MAVLink system ID (1-255) |
| `component_id` | int | 1 | MAVLink component ID (1 = autopilot) |
| `drone_id` | str | "" | ROS 2 namespace for multi-drone (empty = single drone) |
| `mavlink_host` | str | localhost | UDP hostname/IP for QGC |
| `mavlink_port` | int | 14550 | UDP port for QGC |

---

## Coordinate Frames & Units

### NED Frame (North-East-Down)
- **X-axis:** North (positive forward)
- **Y-axis:** East (positive right)
- **Z-axis:** Down (positive down, so altitude is -Z)

Both PX4 and MAVLink use NED, so conversions are straightforward.

### Units in MAVLink Messages

| Field | Unit | Notes |
|-------|------|-------|
| Position (lat/lon) | 1e-7 degrees | Integer format for precision |
| Altitude | mm (millimeters) | Absolute and relative |
| Velocity | cm/s | Converted from m/s in px4_msgs |
| Battery voltage | mV (millivolts) | Per-cell and total |
| Battery current | cA (centiamps) | 0.01 A resolution |
| Temperature | °C | From battery sensor |
| Angles (roll/pitch/yaw) | radians | Floating-point, [-π, π] |
| Angular rates | rad/s | Floating-point |

---

## Message Details

### HEARTBEAT (msg 0)

Sent **every 1 second** to inform QGC of vehicle state.

```
Fields:
  - type: Vehicle type (2 = MAV_TYPE_QUADROTOR)
  - autopilot: Autopilot type (4 = MAV_AUTOPILOT_PX4)
  - base_mode: System flags (0x80 = MAV_MODE_FLAG_ARMED)
  - custom_mode: PX4 flight mode (0=stabilize, 4=offboard, etc.)
  - system_status: System state (3=standby, 4=active)
```

**QGC Display:** Vehicle status indicator, connection status, armed/disarmed state

---

### GLOBAL_POSITION_INT (msg 33)

Sent **10 times per second** with GPS and global position data.

```
Fields:
  - time_boot_ms: System uptime (milliseconds)
  - lat/lon: GPS position (latitude/longitude × 1e-7 degrees)
  - alt: Absolute altitude above sea level (mm)
  - relative_alt: Altitude above home (mm)
  - vx/vy/vz: Velocity components (cm/s, NED frame)
  - hdg: Heading / yaw (0-360°, ×100)
```

**QGC Display:** Position on map, altitude display, heading compass

---

### ATTITUDE (msg 30)

Sent **10 times per second** with aircraft attitude.

```
Fields:
  - time_boot_ms: System uptime (milliseconds)
  - roll: Roll angle (radians, [-π, π])
  - pitch: Pitch angle (radians, [-π/2, π/2])
  - yaw: Yaw angle (radians, [-π, π])
  - rollspeed/pitchspeed/yawspeed: Angular rates (rad/s)
```

**QGC Display:** Attitude indicator (artificial horizon), roll/pitch/yaw gauges

---

### SYS_STATUS (msg 1)

Sent **10 times per second** with system-level health information.

```
Fields:
  - onboard_control_sensors_present/enabled/health: Bitmask of sensor status
  - load: CPU load (0-1000, where 1000=100%)
  - voltage_battery: Battery voltage (mV)
  - current_battery: Battery current (cA, -1=unknown)
  - battery_remaining: Battery percentage (0-100, -1=unknown)
  - drop_rate_comm: Packet loss rate (0-100%)
  - errors_comm: Communication errors count
```

**QGC Display:** Battery widget, health panel, sensor status, link quality

---

### BATTERY_STATUS (msg 147)

Sent **10 times per second** with detailed battery telemetry.

```
Fields:
  - id: Battery ID (usually 0)
  - type: Battery type (2=MAV_BATTERY_TYPE_LIPO)
  - voltage_cell_v[]: Individual cell voltages (mV, up to 10 cells)
  - current_battery: Battery current (cA)
  - current_consumed: Total charge consumed (mAh)
  - energy_consumed: Total energy consumed (J × 1000)
  - battery_remaining: Remaining percentage (0-100)
  - temperature: Battery temperature (°C, 0-100)
```

**QGC Display:** Battery voltage per cell, current/capacity, power consumption graph

---

## Conversions & Calculations

### GPS Position
```
MAVLink lat/lon = (double lat/lon in degrees) * 1e7
Example: 37.7749° → 377,749,000 (integer)
```

### Altitude
```
Absolute altitude = GPS ellipsoid height (from SensorGps)
Relative altitude = -vehicle_local_position.z (NED: negative Z = up)
```

### Velocity
```
MAVLink velocity (cm/s) = px4_msgs velocity (m/s) * 100
Example: 2.5 m/s → 250 cm/s
```

### Heading (Yaw)
```
MAVLink heading (0-360°) = vehicle_local_position.heading (radians) * 180/π
Stored as integer: heading (×100) to preserve decimals
Example: 1.57 rad (90°) → 9,000 (integer, ×100)
```

### Battery
```
Voltage: sum(cell_voltages) / 1000 = total voltage in V
Current: battery_status.current_a → stored as centiamps (×100)
Remaining: battery_status.remaining → percentage (0-100)
```

### Attitude
```
Quaternion → Euler angles using ZYX aerospace sequence:
  roll = atan2(2(wx+yz), 1-2(x²+y²))
  pitch = asin(clamp(2(wy-zx), -1, 1))
  yaw = atan2(2(wz+xy), 1-2(y²+z²))
```

---

## Running the Integration

### Single-Drone Setup

```bash
# Terminal 1: Source workspace
cd d:/praetoriandevices/SAS
source install/setup.bash

# Terminal 2: Launch complete integration
ros2 launch mavlink-bridge launch_sas_qgc_integration.py system_id:=1

# Terminal 3: Connect QGC
# In QGC: Settings → Comm Links → Add → UDP
# Host: localhost, Port: 14550
```

### Multi-Drone Setup

```bash
# Drone 1
ros2 launch mavlink-bridge launch_sas_qgc_integration.py \
  system_id:=1 \
  drone_id:=drone_1

# Drone 2 (different port)
ros2 launch mavlink-bridge launch_sas_qgc_integration.py \
  system_id:=2 \
  drone_id:=drone_2 \
  mavlink_port:=14551
```

---

## QGroundControl Display

When connected to the telemetry bridge, QGC shows:

### Fly View
- **Top status bar:** Armed/disarmed, flight mode, GPS fix, link quality
- **Compass:** Real-time heading and attitude
- **Altitude indicator:** Altitude above home and sea level
- **Speed gauge:** Ground speed and vertical speed
- **Map:** Vehicle position, heading arrow, trail

### Vehicle Health
- **Battery widget:** Voltage, current, remaining %, discharge rate
- **Signal strength:** Radio and telemetry link quality
- **Sensor status:** GPS fix type, compass, IMU, barometer health
- **System load:** CPU load percentage

### Telemetry Overlay
- Position coordinates (lat/lon)
- Heading (°)
- Ground speed (m/s or knots)
- Altitude MSL and relative to home
- Battery voltage and current

---

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| No telemetry in QGC | Bridge not running or topics empty | Check `ros2 topic list`, verify topics are publishing |
| Incorrect position | Wrong coordinate conversion | Check NED frame handling, verify GPS is locked (fix type ≥ 3) |
| Battery shows 0V | BatteryStatus not published | Verify `/fmu/out/battery_status` is publishing |
| QGC disconnects | UDP socket error | Check port 14550 is available, check firewall settings |
| Altitude incorrect | Home not set | Set home position after drone powers on in QGC |
| Heading fluctuates | Magnetometer interference | Calibrate compass, avoid metal structures during test |

---

## Integration with Other Nodes

### With GPS Spoofing Detector
Both bridges publish to the same UDP port 14550. QGC receives:
- Telemetry streams (HEARTBEAT, POSITION, ATTITUDE, BATTERY)
- Alert streams (STATUSTEXT for spoofing warnings)

No conflicts; MAVLink handles multiple message types.

### With Mission Executor
The telemetry bridge is independent of mission execution. It streams vehicle state regardless of mission status. The mission_executor_node controls waypoint sequences; this bridge just reports what the vehicle is doing.

---

## Performance & Load

### CPU Usage
- **Bridge node:** ~5-10% (single drone, processing 50+ Hz input → 10 Hz output)
- **Scaling:** Linear with drone count (each drone adds ~5-10%)

### Network Usage
- **Outbound:** ~2-3 kB/s per drone to QGC (UDP broadcasts at 10 Hz + 1 Hz heartbeat)
- **Bandwidth requirement:** 5+ Mbps link (easily within WiFi/network limits)

### Latency
- **Sensor → Bridge:** <10 ms (ROS 2 local)
- **Bridge → QGC:** <50 ms (UDP, localhost)
- **Total end-to-end:** ~100-150 ms (acceptable for monitoring)

---

## References

- **MAVLink Spec:** https://mavlink.io/en/messages/common.html
- **PX4 Documentation:** https://docs.px4.io/main/
- **QGC Telemetry:** `QGroundControl/src/Comms/MAVLinkProtocol.h`
- **px4_msgs:** https://github.com/PX4/px4_msgs
- **Coordinate Frames:** https://en.wikipedia.org/wiki/North-East-Down
