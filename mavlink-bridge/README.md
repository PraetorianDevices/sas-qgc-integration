# MAVLink Bridge Node

ROS 2 adapter that translates between px4_msgs (PX4 uORB) and MAVLink protocol, enabling QGroundControl to communicate with SAS drones.

## Overview

Subscribes to PX4 uORB topics and publishes MAVLink packets to UDP port 14550 (QGC default), and vice versa.

### ROS 2 Topics

**Subscribed (PX4 output → MAVLink):**
- `/px4_*/vehicle_local_position`
- `/px4_*/sensor_gps`
- `/px4_*/vehicle_attitude`
- `/px4_*/battery_status`
- `/px4_*/vehicle_status`

**Published (MAVLink input → PX4):**
- `/px4_*/offboard_control_mode`
- `/px4_*/trajectory_setpoint`
- `/px4_*/vehicle_command`

### MAVLink Messages

- **HEARTBEAT** — sent every 1Hz with system_id/component_id
- **LOCAL_POSITION_NED** — vehicle position/velocity updates
- **GPS_RAW_INT** — global position, heading
- **ATTITUDE** — roll/pitch/yaw
- **BATTERY_STATUS** — battery voltage/current/capacity
- **STATUSTEXT** — text alerts (GPS spoofing, warnings)
- **COMMAND_LONG** — incoming commands from QGC (arm, disarm, takeoff, land, etc.)

## Implemented Integrations

### 1. GPS Spoofing Detector → MAVLink Bridge

**Status:** ✅ Complete

The `gps_spoof_mavlink_bridge.py` node bridges GPS spoofing alerts from the SAS detector to MAVLink STATUSTEXT messages for QGroundControl.

### 2. Offboard Controller (Telemetry) → MAVLink Bridge

**Status:** ✅ Complete

The `telemetry_mavlink_bridge.py` node bridges vehicle telemetry from the offboard controller to MAVLink messages for QGroundControl.

### GPS Spoof Bridge Features

- **Subscribes to:** `/gps_spoof_alert` (from gps_spoof_detector_node)
- **Publishes to:** UDP 14550 as MAVLink STATUSTEXT (msg ID 253)
- **Alert Level Mapping:**
  - `INFO` → MAV_SEVERITY_INFO (green in QGC)
  - `WARNING` → MAV_SEVERITY_WARNING (yellow in QGC)
  - `CRITICAL` → MAV_SEVERITY_CRITICAL (red in QGC)

### Telemetry Bridge Features

- **Subscribes to:** PX4 telemetry topics (via offboard controller)
  - `/fmu/out/vehicle_local_position` (position, velocity)
  - `/fmu/out/vehicle_attitude` (roll, pitch, yaw)
  - `/fmu/out/vehicle_status` (armed state, flight mode)
  - `/fmu/out/battery_status` (voltage, current, capacity)
  - `/fmu/out/sensor_gps` (GPS position, heading)
- **Publishes to:** UDP 14550 as MAVLink messages
  - **HEARTBEAT** (msg 0) - Vehicle armed/disarmed, flight mode (1 Hz)
  - **GLOBAL_POSITION_INT** (msg 33) - GPS position, altitude, heading (10 Hz)
  - **ATTITUDE** (msg 30) - Roll, pitch, yaw, angular velocities (10 Hz)
  - **SYS_STATUS** (msg 1) - Battery, CPU load, sensor health (10 Hz)
  - **BATTERY_STATUS** (msg 147) - Detailed battery telemetry (10 Hz)

### Quick Start: Complete SAS-QGC Integration

```bash
# Terminal 1: Build and source the workspace
cd d:/praetoriandevices/SAS
colcon build --packages-select mavlink-bridge
source install/setup.sh

# Terminal 2: Launch complete integration (detector + GPS bridge + telemetry bridge)
ros2 launch mavlink-bridge launch_sas_qgc_integration.py system_id:=1 mavlink_port:=14550

# Terminal 3: Connect QGroundControl
# In QGC: Comm Links → Add Link → UDP → Host: localhost, Port: 14550
```

**In QGroundControl (after connecting):**
- **Top status bar:** Vehicle armed/disarmed, flight mode, GPS status
- **Telemetry widget:** Real-time position, altitude, heading, speed
- **Battery widget:** Voltage, current, capacity, remaining percentage
- **Compass:** Live heading and attitude
- **Alerts:** GPS spoofing warnings (yellow) and critical alerts (red)

### Quick Start: GPS Spoofing Detection Only

If you only want GPS spoofing alerts:

```bash
ros2 launch mavlink-bridge launch_gps_spoof_qgc.py system_id:=1 mavlink_port:=14550
```

### Quick Start: Telemetry Only

If you only want vehicle telemetry:

```bash
ros2 run mavlink-bridge telemetry_mavlink_bridge \
  --ros-args \
  -p system_id:=1 \
  -p mavlink_port:=14550
```

### Configuration

Launch with parameters:

```bash
ros2 launch mavlink-bridge launch_gps_spoof_qgc.py \
  system_id:=1 \
  mavlink_host:=localhost \
  mavlink_port:=14550
```

Or run standalone:

```bash
ros2 run mavlink-bridge gps_spoof_mavlink_bridge \
  --ros-args \
  -p system_id:=1 \
  -p component_id:=200 \
  -p mavlink_host:=localhost \
  -p mavlink_port:=14550
```

---

## Building & Running (Full MAVLink Bridge - Future)

(To be implemented)

```bash
cd SAS && colcon build --packages-select mavlink-bridge
source install/setup.sh
ros2 run mavlink-bridge mavlink_bridge_node --ros-args -p system_id:=1 -p udp_port:=14550
```

Then in QGC: **Comm Links** → **Add Link** → **Type: UDP**, Host: `<drone-ip>`, Port: `14550`

## References

- MAVLink spec: https://mavlink.io/
- QGC MAVLink protocol: `QGroundControl/src/Comms/MAVLinkProtocol.h`
- PX4 message spec: https://px4.io/
