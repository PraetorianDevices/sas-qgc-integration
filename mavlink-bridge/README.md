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

## Implemented: GPS Spoofing Detector → MAVLink Bridge

**Status:** ✅ Complete

The `gps_spoof_mavlink_bridge.py` node bridges GPS spoofing alerts from the SAS detector to MAVLink STATUSTEXT messages for QGroundControl.

### GPS Spoof Bridge Features

- **Subscribes to:** `/gps_spoof_alert` (from gps_spoof_detector_node)
- **Publishes to:** UDP 14550 as MAVLink STATUSTEXT (msg ID 253)
- **Alert Level Mapping:**
  - `INFO` → MAV_SEVERITY_INFO (green in QGC)
  - `WARNING` → MAV_SEVERITY_WARNING (yellow in QGC)
  - `CRITICAL` → MAV_SEVERITY_CRITICAL (red in QGC)

### Quick Start: GPS Spoofing Detection in QGC

```bash
# Terminal 1: Build and source the workspace
cd d:/praetoriandevices/SAS
colcon build --packages-select mavlink-bridge
source install/setup.sh

# Terminal 2: Launch GPS spoofing detector + MAVLink bridge
ros2 launch mavlink-bridge launch_gps_spoof_qgc.py system_id:=1 mavlink_port:=14550

# Terminal 3: Connect QGroundControl
# In QGC: Comm Links → Add Link → UDP → Host: localhost, Port: 14550
```

**In QGroundControl:**
- Spoofing alerts appear as STATUSTEXT messages in the vehicle status bar
- Red (CRITICAL) alerts trigger mission abort options
- Yellow (WARNING) alerts notify operator to monitor

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
