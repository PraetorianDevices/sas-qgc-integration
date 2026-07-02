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

## Building & Running

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
