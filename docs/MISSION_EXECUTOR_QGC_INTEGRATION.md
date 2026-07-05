# Mission Executor → QGroundControl Integration

## Overview

The **Mission Control Bridge** (`mission_control_bridge.py`) enables **bidirectional mission control** between QGroundControl and the SAS mission executor. It handles:

- **Mission Upload:** QGC → SAS (operator plans mission in QGC, uploads waypoints)
- **Mission Execution:** SAS executes mission while tracking progress
- **Real-time Updates:** SAS → QGC (current waypoint, mission status)
- **Mission Editing:** QGC can modify waypoints mid-flight (future enhancement)

---

## Architecture

```
QGroundControl (Mission Planner)
  │
  ├─ Create/Edit Mission (drag-drop waypoints)
  ├─ Upload MISSION_ITEM messages
  └─ Monitor MISSION_CURRENT progress
       │
       ↓ (MAVLink UDP 14550)
       │
mission_control_bridge (ROS 2 node)
  │
  ├─ MISSION_REQUEST_LIST (from QGC)
  │   → List missions on vehicle
  │
  ├─ MISSION_ITEM (from QGC)
  │   → Store waypoint in memory
  │   → Convert to SAS mission format
  │   → Send MISSION_ACK (accepted/rejected)
  │
  ├─ MISSION_REQUEST (from QGC)
  │   → Retrieve waypoint from memory
  │   → Send back MISSION_ITEM
  │
  └─ Receive mission status (from mission_executor_node)
      → Publish MISSION_CURRENT (current waypoint index)
       │
       ↓ (ROS 2 topics)
       │
mission_executor_node (SAS)
  │
  ├─ Load mission from /mission_executor/load_mission
  ├─ Execute waypoint sequence
  ├─ Publish mission progress to /mission_executor/status
  └─ Send commands to offboard_controller_node
```

---

## MAVLink Messages Handled

### Incoming (QGC → SAS)

| Message | ID | Purpose |
|---------|----|---------| 
| **MISSION_REQUEST_LIST** | 43 | QGC asking "how many waypoints?" |
| **MISSION_ITEM** | 39 | QGC uploading a waypoint |
| **MISSION_REQUEST** | 40 | QGC requesting a specific waypoint |

### Outgoing (SAS → QGC)

| Message | ID | Purpose |
|---------|----|---------| 
| **MISSION_COUNT** | 44 | "We have N waypoints" |
| **MISSION_ITEM** | 39 | "Here's waypoint #X" |
| **MISSION_ACK** | 47 | "Mission accepted" or "Mission rejected" |
| **MISSION_CURRENT** | 42 | "Currently executing waypoint #X" (every 1 Hz) |
| **MISSION_ITEM_REACHED** | 61 | "Waypoint #X reached" (optional, when reached) |

---

## Message Format: MISSION_ITEM

**Size:** 39 bytes (MAVLink 2.0)

```
Offset  Type    Field         Description
------  ------  -----------   ----------------------------------------
0-1     uint16  seq           Waypoint sequence number (0-indexed)
2       uint8   frame         Coordinate frame (0=global, 3=relative-alt)
3-4     uint16  command       MAV_CMD (16=waypoint, 21=land, 22=takeoff)
5       uint8   current       Is this the current waypoint? (0=no, 1=yes)
6       uint8   autocontinue  Jump to next waypoint automatically? (1=yes)
7-10    float   param1        Command param 1 (hold time, radius, etc.)
11-14   float   param2        Command param 2
15-18   float   param3        Command param 3
19-22   float   param4        Command param 4
23-26   int32   x             Latitude (degrees × 1e7)
27-30   int32   y             Longitude (degrees × 1e7)
31-34   float   z             Altitude (meters above sea level)
```

### Example: Waypoint at San Francisco, 100m altitude

```json
{
  "sequence": 0,
  "frame": 3,              // MAV_FRAME_GLOBAL_RELATIVE_ALT
  "command": 16,           // NAV_WAYPOINT
  "current": 0,
  "autocontinue": 1,
  "params": [0, 0, 0, 0],
  "position": {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "altitude": 100.0
  }
}
```

---

## Bridge Operations

### Upload Mission from QGC

```
QGC User:
  1. Create mission (5 waypoints)
  2. Click "Upload to Vehicle"

QGC:
  1. Send MISSION_REQUEST_LIST
  2. Bridge responds: MISSION_COUNT (5)
  3. For each waypoint (0-4):
     - Send MISSION_ITEM
     - Bridge ACKs: MISSION_ACK (accepted)
  4. Mission upload complete

Mission Control Bridge:
  - Stores 5 waypoints in memory
  - Converts to SAS JSON format
  - Publishes to mission_executor_node via /mission_executor/load_mission
  - mission_executor_node parses and begins execution
```

### Track Mission Progress

```
Mission Executor:
  1. Execute waypoint #0
  2. Publish status: {"current_waypoint": 0, "in_progress": true}
  3. Move to waypoint #1
  4. Publish status: {"current_waypoint": 1, "in_progress": true}
  ... etc ...

Mission Control Bridge:
  - Every 1 Hz, publish MISSION_CURRENT with index
  - QGC displays "Executing waypoint 1 of 5"
  - Progress bar updates in real-time
```

---

## ROS 2 Integration

### Topics

**Published:**
- `/mission_executor/load_mission` (std_msgs/String)
  - Publishes JSON mission to be loaded
  - Format: `{"waypoints": [...], "home": {...}, ...}`

**Subscribed:**
- `/mission_executor/status` (std_msgs/String)
  - Receives mission progress: `{"current_waypoint": N, "in_progress": true/false}`
  - Rate: Variable (depends on mission executor)

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `system_id` | int | 1 | MAVLink system ID |
| `component_id` | int | 1 | MAVLink component ID |
| `drone_id` | str | "" | ROS 2 namespace (empty = single drone) |
| `mavlink_host` | str | localhost | Listen address |
| `mavlink_port` | int | 14550 | UDP port |

---

## Coordinate Frames

The bridge supports multiple MAVLink coordinate frames:

| Frame ID | Name | Altitude Reference | Use Case |
|----------|------|-------------------|----------|
| 0 | MAV_FRAME_GLOBAL | Mean sea level (MSL) | Absolute GPS waypoints |
| 3 | MAV_FRAME_GLOBAL_RELATIVE_ALT | Relative to home | Home-relative waypoints (typical) |
| 10 | MAV_FRAME_LOCAL_NED | Local NED origin | Nearby local missions |

**Default:** Frame 3 (relative altitude) — most common for QGC missions

---

## Running the Integration

### Single-Drone Setup

```bash
# Launch complete integration (detector + telemetry + missions)
ros2 launch mavlink-bridge launch_sas_qgc_integration.py system_id:=1

# In QGC:
# 1. Settings → Comm Links → Add → UDP
# 2. Host: localhost, Port: 14550
# 3. Click "Connect"
# 4. Go to Plan tab → Create mission with waypoints
# 5. Click "Upload" → Mission uploads to SAS
```

### Multi-Drone Setup

```bash
# Drone 1
ros2 launch mavlink-bridge launch_sas_qgc_integration.py \
  system_id:=1 \
  drone_id:=drone_1

# Drone 2 (different UDP port)
ros2 launch mavlink-bridge launch_sas_qgc_integration.py \
  system_id:=2 \
  drone_id:=drone_2 \
  mavlink_port:=14551
```

---

## QGroundControl Display

When connected, QGC shows:

### Plan Tab
- **Waypoint list** - all uploaded waypoints
- **Map** - waypoint positions, flight path
- **Edit controls** - add/remove/reorder waypoints
- **Upload button** - send mission to vehicle

### Fly Tab
- **Mission progress bar** - "Executing waypoint 3 of 5"
- **Active waypoint** - highlighted on map
- **Next waypoint** - indicated with arrow
- **Distance/time** - to next waypoint, to mission end

---

## Error Handling

| Error | MAVLink Response | QGC Display |
|-------|------------------|-------------|
| Mission format invalid | MISSION_ACK (ERROR) | Red error message |
| Waypoint out of bounds | MISSION_ACK (ERROR) | Mission rejected |
| Communication lost | No response (timeout) | "Lost vehicle connection" |
| Bridge crashed | Mission stays in memory | Can re-upload after restart |

---

## Limitations & Future Work

### Current Limitations
- ❌ No mission editing mid-flight (waypoints are locked during execution)
- ❌ No geofence/rally point support
- ❌ No complex mission items (do-set-mode, do-change-speed with timing)

### Future Enhancements
- ✅ Mid-flight waypoint insertion/deletion
- ✅ Geofence and rally point support
- ✅ Mission pausing and resumption
- ✅ Conditional waypoints (if/then logic)
- ✅ Home position verification

---

## Testing

### Unit Tests (TODO)
```python
def test_mission_item_parsing():
    """Parse MAVLink MISSION_ITEM payload."""
    # Binary waypoint data from QGC
    # Extract: seq, lat, lon, alt, command
    # Assert conversions are correct

def test_coordinate_frame_conversion():
    """Convert between MAVLink frames."""
    # Global (MSL) → Relative alt
    # NED local → Global GPS
```

### Integration Tests (TODO)
```python
def test_qgc_mission_upload():
    """End-to-end mission upload flow."""
    # 1. Send MISSION_REQUEST_LIST
    # 2. Bridge responds with count
    # 3. Send 5 MISSION_ITEM messages
    # 4. Bridge ACKs each
    # 5. Verify mission_executor receives JSON
    # 6. Verify mission executes

def test_mission_progress_tracking():
    """Track mission progress in real-time."""
    # 1. Start mission execution
    # 2. Publish status updates from executor
    # 3. Verify MISSION_CURRENT messages sent
    # 4. QGC progress bar updates
```

---

## References

- **MAVLink Mission Protocol:** https://mavlink.io/en/services/mission.html
- **QGC Mission Planner:** https://docs.qgroundcontrol.com/master/en/PlanView/PlanView.html
- **PX4 Mission Types:** https://px4.io/
