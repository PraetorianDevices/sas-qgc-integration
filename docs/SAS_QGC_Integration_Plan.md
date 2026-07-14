# Plan: SAS-QGC Module Integration Analysis

## Context

The SAS repository is a mature ROS 2 autonomous drone system with 15 nodes covering mission execution, fleet management, GPS spoofing detection, and emergency operations. QGroundControl is a professional GCS with a robust plugin architecture. The goal is to identify which SAS modules/nodes should be connected to QGC and design the integration architecture.

**Current State:**
- SAS can parse QGC `.plan` files (JSON) but has no real-time MAVLink bridge to QGC
- QGC can display missions and telemetry from MAVLink vehicles but is unaware of SAS fleet operations
- GPS spoofing detection, mission signing, and fleet coordination exist in SAS but are invisible to the operator in QGC

---

## All 15 SAS Nodes: Integration Status

| Node | Purpose | QGC Connect? | Reason |
|------|---------|--------------|--------|
| **mission_executor_node** | Mission parsing & waypoint sequencing | ✓ **YES** | Gateway to fleet ops; accept real-time mission uploads |
| **offboard_controller_node** | Flight control & PX4 interface | ✓ **YES** | Primary telemetry source (position, attitude, battery) |
| **gps_spoof_detector_node** | GPS spoofing detection (3-strategy) | ✓ **YES** | Critical security alerts; operator must be aware |
| **fleet_manager_node** | Multi-drone formation control | ✓ **YES** | Fleet coordination; formation UI + status aggregation |
| **collision_offboard_controller_node** | Collision avoidance (SF45 LiDAR) | ✓ **YES** | Obstacle visualization; real-time safety display |
| **emergency_wipe_node** | Secure data destruction | ✓ **YES** | Emergency button in QGC; operator-triggered |
| **gesture_bridge_node** | Hand signal recognition (military) | ~ **OPTIONAL** | Failsafe override; adds complexity; lower priority |
| **navigation_control_node** | Waypoint-to-trajectory converter | ✗ NO | Internal plumbing; abstracted by mission_executor |
| **sf45_px4_node** | LiDAR serial I/O | ✗ NO | Already feeds PX4; collision_offboard exposes data |
| **hand_gesture_node** | Camera gesture recognition | ✗ NO | Internal input; fleet-level control via gesture_bridge |
| **image_src_node** | RealSense camera capture | ✗ NO | Internal sensor; no operator relevance |
| **image_zoom_src_node** | Camera capture (2× zoom) | ✗ NO | Internal sensor; no operator relevance |
| **odometry_control_node** | Visual SLAM → EKF fusion | ✗ NO | Internal sensor fusion; PX4 handles output |
| **test_node** | Interactive test harness | ✗ NO | Development/debugging only |
| **mission_test_interface** | Mission executor test UI | ✗ NO | Development/testing only |

**Summary:** **7 nodes for real-world operations** (critical + important), **1 optional**, **7 internal/dev-only** (no operator visibility needed).

---

## Recommended SAS Nodes for QGC Integration

### **Tier 1: Critical (Must Connect)**

#### 1. **mission_executor_node** (`my_python_package/mission_executor_node.py`)
- **Purpose:** Parses QGC `.plan` files, manages waypoint sequences, implements search patterns
- **Current I/O:** Reads .plan JSON, publishes `/mission_executor/status`, `/mission_executor/current_waypoint`
- **QGC Integration Point:** 
  - Accept real-time waypoint uploads from QGC (via MAVLink MISSION_ITEM messages translated to ROS 2)
  - Publish mission progress back to QGC (current waypoint, completion %, next target)
  - Support QGC mission editing: allow mid-flight waypoint insertion/deletion
- **Critical Files:**
  - `my_python_package/mission_executor_node.py:180-350` (mission parser)
  - `config/drones.yaml` (MAVLink system ID mapping)
- **Why:** This is the gateway to fleet operations; all mission commands flow through here.

#### 2. **offboard_controller_node** (`my_python_package/offboard_controller_node.py`)
- **Purpose:** Primary flight control interface; sends trajectories to PX4, receives vehicle state
- **Current I/O:** Subscribes to `/fmu/out/*` (position, attitude, status), publishes `/fmu/in/*` (control commands)
- **QGC Integration Point:**
  - Publish high-rate telemetry: position, attitude, battery, speed, altitude
  - Convert px4_msgs to MAVLink messages (GLOBAL_POSITION_INT, ATTITUDE, SYS_STATUS, BATTERY_STATUS)
  - Publish to UDP port 14550 (QGC standard telemetry port)
- **Critical Files:**
  - `my_python_package/offboard_controller_node.py` (control loop)
  - `config/drones.yaml` (system ID, drone frame type)
- **Why:** Telemetry is the life blood of GCS monitoring; without this QGC is blind to vehicle state.

#### 3. **gps_spoof_detector_node** (`my_python_package/gps_spoof_detector_node.py`)
- **Purpose:** Multi-strategy GPS spoofing detection (heading cross-check, altitude cross-check, PX4 flags)
- **Current I/O:** Subscribes to `/fmu/out/*` (GPS, magnetometer, barometer), publishes `/gps_spoof_alert`, `/gps_spoof_status`
- **QGC Integration Point:**
  - Bridge `/gps_spoof_alert` → MAVLink STATUSTEXT messages with severity WARNING/CRITICAL
  - Display spoofing state in QGC vehicle health panel
  - **Trigger mission abort when spoofing detected** (operator confirmation required)
- **Critical Files:**
  - `my_python_package/gps_spoof_detector_node.py:1-150` (detection logic)
  - `/gps_spoof_alert` topic (JSON alert format)
- **Why:** GPS spoofing is a critical security issue; the operator must be alerted immediately and can choose to abort.

---

### **Tier 2: Important (Should Connect)**

#### 4. **fleet_manager_node** (`my_python_package/fleet_manager_node.py`)
- **Purpose:** Multi-drone coordination, formation control (line, wedge, V-formation), aggregated fleet status
- **Current I/O:** Subscribes to `/{drone_id}/drone_position`, `/fleet/gesture_command`, publishes `/{drone_id}/navigation_control/mission_command`
- **QGC Integration Point:**
  - Publish `/fleet/status` (aggregated position, battery, mission progress for all drones)
  - Allow QGC UI to select formation type (line, wedge, etc.) and broadcast to fleet
  - Display formation visualization overlay in QGC Fly View (custom QML panel)
- **Critical Files:**
  - `my_python_package/fleet_manager_node.py:50-150` (formation logic)
  - `/fleet/status` topic structure
- **Why:** Multi-drone operations are a key differentiator; visualization and control from QGC unlock operator productivity.

#### 5. **collision_offboard_controller_node** (`my_python_package/collision_offboard_controller_node.py`)
- **Purpose:** Enhanced offboard control with collision avoidance (obstacle distance from SF45 LiDAR)
- **Current I/O:** Subscribes to `/fmu/in/obstacle_distance`, publishes collision-aware trajectories
- **QGC Integration Point:**
  - Visualize obstacle grid / heatmap on QGC map (custom QML)
  - Display collision risk score (real-time)
  - **Optional:** Allow operator to adjust collision avoidance thresholds from QGC UI
- **Critical Files:**
  - `my_python_package/collision_offboard_controller_node.py`
  - `/fmu/in/obstacle_distance` (from SF45 LiDAR via `sf45_px4_node`)
- **Why:** Collision avoidance is critical for safety; real-time visualization helps operator understand vehicle behavior.

---

### **Tier 3: Optional (Nice to Have)**

#### 6. **sf45_px4_node** (`my_python_package/sf45_px4_node.py`)
- **Purpose:** Reads Lightware SF45 rotating LiDAR, converts to obstacle distance for PX4
- **QGC Integration:** Already integrated via PX4 → relies on collision_offboard_controller to expose obstacle data
- **Note:** No direct QGC connection needed; obstacle data flows through PX4 → telemetry

#### 7. **gesture_bridge_node** (`my_python_package/gesture_bridge_node.py`)
- **Purpose:** Hand gesture recognition (TC 3-21.60 military signals) → fleet commands
- **QGC Integration:** 
  - **Optional feature:** Allow QGC to trigger gesture-based actions via custom button (e.g., "Send Rally Point Signal")
  - Could also use gesture recognition as physical failsafe override
- **Note:** Lower priority; adds complexity but provides a cool non-radio command option.

#### 8. **emergency_wipe_node** (`my_python_package/emergency_wipe_node.py`)
- **Purpose:** Multi-pass secure deletion of flight logs and mission data
- **QGC Integration:**
  - Add "Emergency Wipe" button in QGC Emergency panel
  - Calls `/emergency_wipe/execute` ROS 2 service
  - Confirmatory dialog to prevent accidental activation
- **Critical Files:**
  - `my_python_package/emergency_wipe_node.py`
  - `/emergency_wipe/execute` (ROS 2 Trigger service)
- **Note:** Important for security-sensitive operations; should have two-factor confirmation.

---

## Integration Architecture

### **High-Level Data Flow**

```
QGroundControl
├─ Mission Upload → [MAVLink MISSION_ITEM msgs]
│  └─ MAVLink Bridge Node (NEW)
│     └─ /mission_executor/load_mission (ROS 2 service)
│        └─ mission_executor_node
│           └─ offboard_controller_node → PX4 → Drone
│
├─ Telemetry Subscribe ← [MAVLink HEARTBEAT, POSITION, ATTITUDE, BATTERY]
│  └─ MAVLink Bridge Node (NEW)
│     ├─ offboard_controller_node (/fmu/out/*)
│     ├─ gps_spoof_detector_node (/gps_spoof_alert)
│     ├─ fleet_manager_node (/fleet/status)
│     └─ collision_offboard_controller_node (/fmu/in/obstacle_distance)
│
├─ Custom QGC Plugin (NEW)
│  ├─ SASPlugin.h/cpp (extends QGCCorePlugin)
│  ├─ mavlinkMessage() hook → filter custom messages
│  ├─ preSaveToJson() → inject mission metadata (priority, classification)
│  ├─ createQmlApplicationEngine() → register SAS QML modules
│  └─ SASFleetView.qml (custom Fly View overlay)
│     ├─ Formation selector UI
│     ├─ Fleet status grid
│     ├─ GPS spoofing alert banner
│     └─ Collision risk heatmap
│
└─ Emergency Commands
   ├─ "Land All" button → /fleet/gesture_command (broadcast)
   └─ "Wipe Data" button → /emergency_wipe/execute (service call)
```

### **Key Components to Build**

1. **MAVLink Bridge Node** (in `mavlink-bridge/` directory)
   - Runs as a ROS 2 node alongside SAS
   - Subscribes to `/fmu/out/*` topics (telemetry from offboard_controller)
   - Publishes MAVLink UDP packets to `localhost:14550` (QGC standard)
   - Subscribes to MAVLink COMMAND messages from QGC, converts to ROS 2 topics
   - **Files to Create:**
     - `mavlink-bridge/mavlink_bridge_node.py` (main bridge logic)
     - `mavlink-bridge/CMakeLists.txt` or setup.py (build config)

2. **Custom QGC Plugin** (in `qgc-plugin/` directory)
   - Extend QGCCorePlugin to intercept all vehicle telemetry
   - Register custom QML for fleet visualization
   - Inject mission signing metadata into `.plan` files
   - **Files to Create:**
     - `qgc-plugin/SASPlugin.h/cpp` (plugin entry point)
     - `qgc-plugin/SASFleetView.qml` (custom Fly View panel)
     - `qgc-plugin/CMakeLists.txt` (integration with QGC build)

3. **Mission Signer Integration** (activate existing code)
   - Wire up `security/mission_signer.py` to sign missions before upload
   - Enable `mission_verifier.py` in `mission_executor_node` to validate signatures on receipt
   - **Files to Modify:**
     - `my_python_package/mission_executor_node.py:185` (uncomment MAVROS subscription, add verifier)

4. **Secure Launch Configuration** (activate existing code)
   - Use existing `launch/secure_launch.py` to enable ROS 2 DDS-Security
   - Ensures all node-to-node communication is encrypted
   - **Files to Check:**
     - `launch/secure_launch.py`
     - `security/keystore/` (DDS-Security certificates)

---

## Critical Files Identified

| File | Purpose | Integration Touch Points |
|------|---------|--------------------------|
| `mission_executor_node.py` | Mission parsing, waypoint sequencing | Accept real-time mission uploads from QGC |
| `offboard_controller_node.py` | Flight control, telemetry aggregation | Publish vehicle state to MAVLink bridge |
| `gps_spoof_detector_node.py` | GPS spoofing detection | Alert QGC via MAVLink STATUSTEXT |
| `fleet_manager_node.py` | Multi-drone formation control | Publish fleet status, accept formation commands |
| `collision_offboard_controller_node.py` | Collision avoidance logic | Expose obstacle distance to QGC visualization |
| `config/drones.yaml` | Drone IDs, MAVLink system IDs | Must match QGC vehicle system IDs |
| `mission_signer.py` (security/) | Mission signing utility | Sign missions before QGC upload |
| `mission_verifier.py` (other_code/) | Signature validation | Validate missions in mission_executor |
| `QGCCorePlugin.h` (QGC/src/API/) | Plugin interface | Extend with SASPlugin |
| `custom-example/` (QGC/custom-example/) | Reference plugin implementation | Template for SASPlugin |

---

## Verification Strategy

### **Phase 1: MAVLink Bridge Verification**
1. Launch single SAS drone: `ros2 launch SAS single_drone.launch.py`
2. Launch MAVLink bridge node separately
3. Connect QGC to `localhost:14550` (UDP)
4. Verify QGC receives:
   - HEARTBEAT every 1 Hz
   - GLOBAL_POSITION_INT (vehicle position)
   - ATTITUDE (roll/pitch/yaw)
   - SYS_STATUS (battery voltage, CPU load)
5. Send MAVLink command (e.g., ARM) from QGC → verify drone responds

### **Phase 2: Mission Upload Verification**
1. Create mission in QGC (3-5 waypoints)
2. Export as `.plan` file
3. Verify mission_executor_node accepts MAVLink MISSION_ITEM commands
4. Fly mission, verify waypoint progression
5. Send waypoint update mid-flight, verify drone responds

### **Phase 3: GPS Spoofing Alert Verification**
1. Verify `/gps_spoof_alert` messages are published
2. MAVLink bridge converts to STATUSTEXT
3. QGC displays alert banner with red warning icon
4. Operator can choose to abort mission

### **Phase 4: Fleet Operations Verification**
1. Launch 2-3 drones: `ros2 launch SAS multi_drone.launch.py`
2. Verify `/fleet/status` aggregates all drone positions
3. QGC custom plugin displays all drones on map
4. Select formation type (line/wedge/V) in QGC UI
5. Verify all drones move to formation positions

### **Phase 5: Mission Signing Verification**
1. Enable mission_verifier in mission_executor_node
2. Sign mission with `mission_signer.py`
3. Load signed mission → verify signature is valid
4. Tamper with mission JSON → verify signature validation fails
5. Attempt to load unsigned mission → verify rejection (or warning)

### **Phase 6: End-to-End GCS Integration**
1. Launch integrated SAS + MAVLink bridge + QGC plugin
2. Plan multi-drone mission in QGC
3. Upload to fleet
4. Monitor telemetry, formation, GPS spoofing in QGC UI
5. Trigger emergency wipe from QGC "Emergency" panel
6. Verify data destruction and system shutdown

---

## Summary: Which Nodes Connect to QGC

| Node | Direct MAVLink Bridge | Custom QGC Plugin | Mission Command | Telemetry Output |
|------|----------------------|-------------------|-----------------|------------------|
| **mission_executor_node** | ✓ (MISSION_ITEM cmds) | ✓ (sign/verify) | ✓ | ✓ (progress) |
| **offboard_controller_node** | ✓ (COMMAND_LONG cmds) | — | — | ✓ (HEARTBEAT, POSITION, ATTITUDE, BATTERY) |
| **gps_spoof_detector_node** | ✓ (STATUSTEXT alerts) | ✓ (display banner) | — | ✓ (alerts) |
| **fleet_manager_node** | — | ✓ (formation UI) | ✓ (formation cmds) | ✓ (fleet status) |
| **collision_offboard_controller** | — | ✓ (obstacle viz) | — | ✓ (obstacle grid) |
| **gesture_bridge_node** | — | ✓ (optional override) | — | — |
| **emergency_wipe_node** | ✓ (MAV_CMD_STORAGE) | ✓ (emergency button) | — | ✓ (status) |

**Answer:** **7 of 15 SAS nodes should be connected to QGC** for a complete integrated GCS experience. The remaining 8 nodes (odometry_control, sf45_px4, hand_gesture, image_src, navigation_control, test_node, mission_test_interface, image_zoom_src) are internal to the SAS system and don't need direct QGC visibility.
