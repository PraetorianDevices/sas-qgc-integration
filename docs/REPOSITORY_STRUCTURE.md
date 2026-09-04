# Repository Structure

A map of this repository: what lives where, and what each piece is responsible for.

**Last updated:** 2026-08-03

---

## 1. What this repository is

`praetoriandevices-gcs` is an **integration repository**. It contains almost no
product code of its own — instead it pulls two large codebases together as git
submodules and adds the glue between them:

| Piece | Role |
| --- | --- |
| **SAS** (submodule) | ROS 2 autonomous drone system — mission execution, fleet management, GPS-spoof detection, emergency wipe, gesture/vision control |
| **QGroundControl** (submodule) | C++/Qt ground control station, with Praetorian Devices customizations |
| **mavlink-bridge** (this repo) | ROS 2 package translating `px4_msgs` ⇄ MAVLink UDP, so QGC can talk to SAS drones |
| **qgc-plugin** (this repo) | Placeholder for the future custom QGC fleet-control plugin |
| **docs** (this repo) | Integration design, status, and test documentation |

Both submodules track their own **`develop`** branch, and the outer repo's
active branch is also `develop`. All three have independent histories.

---

## 2. Top-level layout

```
d:\praetoriandevices\
├── README.md                 # Project overview, submodule cloning/updating
├── .gitmodules               # Submodule pins (SAS, QGroundControl → develop)
├── .gitignore                # Ignores colcon build/ install/ log/, venvs, __pycache__
├── .claude/                  # Claude Code local settings (settings.local.json)
│
├── SAS/                      # ── submodule ── ROS 2 drone system
├── QGroundControl/           # ── submodule ── Qt/C++ ground control station
│
├── mavlink-bridge/           # ROS 2 package: the MAVLink ⇄ px4_msgs glue
├── qgc-plugin/               # README only — Phase 3, not started
├── docs/                     # Integration docs (this file lives here)
│
├── build/  install/  log/    # colcon artifacts — generated, gitignored
```

`build/`, `install/`, and `log/` are produced by `colcon build` at the workspace
root and are never committed. `install/setup.bash` is what you source to get
`ros2 run mavlink-bridge …` and `ros2 run my_python_package …` on your path.

---

## 3. `mavlink-bridge/` — the integration layer

This is where nearly all first-party code in the outer repo lives. It is a
standard ament-python ROS 2 package (`package.xml`, `setup.py`, `setup.cfg`,
`resource/mavlink-bridge`), but note that **modules sit at the package root**
rather than in a nested Python package directory — so `setup.py` lists them via
`py_modules` instead of `packages`, and launch files are installed by explicit
name rather than a `launch/*.py` glob.

### 3.1 Protocol core

| File | Purpose |
| --- | --- |
| [mavlink_v2.py](../mavlink-bridge/mavlink_v2.py) | Hand-rolled MAVLink 2.0 frame encoder/decoder — 10-byte header, CRC16-CCITT with message CRC_EXTRA. Every bridge builds its frames through this. Verified byte-for-byte against `pymavlink`. |

### 3.2 Bridge nodes

Each bridge is a ROS 2 node pairing one SAS subsystem with its MAVLink
representation. **Outbound** bridges only `connect()`+send, so they can share
the QGC port freely. **Inbound** bridges must `bind()` a socket, which is why
the router exists (see below).

| Node | Direction | What it does |
| --- | --- | --- |
| [gps_spoof_mavlink_bridge.py](../mavlink-bridge/gps_spoof_mavlink_bridge.py) | out | `/gps_spoof_alert` → STATUSTEXT (INFO/WARNING/CRITICAL → MAV_SEVERITY) |
| [telemetry_mavlink_bridge.py](../mavlink-bridge/telemetry_mavlink_bridge.py) | out | PX4 telemetry → HEARTBEAT, LOCAL_POSITION_NED, GLOBAL_POSITION_INT, ATTITUDE, BATTERY_STATUS |
| [fleet_manager_mavlink_bridge.py](../mavlink-bridge/fleet_manager_mavlink_bridge.py) | out | Per-drone fleet mission-state summaries → STATUSTEXT |
| [collision_mavlink_bridge.py](../mavlink-bridge/collision_mavlink_bridge.py) | out | SF45 lidar obstacle sweep → OBSTACLE_DISTANCE |
| [mission_control_bridge.py](../mavlink-bridge/mission_control_bridge.py) | **in**+out | Bidirectional mission transfer: QGC waypoint upload → SAS mission executor, and progress/current-waypoint back to QGC. Enforces mission signing on upload. |
| [emergency_wipe_mavlink_bridge.py](../mavlink-bridge/emergency_wipe_mavlink_bridge.py) | **in** | QGC `COMMAND_LONG` → SAS emergency-wipe service call |
| [mavlink_router_node.py](../mavlink-bridge/mavlink_router_node.py) | relay | Binds the single external QGC UDP port (14550) and fans every datagram byte-for-byte to each inbound bridge on its own internal port (mission_control 14551, wipe 14556); relays their replies back out. Pure byte relay — no MAVLink parsing. |

The router solves a real constraint: QGC opens **one** UDP comm link per
vehicle, but two processes cannot cleanly bind the same UDP port. Because both
inbound bridges already bind a configurable port and learn their reply address
from the last sender, neither needed code changes to sit behind the router.

### 3.3 Launch files & tooling

| File | Purpose |
| --- | --- |
| [launch_sas_qgc_integration.py](../mavlink-bridge/launch_sas_qgc_integration.py) | Brings up the full stack: all 6 bridges + router |
| [launch_gps_spoof_qgc.py](../mavlink-bridge/launch_gps_spoof_qgc.py) | Minimal launch — GPS-spoof bridge only |
| [test_gps_spoof_alert_generator.py](../mavlink-bridge/test_gps_spoof_alert_generator.py) | Console script publishing synthetic spoof alerts: `--count`, `--rate`, `--level` |
| [demo_qgc_wire_protocol.py](../mavlink-bridge/demo_qgc_wire_protocol.py) | Standalone demo/verification of the wire format |
| [conftest.py](../mavlink-bridge/conftest.py) | Root-level pytest config (makes root modules importable) |

### 3.4 Tests

```
mavlink-bridge/tests/
├── README.md            # Test-suite guide
├── TEST_SUMMARY.md      # Coverage summary
├── conftest.py          # Shared fixtures; ROS 2 / message stubbing
├── unit/                # No ROS 2 required — frame format, CRC, conversions,
│                        #   per-bridge logic, mission signing
├── integration/         # Require ROS 2 — one file per bridge, plus
│                        #   test_router_with_real_bridges_integration.py and
│                        #   launch_full_integration.py for manual runs
└── fixtures/
    └── alert_generator.py
```

Tests import the **real** production modules (not stubs) and drive real
sockets. `tests/README.md` documents a slightly older file layout than what's
on disk — trust the directory listing over that README.

---

## 4. `SAS/` — the ROS 2 drone system (submodule)

ament-python package named **`my_python_package`**. Repo README is titled
"DroneCodeV2" and is mostly WSL/Ubuntu 24.04 environment setup — follow
`support_docs/New_Build` as it instructs.

```
SAS/
├── my_python_package/        # All ROS 2 nodes (the actual package)
│   ├── offboard_controller_node.py
│   ├── collision_offboard_controller_node.py
│   ├── navigation_control_node.py
│   ├── odometry_control_node.py
│   ├── mission_executor_node.py       # Executes QGC .plan-derived missions
│   ├── mission_verifier.py            # EC P-256 mission signature verification
│   ├── mission_test_interface.py
│   ├── gps_spoof_detector_node.py     # 3 strategies: heading, altitude, PX4 flags
│   ├── emergency_wipe_node.py
│   ├── fleet_manager_node.py
│   ├── hand_gesture_node.py
│   ├── gesture_bridge_node.py         # Gesture cmds, gated on safe mission state
│   ├── image_src_node.py
│   ├── image_zoom_src_node.py
│   ├── sf45_px4_node.py               # SF45 lidar driver
│   ├── tank_detector.py
│   ├── test_node.py
│   ├── example_mission.json
│   └── other_code/                    # Research/standalone code, not ROS nodes
│       ├── MilHandSignals/            # TC 3-21.60 gesture recognition, YOLOv8 +
│       │                              #   pose models, checksums, signal_tests/
│       ├── ObjectDetection/           # EfficientDet-Lite tracking + tests
│       ├── EmergencyWipe/             # Standalone wipe implementation
│       ├── Hardpoints/                # hp_Control.py + Arduino .ino sketches
│       ├── Voice/                     # Offline speech recognizer
│       ├── PoseDroneControl.py, DronePadDector.py
│       └── *_tests/                   # Tests colocated with their subject
│
├── launch/
│   ├── single_drone.launch.py
│   ├── multi_drone.launch.py
│   └── secure_launch.py               # ⚠ gitignored (see §7)
│
├── config/drones.yaml        # Fleet roster: id + MAVLink system_id per drone.
│                             #   Tested up to 16; system_id must be unique.
├── security/                 # ⚠ gitignored — mission_signer.py + keystore/
├── mission_signing.pem       # ⚠ gitignored signing key
│
├── tests/
│   ├── unit/                 # Per-node unit tests + unit/gesture/ subsuite
│   └── integration/          # Mock GPS/lidar publishers, spoof & executor tests
├── test/                     # ament lint tests (copyright, flake8, pep257)
│                             #   + e2e_emergency_wipe_demo.py
│
├── image-compression/        # Separate subsystem: drone-side image processing
│   ├── drone_processor/      #   processor.py, transmitter.py
│   ├── ground_station/       #   receiver.py, link_simulator.py
│   └── fleet.example.json
│
├── support_docs/             # Environment setup, project plan, TC 3-21.60 PDF
├── package.xml setup.py setup.cfg requirements.txt resource/
└── build/ install/ log/ venv_export/   # generated, gitignored
```

Node entry points are declared in `SAS/setup.py` under `console_scripts` — that
list is the authoritative index of runnable nodes (`ros2 run my_python_package
<node>`).

---

## 5. `QGroundControl/` — the GCS (submodule)

An upstream QGroundControl fork with Praetorian Devices features. Standard QGC
layout; the parts you'll actually touch:

| Path | Contents |
| --- | --- |
| `src/` | Main C++/QML source, organized by subsystem: `Vehicle/`, `MAVLink/`, `MissionManager/`, `FlyView/`, `FlightMap/`, `Comms/`, `UI/`, `QmlControls/`, `FirmwarePlugin/`, `AutoPilotPlugins/`, `Settings/`, `Camera/`, `Gimbal/`, `Joystick/`, `ADSB/`, `GPS/`, `Terrain/`, `VideoManager/`, `Viewer3D/`, `Utilities/`, `API/` |
| `praetoriandevices_docs/` | **Praetorian feature specs**, one file per task (`feature-*-task<N>.txt`) — supply drop, KML export, multi-drone control, return-home randomization, encryption layers, camera actions, logo change, etc. Plus `custom-buttons-test-guide.md`. Start here to understand what's been customized. |
| `custom-example/` | Upstream's custom-build example — the reference pattern for a plugin |
| `cmake/ CMakeLists.txt CMakePresets.json.template justfile` | Build system |
| `deploy/ android/ tools/ scripts/` | Packaging and platform support |
| `resources/ translations/ *.qrc` | Assets, i18n, Qt resource manifests |
| `test/ Testing/ CTestCustom.cmake.in` | Test harness |
| `docs/ CODING_STYLE.md THREAD_SAFETY_ANALYSIS.md CHANGELOG.md` | Upstream docs |

---

## 6. `docs/` — integration documentation

Read in roughly this order:

| File | What it's for |
| --- | --- |
| [IMPLEMENTATION_STATUS.md](IMPLEMENTATION_STATUS.md) | **Start here.** Current status, what is verified live, the operational requirements for launching, and which SAS nodes connect to QGC and why |
| [IMPLEMENTATION_ROADMAP.md](IMPLEMENTATION_ROADMAP.md) | Phased plan (0, 1, 1.5, 2, 3) with per-phase status |
| [IMPLEMENTATION_STATUS.md](IMPLEMENTATION_STATUS.md) | Current state, bridge-by-bridge protocol verification table |
| [GPS_SPOOFING_QGC_INTEGRATION.md](GPS_SPOOFING_QGC_INTEGRATION.md) | Spoof detector → STATUSTEXT design |
| [OFFBOARD_CONTROLLER_QGC_INTEGRATION.md](OFFBOARD_CONTROLLER_QGC_INTEGRATION.md) | Telemetry bridge design |
| [MISSION_EXECUTOR_QGC_INTEGRATION.md](MISSION_EXECUTOR_QGC_INTEGRATION.md) | Bidirectional mission control design |
| [INTEGRATION_TEST_CHECKLIST.md](INTEGRATION_TEST_CHECKLIST.md) | Manual end-to-end validation steps (note its own "read before running" caveat) |
| REPOSITORY_STRUCTURE.md | This file |

**Phase status at a glance:** Phases 0, 1, 1.5, and 2 are complete — all seven
bridge/router nodes exist and are protocol-verified, mission signing is wired
into the live upload path, and the full test suite runs against real modules.
**Phase 3 (the QGC custom plugin) has not been started** — hence
`qgc-plugin/` containing only a README.

---

## 7. Gotchas worth knowing

- **`SAS/.gitignore` ignores `launch/` and `security/` wholesale.** So
  `launch/secure_launch.py`, `security/mission_signer.py`, `security/keystore/`,
  and `mission_signing.pem` exist on disk but are **not tracked** — the two
  launch files that *are* tracked were force-added. Don't assume a fresh clone
  of SAS gives you a working secure launch or signing key.
- **Three independent git histories.** Committing in the outer repo does not
  commit submodule changes; each has its own `develop`. Clone with
  `--recurse-submodules`; update with `git submodule update --remote`.
- **`mavlink-bridge` modules live at the package root**, so any new module must
  be added to *both* `py_modules` and `console_scripts` in `setup.py` — a
  missing `py_modules` entry installs a console script that can't import.
- **`SAS/received/`, `SAS/sssss`** and similar are stray artifacts, not
  structure. `received/` is gitignored at the outer level.
- **PX4 DDS topic names are versioned** and have drifted before (see recent
  commits on both `develop` branches). If telemetry silently stops, suspect a
  topic-name mismatch first.
