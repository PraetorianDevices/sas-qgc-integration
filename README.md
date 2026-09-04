# Praetorian Devices GCS Integration

This repository integrates **SAS** (ROS 2 autonomous drone system) with **QGroundControl** (professional ground control station) to provide unified mission planning, fleet coordination, and security monitoring.

## Repository Structure

```
.
├── SAS/                    # ROS 2 autonomous drone control system (submodule: develop branch)
├── QGroundControl/         # Ground control station UI (submodule: develop branch)
├── qgc-plugin/             # Custom QGC plugin for fleet control (not started)
├── mavlink-bridge/         # MAVLink ↔ px4_msgs bridges + router (implemented, live-verified)
├── docs/                   # Status, architecture and per-feature integration docs
└── README.md
```

## Submodules

Both submodules track their `develop` branches for active development:

- **SAS** (develop): ROS 2 nodes, mission execution, GPS spoofing detection, fleet management
- **QGroundControl** (develop): C++/Qt ground control station with plugin architecture

### Cloning

```bash
git clone --recurse-submodules https://github.com/PraetorianDevices/praetoriandevices-gcs.git
cd praetoriandevices-gcs
```

### Updating Submodules

To pull the latest commits from both develop branches:

```bash
git submodule update --remote
```

## Integration Points

| # | Integration | Status |
|---|---|---|
| 1 | **MAVLink Bridge** — px4_msgs ↔ MAVLink UDP so QGC can connect to SAS drones | ✅ implemented, verified live |
| 2 | **Mission File Sync** — QGC `.plan` format for mission planning | ✅ implemented, verified live |
| 3 | **Custom QGC Plugin** — multi-drone fleet control from the QGC UI | ⏳ not started (C++/Qt/QML) |
| 4 | **GPS Spoofing Alerts** — ROS 2 detections → MAVLink `STATUSTEXT` | ✅ implemented, verified live |
| 5 | **Mission Signing & Emergency Ops** — signature validation, emergency wipe via QGC | ✅ implemented; wipe verified but runs in `STUB_MODE` and is not yet in a launch file |

All seven MAVLink bridges and the three QGC-facing SAS control nodes have been run against
real PX4 SITL and real QGroundControl, including an armed flight and a real mission upload.
See [docs/IMPLEMENTATION_STATUS.md](docs/IMPLEMENTATION_STATUS.md) for what is verified, the
bugs that live testing found, and the remaining gaps.

## Running the Stack

Two settings are easy to get wrong and fail **silently** — see the Operational Requirements
section of [docs/IMPLEMENTATION_STATUS.md](docs/IMPLEMENTATION_STATUS.md) for the full list.

```bash
# 1. PX4 SITL, then at the pxh> prompt stop PX4's own mavlink instance
#    (it contends with our router on port 14550):
#       mavlink stop -u 18570

# 2. Micro XRCE-DDS agent
MicroXRCEAgent udp4 -p 8888

# 3. SAS control nodes (runs them under the /drone_1 namespace)
ros2 launch my_python_package single_drone.launch.py

# 4. MAVLink bridges. sas_namespace:=drone_1 is REQUIRED to match step 3 --
#    without it, mission uploads are published where nothing is listening.
ros2 launch mavlink-bridge launch_sas_qgc_integration.py \
  system_id:=1 \
  sas_namespace:=drone_1 \
  mavlink_host:=<address QGC is reachable at>
```

In QGC, add a UDP Comm Link on port 14550 pointing at the address **this stack** runs on.
Under WSL2 that must be the WSL2 interface IP (`ip addr show eth0`), not `localhost` —
WSL2's localhost forwarding drops UDP silently, and the IP changes on WSL restart.

## Documentation

| Doc | Contents |
|---|---|
| [docs/IMPLEMENTATION_STATUS.md](docs/IMPLEMENTATION_STATUS.md) | **Start here.** Current status, what is verified live, operational requirements, known limitations |
| [docs/IMPLEMENTATION_ROADMAP.md](docs/IMPLEMENTATION_ROADMAP.md) | Phase-by-phase history and what remains |
| [docs/REPOSITORY_STRUCTURE.md](docs/REPOSITORY_STRUCTURE.md) | Repository and package layout |
| [docs/OFFBOARD_CONTROLLER_QGC_INTEGRATION.md](docs/OFFBOARD_CONTROLLER_QGC_INTEGRATION.md) | Telemetry bridge reference |
| [docs/MISSION_EXECUTOR_QGC_INTEGRATION.md](docs/MISSION_EXECUTOR_QGC_INTEGRATION.md) | Mission upload/download bridge reference |
| [docs/GPS_SPOOFING_QGC_INTEGRATION.md](docs/GPS_SPOOFING_QGC_INTEGRATION.md) | GPS spoofing alert bridge reference |
| [docs/INTEGRATION_TEST_CHECKLIST.md](docs/INTEGRATION_TEST_CHECKLIST.md) | Manual test procedure for the GPS spoofing path |
