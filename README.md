# Praetorian Devices GCS Integration

This repository integrates **SAS** (ROS 2 autonomous drone system) with **QGroundControl** (professional ground control station) to provide unified mission planning, fleet coordination, and security monitoring.

## Repository Structure

```
.
├── SAS/                    # ROS 2 autonomous drone control system (submodule: develop branch)
├── QGroundControl/         # Ground control station UI (submodule: develop branch)
├── qgc-plugin/             # Custom QGC plugin for fleet control (TODO)
├── mavlink-bridge/         # MAVLink ↔ px4_msgs adapter node (TODO)
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

## Integration Points (Planned)

1. **MAVLink Bridge** — Translates px4_msgs ↔ MAVLink UDP so QGC can connect to SAS drones
2. **Mission File Sync** — Standardize on QGC `.plan` format for mission planning
3. **Custom QGC Plugin** — Multi-drone fleet control, formation commands from QGC UI
4. **GPS Spoofing Alerts** — Bridge ROS 2 spoofing detections to MAVLink STATUSTEXT
5. **Mission Signing & Emergency Ops** — EC P-256 signature validation, emergency wipe via QGC

## Development

Estimated timeline: **15–24 days** (1 developer) or **10–14 days** (2 developers in parallel)

See the integration points above for task breakdown.
