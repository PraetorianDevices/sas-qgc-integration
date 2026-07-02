# Custom QGC Plugin

This directory contains the custom QGC plugin for SAS fleet integration.

## Overview

Extends QGroundControl's plugin architecture to add:
- Multi-drone fleet control UI (formation, arm/disarm, emergency land)
- GPS spoofing alerts display
- Mission signing validation
- Emergency wipe command button

## Building

(To be implemented)

```bash
# From the parent directory
cmake -S QGroundControl -B build-qgc \
  -DCUSTOM_QML_PATH=$(pwd)/qgc-plugin \
  -DCUSTOM_C++_PLUGIN=$(pwd)/qgc-plugin
```

## References

- QGC Plugin Architecture: `QGroundControl/src/API/QGCCorePlugin.h`
- Custom Example: `QGroundControl/custom-example/`
