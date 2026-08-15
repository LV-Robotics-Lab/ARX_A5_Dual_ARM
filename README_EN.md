# arx_wrapper

`arx_wrapper` is LV Robotics Lab's safety-gated Python wrapper and field data
workflow for the ARX A5 dual-arm platform. It provides typed environment
configuration, read-only diagnostics, lazy vendor-SDK loading, explicit motion
gates, ROS 1 teaching/recording, replay, visualization, and offline dataset
tools.

```bash
python -m pip install -e '.[dev,replay]'
arx-config
arx-doctor
pytest
```

Core imports and unit tests do not require ROS, CAN, RealSense, or the Linux ARX
binary. Live use requires a checkout containing `A5/`, a locally built vendor
SDK, the robot workstation's ROS environment, and verified hardware mapping in
`config/arx.local.env`.

Motion is fail-closed. Replay is dry-run by default, and every live path
requires explicit execution, workspace-clearance, e-stop, and exclusive-control
confirmations. Passing those software gates is not physical hardware
acceptance.

See [the architecture](docs/ARCHITECTURE.md), [migration guide](docs/MIGRATION.md),
and [upstream policy](docs/UPSTREAM.md). The detailed Chinese collection SOP is
in [README.md](README.md).
