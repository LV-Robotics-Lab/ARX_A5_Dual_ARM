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

The optional `arx_wrapper.x5` layer provides a ROS/Hydra-independent
`X5DualArm` controller around the pinned `third_party/arx5-sdk` checkout.
On a Linux robot workstation, initialize that submodule, install `.[x5]`, and
run `scripts/install_x5_sdk.sh`. Import and construction remain lazy; only
`connect()` loads `arx5_interface`. Once both controllers exist, connect
immediately performs the ungated, non-trajectory `safe_stop()` transition and
returns in damping mode, never an unknown mode.

Motion is fail-closed. Replay is dry-run by default, and every live path
requires explicit execution, workspace-clearance, e-stop, and exclusive-control
confirmations. Passing those software gates is not physical hardware
acceptance.

X5 motion, homing, and mode changes use the same complete `MotionGate`.
`safe_stop()` and `close()` remain available with a closed gate because they
only request damping and release resources; they never initiate homing.

See [the architecture](docs/ARCHITECTURE.md), [migration guide](docs/MIGRATION.md),
and [upstream policy](docs/UPSTREAM.md). The detailed Chinese collection SOP is
in [README.md](README.md).
