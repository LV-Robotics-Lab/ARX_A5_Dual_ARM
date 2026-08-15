# arx_wrapper architecture

`arx_wrapper` separates reusable control contracts from the field workflow and
the vendored ARX SDK. Importing the wrapper must work on a development machine
that has no ROS, CAN interface, camera, or ARX binary extension.

| Layer | Location | Responsibility |
|---|---|---|
| Public wrapper | `src/arx_wrapper/` | Typed configuration, read-only diagnostics, lifecycle, command validation, and motion gates |
| Field workflows | `data_collection/`, `data_replay/`, `dual_arm_sys.sh` | ROS 1 publishing, recording, replay, and operator UI |
| Offline data tools | `training/`, non-hardware replay/collection utilities | Episode validation, conversion, visualization, and loader checks |
| Vendored SDK | `A5/` | ARX binary bindings, URDFs, CAN helpers, and retained legacy compatibility |

New Python consumers should import `arx_wrapper`. Existing `A5.bimanual`
imports remain available inside a checkout for compatibility, but hardware
entry points in this repository now route through `ArxArm` or `ArxDualArm`.

## Safety contract

Construction is lazy: `ArxArm(...)` does not import the vendor extension or
connect to CAN. Configuration and `arx-doctor` are read-only. Methods that can
change hardware state call `MotionGate.require_motion()` immediately before the
vendor SDK. Real execution needs all of the following:

- `execute=True`;
- workspace clearance confirmed;
- emergency stop ready;
- exclusive control source confirmed.

Trajectory replay is dry-run by default. The collection menu asks the operator
to type `TEACH` before it starts the arm publisher. These checks do not prove
physical readiness; calibration, wiring, clearances, and an on-site low-speed
test remain separate acceptance gates.

## Configuration

`ArxConfig.from_env()` is the canonical resolver. Machine-specific values live
in ignored `config/arx.local.env`, based on `config/arx.env.example`. Camera
roles are bound by USB serial, never by enumeration order. Data and generated
artifacts stay outside Git under `ARX_DATA_ROOT`.

## Dependency policy

The core package has no mandatory third-party Python dependency. Data, replay,
and RealSense dependencies are extras. ROS 1 and the ARX binary SDK are host
capabilities and are not silently installed by the Python package.
