# ARX upstream and vendor maintenance

The `A5/` directory is a retained vendor snapshot derived from
[`ARXroboticsX/A5`](https://github.com/ARXroboticsX/A5). It also contains
lab-tested paths, prebuilt Linux shared libraries, CAN helpers, and local field
documentation. It is not currently a Git submodule, and the repository history
does not record a reliable upstream commit pin.

Do not replace `A5/` with upstream HEAD or delete its prebuilt libraries as a
mechanical cleanup. First reconcile local changes and binary provenance.

For an upstream refresh:

1. Record the exact upstream commit and create a dedicated update branch.
2. Diff source, URDF, CAN rules, build scripts, and binary artifacts separately.
3. Preserve the public `arx_wrapper` safety and configuration boundary.
4. Build on the Linux robot workstation; macOS cannot validate the ELF SDK.
5. Run wrapper unit tests and dry-run replay before importing the vendor binary.
6. Run read-only CAN/SDK diagnostics, then a separately authorized, low-speed
   single-arm hardware gate with the e-stop ready.
7. Record the accepted upstream commit and workstation/SDK evidence here.

The Python distribution does not package `A5/`. Live hardware use therefore
requires a repository checkout (or `ARX_VENDOR_ROOT` pointing to one) and a
locally built SDK.

## X5 controller SDK

`third_party/arx5-sdk` is a separate, pinned submodule sourced from
[`MrSecant/arx5-sdk`](https://github.com/MrSecant/arx5-sdk). The accepted
baseline is commit `ce0d1e76a9237de30908ab259dd9f3e4621056cf`, matching the
previous PrometheusV4 X5 integration.

The `0.2.0` wrapper API owns this X5 boundary. A successful
`X5DualArm.connect()` must return with both sides in damping through
`safe_stop()`; neither connection nor shutdown may call home.

Build it only on a compatible Linux robot workstation:

```bash
git submodule update --init third_party/arx5-sdk
python -m pip install -e '.[x5]'
./scripts/install_x5_sdk.sh
```

The install script resolves the SDK relative to this wrapper checkout and
verifies the active environment's SOEM ABI before building. An SDK update must
use a dedicated branch, record the new gitlink, pass the fake-SDK unit tests,
then repeat read-only host diagnostics and separately authorized hardware
validation. Do not update the legacy `A5/` snapshot and the X5 submodule as one
undifferentiated change.
