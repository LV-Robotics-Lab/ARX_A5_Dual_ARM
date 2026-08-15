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
