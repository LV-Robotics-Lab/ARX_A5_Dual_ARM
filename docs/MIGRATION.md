# Migration from ARX_A5_Dual_ARM

The repository and primary Python distribution are now named `arx_wrapper` and
`arx-wrapper`. Existing data formats, `A5/`, `data_collection/`, `data_replay/`,
and `dual_arm_sys.sh` paths are retained.

On a robot workstation:

```bash
mv ~/workspace/ARX_A5_Dual_ARM ~/workspace/arx_wrapper
cd ~/workspace/arx_wrapper
conda activate robo_ctrl
pip install -e '.[collection,realsense,replay]'
cp config/arx.env.example config/arx.local.env
arx-config
arx-doctor
```

Review `config/arx.local.env` against the actual CAN and camera identities. Do
not copy example values onto a different physical rig without verification.

Behavioral changes:

- `python data_replay/replay_episode.py EPISODE` is now dry-run by default;
- live replay requires `--execute`, `--clearance-confirmed`, `--estop-ready`,
  and `--exclusive-control-confirmed`;
- the collection menu requires an explicit `TEACH` confirmation before gravity
  compensation;
- RealSense bindings come from `ARX_CAM_*_SERIAL` rather than source edits;
- new Python code imports `arx_wrapper`; direct `A5.bimanual` imports are legacy.

After moving a checkout, reinstall the editable package so console-script paths
do not retain the old repository location.
