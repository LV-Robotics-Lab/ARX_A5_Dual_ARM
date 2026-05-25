#!/usr/bin/env python3
"""Operator-facing single-episode SOP check.

Use during a collection session, right after each episode finishes, to get
immediate PASS/FAIL feedback (and a precise "what to fix" hint) without
waiting until end-of-batch verification.

Usage:
    ~/miniconda3/envs/robo_ctrl/bin/python \\
        ~/workspace/ARX_A5_Dual_ARM/data_collection/check_episode.py \\
        ~/workspace/raw_data/egg_to_bowl/0103

Exit codes:
    0 = PASS (counts toward the 100-demo target at 0.10 rad threshold)
    1 = FAIL (file/stream/pickle/stillness violation — episode is unusable as
        a success demo; should be moved to _failed/f000x/ if it's still
        informative as a retained failure)
    2 = USAGE error (e.g. wrong path)

The output stays terse on PASS (one line) and verbose with actionable hints
on FAIL.
"""

from __future__ import annotations

import sys
from pathlib import Path


# Joint-level offender frequencies measured across the 102 near-miss episodes
# in raw_data on 2026-05-24 (see stillness_failure_brief.md). Used to print
# "this is the joint the team is most often missing" guidance when stillness
# is the failure mode.
JOINT_FAILURE_FREQ = {
    "right_arm": {
        4: ("the wrist roll / yaw — keep the spatula handle frozen", 64),
        3: ("the wrist pitch — release into a held grip, no settle micro-motion", 14),
        2: ("the upper-wrist segment — make sure the gripper rests, not floats", 9),
        1: ("the shoulder-roll — torso/arm should be relaxed but planted", 5),
        5: ("the inner roll — recheck end-effector lock", 3),
    },
    "left_arm": {
        5: ("the inner roll — passive arm should be parked, not slowly returning", 21),
        0: ("the base/shoulder", 3),
        4: ("the wrist roll on the passive arm", 3),
    },
}


def _format_stillness_hint(stillness: dict, threshold_rad: float) -> str:
    arms = stillness.get("arms", {}) or {}
    lines = []
    for arm_name, arm in arms.items():
        if arm.get("ok"):
            continue
        rng = arm.get("max_range_rad")
        joint = arm.get("worst_joint_idx")
        excess = rng - threshold_rad if rng is not None else None
        joint_info = JOINT_FAILURE_FREQ.get(arm_name, {}).get(joint)
        line = (f"  - {arm_name}: max_range={rng:.4f} rad "
                f"(over gate by {excess:+.4f} rad), worst joint index = {joint}")
        if joint_info:
            hint, freq = joint_info
            line += (f"\n    -> THIS JOINT FAILED IN {freq}/102 PRIOR NEAR-MISSES;"
                     f" hint: {hint}")
        else:
            line += "\n    -> joint not in the top-N prior offenders — possible new failure mode"
        lines.append(line)
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    import argparse
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("episode_dir", type=Path,
                    help="Path to a single episode dir, e.g. "
                         "~/workspace/raw_data/egg_to_bowl/0103")
    ap.add_argument("--stillness-threshold-rad", type=float, default=0.10,
                    help="Threshold in radians for trailing-2s per-joint stillness "
                         "(default 0.10, matches accepted threshold_decision).")
    ap.add_argument("--json", action="store_true",
                    help="Emit the full verifier dict as JSON instead of "
                         "operator-facing summary.")
    args = ap.parse_args(argv)

    here = Path(__file__).resolve().parent
    if str(here) not in sys.path:
        sys.path.insert(0, str(here))
    from sop_episode_verifier import VerifierConfig, verify_episode  # noqa: E402

    ep = args.episode_dir.expanduser().resolve()
    if not ep.exists():
        print(f"error: path does not exist: {ep}", file=sys.stderr)
        return 2
    if not ep.is_dir():
        print(f"error: not a directory: {ep}", file=sys.stderr)
        return 2

    cfg = VerifierConfig(stillness_max_range_rad=args.stillness_threshold_rad)
    result = verify_episode(ep, cfg)

    if args.json:
        import json
        print(json.dumps(result, indent=2, default=str))
        return 0 if result["ok"] else 1

    print(f"Episode: {ep}")
    print(f"Label:   {result['label']}  "
          f"(threshold {args.stillness_threshold_rad:.3f} rad)")
    if result["ok"]:
        stillness = result["checks"].get("stillness", {}) or {}
        arms = stillness.get("arms", {}) or {}
        ranges = ", ".join(
            f"{n}={v.get('max_range_rad', float('nan')):.4f}rad"
            for n, v in arms.items()
        )
        print(f"VERDICT: PASS  (stillness margins: {ranges})")
        return 0

    print("VERDICT: FAIL")
    print("Reasons:")
    for r in result["reasons"]:
        print(f"  - {r}")

    stillness = result["checks"].get("stillness", {}) or {}
    if not stillness.get("ok"):
        print()
        print("Stillness breakdown (where the trailing 2 s went over the gate):")
        print(_format_stillness_hint(stillness, args.stillness_threshold_rad))
        print()
        print("Note: the team-wide stillness failure pattern lives in")
        print("  ~/workspace/.verifier_reports/stillness_failure_brief.md")
        print("If this episode's worst joint matches that table, the hint above")
        print("is the same correction the next collection batch should target.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
