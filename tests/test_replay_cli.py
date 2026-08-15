import pickle
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
REPLAY = REPO_ROOT / "data_replay" / "replay_episode.py"


def episode(tmp_path: Path) -> Path:
    target = tmp_path / "0000"
    target.mkdir()
    rows = np.zeros((2, 7), dtype=np.float64)
    rows[:, 6] = -1.0
    state = {
        side: {"joints": rows, "timestamps": [1000, 1017]} for side in ("left_arm", "right_arm")
    }
    with (target / "state.pkl").open("wb") as stream:
        pickle.dump(state, stream)
    return target


def test_replay_is_dry_run_by_default(tmp_path: Path) -> None:
    completed = subprocess.run(
        [sys.executable, str(REPLAY), str(episode(tmp_path))],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert "not importing the vendor SDK or connecting to arms" in completed.stdout


def test_execute_without_confirmations_fails_before_sdk_load(tmp_path: Path) -> None:
    completed = subprocess.run(
        [sys.executable, str(REPLAY), str(episode(tmp_path)), "--execute"],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode != 0
    assert "execution gate incomplete" in completed.stderr
    assert "ARX A5 Python bindings are unavailable" not in completed.stderr
