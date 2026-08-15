"""Console entry points for configuration and read-only diagnostics."""

from __future__ import annotations

import argparse
import json
from typing import Optional, Sequence

from .config import ArxConfig
from .doctor import run_doctor


def config_main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Print resolved non-secret ARX configuration")
    parser.parse_args(argv)
    print(json.dumps(ArxConfig.from_env().as_dict(), indent=2, sort_keys=True))
    return 0


def doctor_main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Run read-only ARX host diagnostics")
    parser.add_argument("--json", action="store_true", help="emit machine-readable JSON")
    parser.add_argument(
        "--strict",
        action="store_true",
        help="return non-zero if any check fails (default is report-only)",
    )
    args = parser.parse_args(argv)
    results = run_doctor(ArxConfig.from_env())
    if args.json:
        print(json.dumps([result.as_dict() for result in results], indent=2))
    else:
        for result in results:
            label = "PASS" if result.ok else "WARN"
            print(f"[{label}] {result.name}: {result.detail}")
    return 1 if args.strict and any(not result.ok for result in results) else 0
