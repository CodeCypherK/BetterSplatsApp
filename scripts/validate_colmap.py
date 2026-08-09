#!/usr/bin/env python3
"""End-to-end reconstruction validation for CI.

Pipeline (activates milestone by milestone):
  M1: bs_synth generates a synthetic session; the session reader round-trips.
  M4: bs_replay --live tracks the synthetic session within ATE bounds.
  M6: bs_replay --final produces a reconstruction that beats the live one.
  M7: the COLMAP export loads in pycolmap and passes format invariants.

Until a stage's tool support exists, that stage reports SKIP so the script
is safe to wire into CI from M0.
"""
import argparse
import subprocess
import sys
from pathlib import Path


def tool(build_dir: Path, name: str) -> Path | None:
    for candidate in (
        build_dir / "tools" / name.removeprefix("bs_") / name,
        build_dir / "tools" / name / name,
    ):
        if candidate.exists():
            return candidate
    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--build-dir", type=Path, default=Path("build/linux-rel"))
    args = parser.parse_args()

    synth = tool(args.build_dir, "bs_synth")
    replay = tool(args.build_dir, "bs_replay")
    if synth is None or replay is None:
        print(f"ERROR: tools not found under {args.build_dir}", file=sys.stderr)
        return 1

    # Smoke: both tools run and the engine selftest passes through them.
    for exe in (synth, replay):
        out = subprocess.run(
            [str(exe), "--selftest"], capture_output=True, text=True, timeout=120
        )
        print(f"{exe.name} --selftest: {out.stdout.strip()}")
        if out.returncode != 0:
            print(f"ERROR: {exe.name} selftest failed", file=sys.stderr)
            return 1

    print("SKIP: synthetic end-to-end validation activates in M1+ "
          "(session generation not yet implemented)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
