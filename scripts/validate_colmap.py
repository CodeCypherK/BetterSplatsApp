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
import tempfile
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

    # M1: synth session -> reader/replay round trip.
    with tempfile.TemporaryDirectory(prefix="bs_validate_") as tmp:
        session = Path(tmp) / "session"
        frames = 12
        out = subprocess.run(
            [str(synth), str(session), "--frames", str(frames), "--width", "640",
             "--height", "480", "--seed", "3"],
            capture_output=True, text=True, timeout=600,
        )
        if out.returncode != 0:
            print(f"ERROR: bs_synth failed:\n{out.stdout}{out.stderr}",
                  file=sys.stderr)
            return 1

        frame_dirs = sorted((session / "frames").iterdir())
        if len(frame_dirs) != frames:
            print(f"ERROR: expected {frames} frame dirs, found {len(frame_dirs)}",
                  file=sys.stderr)
            return 1
        for d in frame_dirs:
            for name in ("image.jpg", "lidar.depth", "meta.json"):
                if not (d / name).is_file():
                    print(f"ERROR: missing {d / name}", file=sys.stderr)
                    return 1
        if not (session / "ground_truth" / "poses.json").is_file():
            print("ERROR: missing ground truth", file=sys.stderr)
            return 1

        out = subprocess.run(
            [str(replay), str(session), "--info", "--live"],
            capture_output=True, text=True, timeout=600,
        )
        print(out.stdout.strip())
        if out.returncode != 0:
            print(f"ERROR: bs_replay failed:\n{out.stderr}", file=sys.stderr)
            return 1
        if f"fed {frames} frames" not in out.stdout:
            print("ERROR: replay did not feed all frames", file=sys.stderr)
            return 1

    print("OK: synth -> reader -> live-replay round trip")
    print("SKIP: final-solve + pycolmap validation activates in M6/M7")
    return 0


if __name__ == "__main__":
    sys.exit(main())
