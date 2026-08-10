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

    # M1: synth session -> reader/replay round trip. 60 frames over a 60 deg
    # sweep matches realistic handheld angular speed (~30 deg/s at 30 fps).
    with tempfile.TemporaryDirectory(prefix="bs_validate_") as tmp:
        session = Path(tmp) / "session"
        frames = 60
        out = subprocess.run(
            [str(synth), str(session), "--frames", str(frames), "--width", "640",
             "--height", "480", "--seed", "3", "--sweep", "60"],
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

        # Snapshot the RAW layer before any engine processing touches the
        # session — compared again after the final solve.
        import hashlib
        digest = hashlib.sha256()
        for frame_dir in sorted((session / "frames").iterdir()):
            for name in ("image.jpg", "lidar.depth", "meta.json"):
                digest.update((frame_dir / name).read_bytes())
        raw_digest = digest.hexdigest()

        # M4: live tracking with ATE bounds against ground truth (--check
        # enforces >=70% tracked, ATE < 0.10 m, mean rot < 2 deg).
        out = subprocess.run(
            [str(replay), str(session), "--info", "--live", "--check"],
            capture_output=True, text=True, timeout=1800,
        )
        print(out.stdout.strip())
        if out.returncode != 0:
            print(f"ERROR: live replay check failed:\n{out.stderr}",
                  file=sys.stderr)
            return 1
        if f"fed {frames} frames" not in out.stdout:
            print("ERROR: replay did not feed all frames", file=sys.stderr)
            return 1

        # M2: two-view relative pose on the synthetic session, checked
        # against ground truth (bounds enforced by --check). Gap 6 gives
        # ~6 deg / 0.35 m per pair — proper two-view conditioning; smaller
        # gaps are correctly declined as weak-parallax by the estimator.
        out = subprocess.run(
            [str(replay), str(session), "--two-view", "6", "--check"],
            capture_output=True, text=True, timeout=900,
        )
        print(out.stdout.strip())
        if out.returncode != 0:
            print(f"ERROR: two-view check failed:\n{out.stderr}", file=sys.stderr)
            return 1

        # M6/M7: final global reconstruction with ground-truth bounds
        # (--check: >=90% registration, ATE < 0.05 m, rot < 1 deg), then the
        # definitive gate — pycolmap must load the export.
        out = subprocess.run(
            [str(replay), str(session), "--final", "quality", "--check"],
            capture_output=True, text=True, timeout=3600,
        )
        print(out.stdout.strip()[-2000:])
        if out.returncode != 0:
            print(f"ERROR: final solve check failed:\n{out.stderr}",
                  file=sys.stderr)
            return 1

        colmap_dir = session / "final" / "colmap"
        try:
            import pycolmap
        except ImportError:
            print("ERROR: pycolmap unavailable — install it in CI",
                  file=sys.stderr)
            return 1
        rec = pycolmap.Reconstruction(str(colmap_dir))
        n_images = rec.num_images()
        n_points = rec.num_points3D()
        track_len = rec.compute_mean_track_length()
        reproj = rec.compute_mean_reprojection_error()
        print(f"pycolmap: {rec.num_cameras()} cameras, {n_images} images, "
              f"{n_points} points, track {track_len:.2f}, "
              f"reproj {reproj:.3f} px")
        if n_images < 0.9 * frames:
            print(f"ERROR: pycolmap sees only {n_images} images",
                  file=sys.stderr)
            return 1
        if n_points < 500:
            print(f"ERROR: too few points ({n_points})", file=sys.stderr)
            return 1
        if track_len < 2.5:
            print(f"ERROR: mean track length {track_len:.2f} < 2.5",
                  file=sys.stderr)
            return 1
        if reproj > 1.5:
            print(f"ERROR: mean reprojection {reproj:.2f} px > 1.5",
                  file=sys.stderr)
            return 1
        if not (session / "final" / "dense.ply").is_file():
            print("ERROR: dense.ply missing", file=sys.stderr)
            return 1
        if not (session / "final" / "report.json").is_file():
            print("ERROR: report.json missing", file=sys.stderr)
            return 1

        # Immutability invariant: the full pipeline must leave the RAW layer
        # byte-identical (live drift never bakes into sensor data).
        import hashlib
        digest = hashlib.sha256()
        for frame_dir in sorted((session / "frames").iterdir()):
            for name in ("image.jpg", "lidar.depth", "meta.json"):
                digest.update((frame_dir / name).read_bytes())
        if digest.hexdigest() != raw_digest:
            print("ERROR: RAW layer changed during processing!",
                  file=sys.stderr)
            return 1

    # Real-world robustness: the SAME pipeline must survive a hard capture —
    # auto-exposure drift, motion blur, heavy RGB noise, coarse depth — within
    # the same --check bounds (live >=70%/0.10m/2deg, final >=90%/0.05m/1deg).
    # This guards every push against realism regressions, not just clean synth.
    with tempfile.TemporaryDirectory(prefix="bs_validate_hard_") as tmp:
        hard = Path(tmp) / "session"
        out = subprocess.run(
            [str(synth), str(hard), "--frames", "60", "--width", "640",
             "--height", "480", "--seed", "5", "--sweep", "60", "--hard"],
            capture_output=True, text=True, timeout=600,
        )
        if out.returncode != 0:
            print(f"ERROR: hard bs_synth failed:\n{out.stdout}{out.stderr}",
                  file=sys.stderr)
            return 1

        out = subprocess.run(
            [str(replay), str(hard), "--live", "--check"],
            capture_output=True, text=True, timeout=1800,
        )
        print("hard: " + out.stdout.strip().splitlines()[-1])
        if out.returncode != 0:
            print(f"ERROR: hard live check failed:\n{out.stderr}",
                  file=sys.stderr)
            return 1

        out = subprocess.run(
            [str(replay), str(hard), "--final", "quality", "--check"],
            capture_output=True, text=True, timeout=3600,
        )
        print("hard: " + out.stdout.strip().splitlines()[-1])
        if out.returncode != 0:
            print(f"ERROR: hard final solve check failed:\n{out.stderr}",
                  file=sys.stderr)
            return 1

        rec = pycolmap.Reconstruction(str(hard / "final" / "colmap"))
        print(f"hard pycolmap: {rec.num_images()} images, "
              f"{rec.num_points3D()} points, "
              f"track {rec.compute_mean_track_length():.2f}, "
              f"reproj {rec.compute_mean_reprojection_error():.3f} px")
        if rec.num_images() < 0.9 * frames:
            print(f"ERROR: hard export sees only {rec.num_images()} images",
                  file=sys.stderr)
            return 1
        if rec.num_points3D() < 500:
            print(f"ERROR: hard export too few points ({rec.num_points3D()})",
                  file=sys.stderr)
            return 1

    print("OK: synth -> live -> two-view -> final solve -> pycolmap, "
          "RAW immutable; hard-scene pipeline within bounds")
    return 0


if __name__ == "__main__":
    sys.exit(main())
