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
import json
import re
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
            import numpy as np
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

        # report.json is hand-serialized, and it is a contract with three
        # consumers: the app's SolveReport decoder, this script, and whatever
        # the user points at it. Parsing it here catches a stray comma or a
        # non-finite double at the push that introduces it, rather than on a
        # phone. The per-image table is checked against the model it
        # describes, so a mismatch cannot pass as a formatting success.
        import json as _json
        report = _json.loads((session / "final" / "report.json").read_text())
        images = report.get("images")
        flags = report.get("image_flags")
        if not isinstance(images, list) or not isinstance(flags, dict):
            print("ERROR: report.json missing images/image_flags",
                  file=sys.stderr)
            return 1
        listed = flags.get("images_listed")
        if listed != len(images):
            print(f"ERROR: report.json says images_listed={listed} but lists "
                  f"{len(images)}", file=sys.stderr)
            return 1
        registered_names = {im.name for im in rec.images.values()}
        for entry in images:
            for key in ("frame_id", "name", "registered", "observations",
                        "reproj_rmse_px", "lap_var", "overexp_frac", "flags"):
                if key not in entry:
                    print(f"ERROR: report.json image missing {key}: {entry}",
                          file=sys.stderr)
                    return 1
            # Every image the report calls registered must be in the model,
            # and carry the flag that says so when it is not.
            if entry["registered"] != (entry["name"] in registered_names):
                print(f"ERROR: report.json registered flag disagrees with the "
                      f"model for {entry['name']}", file=sys.stderr)
                return 1
            if not entry["registered"] and "unregistered" not in entry["flags"]:
                print(f"ERROR: {entry['name']} unplaced but not flagged",
                      file=sys.stderr)
                return 1
        # Readiness is recomputed from the FINAL model, so it must at least
        # describe the same reconstruction: regions covering real patches,
        # scores in range, and bounds that are not inverted.
        readiness = report.get("readiness")
        if not isinstance(readiness, dict):
            print("ERROR: report.json missing readiness", file=sys.stderr)
            return 1
        if not readiness.get("present"):
            print("ERROR: readiness absent on a 60/60 solve", file=sys.stderr)
            return 1
        if not 0 <= readiness.get("overall", -1) <= 100:
            print(f"ERROR: readiness overall out of range: "
                  f"{readiness.get('overall')}", file=sys.stderr)
            return 1
        regions = readiness.get("regions") or []
        if not regions:
            print("ERROR: readiness has no regions", file=sys.stderr)
            return 1
        for reg in regions:
            if not 0 <= reg.get("score", -1) <= 100:
                print(f"ERROR: region score out of range: {reg}",
                      file=sys.stderr)
                return 1
            if reg.get("patches", 0) <= 0:
                print(f"ERROR: region with no patches: {reg}", file=sys.stderr)
                return 1
            lo, hi = reg.get("min"), reg.get("max")
            if not lo or not hi or any(h < l for l, h in zip(lo, hi)):
                print(f"ERROR: region bounds inverted: {reg}", file=sys.stderr)
                return 1
        print(f"readiness: {readiness['overall']:.0f}% overall, "
              f"{len(regions)} region(s), "
              f"weakest axis {min(range(5), key=lambda i: readiness['overall_sub'][i])}")

        print(f"report.json: {len(images)} images, "
              f"{flags.get('blurry')} blurry, {flags.get('overexposed')} "
              f"overexposed, {flags.get('unregistered')} unplaced")

        # transforms.json must exist and its camera centres must agree with
        # the actual COLMAP reconstruction (the two are derived independently,
        # so this catches any convention bug in the NeRF export).
        import json
        tj = session / "final" / "transforms.json"
        if not tj.is_file():
            print("ERROR: transforms.json missing", file=sys.stderr)
            return 1
        transforms = json.loads(tj.read_text())
        if len(transforms["frames"]) != n_images:
            print(f"ERROR: transforms.json has {len(transforms['frames'])} "
                  f"frames, pycolmap has {n_images} images", file=sys.stderr)
            return 1
        centers = {img.name: img.projection_center()
                   for img in rec.images.values()}
        max_center_delta = 0.0
        for fr in transforms["frames"]:
            name = fr["file_path"].split("/")[-1]
            if name not in centers:
                print(f"ERROR: transforms.json frame {name} not in model",
                      file=sys.stderr)
                return 1
            tm = fr["transform_matrix"]
            pc = centers[name]
            delta = sum((tm[r][3] - float(pc[r])) ** 2 for r in range(3)) ** 0.5
            max_center_delta = max(max_center_delta, delta)
        if max_center_delta > 1e-3:
            print(f"ERROR: transforms.json centres disagree with COLMAP "
                  f"(max {max_center_delta:.2e} m)", file=sys.stderr)
            return 1
        print(f"transforms.json: {len(transforms['frames'])} frames, "
              f"max centre delta {max_center_delta:.2e} m")

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

    # Split export: parts must be loadable AND carry the combined model's
    # coordinates exactly. A room-at-a-time workflow only works because the
    # parts were never re-anchored — any drift here and separately trained
    # splats would not line up, which is the entire reason to split.
    # 340 frames is not a round number, it is a DENSITY. --two-room is now
    # the circle-and-orbit capture walk (ARCHITECTURE.md), 118 m rather than
    # the 33 m loop this fixture was sized for, and a frame count over a
    # fixed path is a spacing: 340 puts stored images 35 cm apart, which is
    # what the old 110 gave and still 3x sparser than a device stores. The
    # cost of getting this wrong is not a failed assertion, it is a fixture
    # that passes while reconstructing nonsense — measured on this scene,
    # 110 frames (94 cm apart) lands 0.74 m from ground truth and 180 frames
    # fragments into six components and lands 7.0 m out, while 340 comes in
    # at 0.051 m and 0.31 deg with 199 of 340 registered. A fixture that
    # cannot reconstruct the scene cannot detect a regression in
    # reconstructing it.
    with tempfile.TemporaryDirectory(prefix="bs_validate_split_") as tmp:
        split = Path(tmp) / "session"
        out = subprocess.run(
            [str(synth), str(split), "--frames", "340", "--width", "640",
             "--height", "480", "--seed", "4", "--two-room"],
            capture_output=True, text=True)
        if out.returncode != 0:
            print(f"ERROR: split bs_synth failed:\n{out.stdout}{out.stderr}",
                  file=sys.stderr)
            return 1
        out = subprocess.run([str(replay), str(split), "--live"],
                             capture_output=True, text=True)
        out = subprocess.run(
            [str(replay), str(split), "--final", "quality", "--config",
             '{"final_split_max_images":40,"final_split_min_images":10}'],
            capture_output=True, text=True)
        if out.returncode != 0:
            print(f"ERROR: split final solve failed:\n{out.stderr}",
                  file=sys.stderr)
            return 1

        # Gate the fixture's GEOMETRY, not just its file structure. Every
        # assertion below this point — part counts, point counts, shared
        # coordinates — passes just as happily on a model that is metres out
        # of place, which is exactly what this fixture was doing before the
        # frame count was raised. 5.4 cm measured; 25 cm leaves room for
        # feature-detector differences across platforms without leaving room
        # for a wrong reconstruction.
        ate = re.search(r"final ATE: RMSE ([0-9.]+) m", out.stdout)
        if ate is None:
            print("ERROR: split final solve printed no ATE", file=sys.stderr)
            return 1
        if float(ate.group(1)) > 0.25:
            print(f"ERROR: split fixture reconstruction is {ate.group(1)} m "
                  f"from ground truth — the fixture is not reconstructing the "
                  f"scene, so it cannot detect a regression in doing so",
                  file=sys.stderr)
            return 1

        final_dir = split / "final"
        combined = pycolmap.Reconstruction(str(final_dir / "colmap"))

        def pose_of(image):
            m = image.cam_from_world
            m = m() if callable(m) else m
            return np.array(m.translation), np.array(m.rotation.quat)

        reference = {im.name: pose_of(im) for im in combined.images.values()}

        manifest_path = final_dir / "colmap_parts" / "parts.json"
        if not manifest_path.exists():
            print("ERROR: split export wrote no parts.json", file=sys.stderr)
            return 1
        manifest = json.loads(manifest_path.read_text())
        if manifest["part_count"] < 2:
            print(f"ERROR: split produced {manifest['part_count']} part(s)",
                  file=sys.stderr)
            return 1

        seen, memberships, worst_t, worst_r = set(), 0, 0.0, 0.0
        for entry in manifest["parts"]:
            part_dir = final_dir / "colmap_parts" / entry["name"]
            rec = pycolmap.Reconstruction(str(part_dir))
            if rec.num_points3D() < 100:
                print(f"ERROR: {entry['name']} has only "
                      f"{rec.num_points3D()} points", file=sys.stderr)
                return 1
            jpgs = len(list((part_dir / "images").glob("*.jpg")))
            if jpgs != rec.num_images():
                print(f"ERROR: {entry['name']} has {jpgs} jpgs for "
                      f"{rec.num_images()} images", file=sys.stderr)
                return 1
            for im in rec.images.values():
                seen.add(im.name)
                memberships += 1
                t, q = pose_of(im)
                worst_t = max(worst_t, float(np.linalg.norm(t - reference[im.name][0])))
                worst_r = max(worst_r, float(np.linalg.norm(q - reference[im.name][1])))

        if worst_t > 1e-9 or worst_r > 1e-9:
            print(f"ERROR: parts left the combined world frame "
                  f"(translation {worst_t:.3e}, quaternion {worst_r:.3e})",
                  file=sys.stderr)
            return 1
        if len(seen) != combined.num_images():
            print(f"ERROR: parts cover {len(seen)} of "
                  f"{combined.num_images()} images", file=sys.stderr)
            return 1
        print(f"split: {manifest['part_count']} parts, {len(seen)} images "
              f"covered, {memberships - len(seen)} shared at seams, "
              f"pose delta {worst_t:.1e} m")

    print("OK: synth -> live -> two-view -> final solve -> pycolmap, "
          "RAW immutable; hard-scene pipeline within bounds; "
          "split parts share one world frame")
    return 0


if __name__ == "__main__":
    sys.exit(main())
