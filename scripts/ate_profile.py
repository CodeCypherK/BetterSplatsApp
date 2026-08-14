#!/usr/bin/env python3
"""Read `bs_replay --live`'s per-frame error dumps and say where the error is.

A single ATE number hides two very different failure modes. A tracker that is
uniformly 5 cm off and a tracker that is perfect for 90% of a walk and 40 cm
off at two doorways can print the same RMSE, and they call for opposite fixes.
This reads `live/ate_<pass>.csv` and separates them.

It also compares runs. The anchored ATE is pinned at each run's first tracked
frame, so two runs that tracked different stretches of the same path are not
directly comparable — a run that starts tracking earlier carries a longer
lever arm from its anchor. Given several CSVs this restricts the comparison to
the frames every run tracked, which is the only apples-to-apples version.

    ate_profile.py session/live/ate_scout.csv
    ate_profile.py --compare before/ate_live.csv after/ate_live.csv
"""

import argparse
import csv
import math
import sys


def load(path):
    rows = {}
    with open(path, newline="") as handle:
        for row in csv.DictReader(handle):
            rows[int(row["frame_id"])] = {
                "pos": (float(row["gt_x"]), float(row["gt_y"]), float(row["gt_z"])),
                "anchored": float(row["err_anchored_m"]),
                "rigid": float(row["err_rigid_m"]),
                "rot": float(row["rot_err_rigid_deg"]),
            }
    return rows


def rms(values):
    if not values:
        return float("nan")
    return math.sqrt(sum(v * v for v in values) / len(values))


def percentile(values, frac):
    if not values:
        return float("nan")
    ordered = sorted(values)
    idx = min(len(ordered) - 1, int(frac * len(ordered)))
    return ordered[idx]


def profile(path, rows):
    ids = sorted(rows)
    anchored = [rows[i]["anchored"] for i in ids]
    rigid = [rows[i]["rigid"] for i in ids]

    print(f"\n{path}")
    print(f"  frames                {len(ids)}  ({ids[0]}..{ids[-1]})")
    print(f"  RMSE anchored         {rms(anchored):.3f} m")
    print(f"  RMSE rigid            {rms(rigid):.3f} m")
    print(f"  rigid median / p95    {percentile(rigid, 0.5):.3f} / "
          f"{percentile(rigid, 0.95):.3f} m")
    print(f"  rigid max             {max(rigid):.3f} m")
    print(f"  mean rot (rigid)      {sum(rows[i]['rot'] for i in ids)/len(ids):.2f} deg")

    # Concentration. If the worst 10% of frames hold most of the squared
    # error, the problem is localized and worth chasing to a place on the
    # floor plan; if they hold ~10% of it, the error is spread and the fix is
    # systemic (scale, calibration, drift).
    total_sq = sum(e * e for e in rigid)
    worst = sorted(rigid, reverse=True)[: max(1, len(rigid) // 10)]
    share = sum(e * e for e in worst) / total_sq if total_sq > 0 else 0
    print(f"  worst 10% of frames hold {100*share:.0f}% of the squared error", end="")
    print("  <- localized" if share > 0.5 else "  <- spread")

    # Where. Bucket along the long axis of the walk so the print reads like a
    # floor plan rather than a frame index.
    xs = [rows[i]["pos"][0] for i in ids]
    zs = [rows[i]["pos"][2] for i in ids]
    span_x, span_z = max(xs) - min(xs), max(zs) - min(zs)
    axis, label = (0, "x") if span_x >= span_z else (2, "z")
    lo = min(r["pos"][axis] for r in rows.values())
    hi = max(r["pos"][axis] for r in rows.values())
    if hi - lo < 1e-6:
        return
    buckets = 10
    print(f"  error along {label} (worst frame per bucket):")
    for b in range(buckets):
        a = lo + (hi - lo) * b / buckets
        z = lo + (hi - lo) * (b + 1) / buckets
        here = [r["rigid"] for r in rows.values() if a <= r["pos"][axis] <= z]
        if not here:
            continue
        bar = "#" * min(40, int(40 * max(here) / max(rigid)))
        print(f"    {a:6.1f}..{z:5.1f}  n={len(here):4d}  "
              f"max {max(here):5.2f} m  {bar}")


def compare(paths):
    runs = {p: load(p) for p in paths}
    common = set.intersection(*(set(r) for r in runs.values()))
    print(f"\ncommon frame set: {len(common)} frames tracked by all "
          f"{len(paths)} runs")
    for path, rows in runs.items():
        every = [rows[i]["rigid"] for i in sorted(rows)]
        shared = [rows[i]["rigid"] for i in sorted(common)]
        print(f"  {path}")
        print(f"    all {len(every):5d} frames: rigid RMSE {rms(every):.3f} m")
        print(f"    common {len(shared):5d}   : rigid RMSE {rms(shared):.3f} m "
              f"<- comparable")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv", nargs="+")
    parser.add_argument("--compare", action="store_true",
                        help="restrict to frames tracked by every run")
    args = parser.parse_args()

    if args.compare:
        if len(args.csv) < 2:
            sys.exit("--compare needs at least two CSVs")
        compare(args.csv)
        return
    for path in args.csv:
        profile(path, load(path))


if __name__ == "__main__":
    main()
