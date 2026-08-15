#!/usr/bin/env python3
"""Draw the capture walk, top-down, as an SVG.

A capture plan is geometry, and geometry is worth looking at. The most
expensive defect in this harness so far — a lap that pointed the camera
square at a wall half a metre away — was invisible in every summary
statistic and obvious the moment the view directions were drawn.

    build/linux-rel/tools/synth/bs_synth --dump-plan plan.json
    scripts/plot_capture_plan.py plan.json -o plan.svg

The walls and furniture come from the dump, not from this file: a picture
of a walk drawn against a stale set of rooms is worse than no picture,
because it looks authoritative.
"""
import argparse
import json
import math
import sys

# Phase ids, matching CapturePhase in tools/synth/synth_scene.h.
LAP, ORBIT_OBJECT, ORBIT_DOORWAY, THROUGH_DOORWAY, APPROACH = range(5)

STYLE = {
    LAP: ("#2b7fff", "circling the room"),
    ORBIT_OBJECT: ("#ff8a3d", "orbiting an object"),
    ORBIT_DOORWAY: ("#e0409a", "orbiting the doorway"),
    THROUGH_DOORWAY: ("#00b894", "through the opening"),
    APPROACH: ("#9aa4b2", "walking between"),
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("plan", help="JSON from bs_synth --dump-plan")
    ap.add_argument("-o", "--out", default="capture_plan.svg")
    ap.add_argument("--px-per-m", type=float, default=68.0)
    ap.add_argument("--view-every", type=float, default=0.55,
                    help="metres between view-direction ticks")
    ap.add_argument("--view-len", type=float, default=0.62,
                    help="length of a view tick, metres")
    args = ap.parse_args()

    doc = json.loads(open(args.plan).read())
    rooms = doc["rooms"]
    div = doc["divider"]
    plan = doc["plan"]
    if not plan:
        print("empty plan", file=sys.stderr)
        return 1

    x0 = min(r[0] for r in rooms) - 0.6
    x1 = max(r[1] for r in rooms) + 0.6
    z0 = min(r[2] for r in rooms) - 0.6
    z1 = max(r[3] for r in rooms) + 1.5  # room for the legend
    s = args.px_per_m
    W, H = (x1 - x0) * s, (z1 - z0) * s

    def px(x, z):
        # +z is drawn downward, so the picture reads as a floor plan seen
        # from above with the world's +x to the right.
        return (x - x0) * s, (z - z0) * s

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W:.0f}" '
           f'height="{H:.0f}" viewBox="0 0 {W:.0f} {H:.0f}" '
           f'font-family="ui-sans-serif,system-ui,sans-serif">',
           f'<rect width="{W:.0f}" height="{H:.0f}" fill="#0f1216"/>']

    def rect(ax, bx, az, bz, **kw):
        px0, pz0 = px(ax, az)
        px1, pz1 = px(bx, bz)
        attrs = " ".join(f'{k.replace("_", "-")}="{v}"' for k, v in kw.items())
        out.append(f'<rect x="{px0:.1f}" y="{pz0:.1f}" '
                   f'width="{px1 - px0:.1f}" height="{pz1 - pz0:.1f}" {attrs}/>')

    # Rooms, then the divider as a solid slab with the opening cut out of it,
    # because the opening having depth is the point.
    for r in rooms:
        rect(r[0], r[1], r[2], r[3], fill="#161b22", stroke="#39414d",
             stroke_width=3)
    dh = div["door_half"]
    rect(div["x0"], div["x1"], rooms[0][2], -dh, fill="#39414d")
    rect(div["x0"], div["x1"], dh, rooms[0][3], fill="#39414d")
    for edge in (-dh, dh):  # the jambs: the surfaces the doorway orbit is for
        rect(div["x0"], div["x1"], edge - 0.045, edge + 0.045, fill="#e0409a")

    for b in doc["furniture"]:
        rect(b[0], b[1], b[2], b[3], fill="#222a35", stroke="#4b5563",
             stroke_width=2, rx=3)

    # The walk, one polyline per run of a single phase so colour changes land
    # exactly where the phase does.
    runs, cur = [], [plan[0]]
    for p in plan[1:]:
        if p[4] == cur[-1][4]:
            cur.append(p)
        else:
            cur.append(p)          # share the joining point
            runs.append(cur)
            cur = [p]
    runs.append(cur)
    for run in runs:
        colour = STYLE[run[0][4]][0]
        pts = " ".join("%.1f,%.1f" % px(p[0], p[1]) for p in run)
        width = 2.0 if run[0][4] == APPROACH else 3.4
        dash = ' stroke-dasharray="5 4"' if run[0][4] == APPROACH else ""
        out.append(f'<polyline points="{pts}" fill="none" stroke="{colour}" '
                   f'stroke-width="{width}" stroke-linecap="round" '
                   f'stroke-linejoin="round" opacity="0.95"{dash}/>')

    # Where the camera is actually pointed, which is the half of a capture
    # plan that a path drawing alone will not tell you.
    travelled, last = 0.0, plan[0]
    for p in plan:
        travelled += math.hypot(p[0] - last[0], p[1] - last[1])
        last = p
        if travelled < args.view_every:
            continue
        travelled = 0.0
        dx, dz = p[2] - p[0], p[3] - p[1]
        n = math.hypot(dx, dz)
        if n < 1e-6:
            continue
        ax, az = px(p[0], p[1])
        bx, bz = px(p[0] + args.view_len * dx / n,
                    p[1] + args.view_len * dz / n)
        out.append(f'<line x1="{ax:.1f}" y1="{az:.1f}" x2="{bx:.1f}" '
                   f'y2="{bz:.1f}" stroke="{STYLE[p[4]][0]}" '
                   f'stroke-width="1.1" opacity="0.5"/>')

    sx, sz = px(plan[0][0], plan[0][1])
    out.append(f'<circle cx="{sx:.1f}" cy="{sz:.1f}" r="6" fill="none" '
               f'stroke="#ffffff" stroke-width="2.5"/>')
    out.append(f'<text x="{sx + 11:.1f}" y="{sz + 4:.1f}" fill="#ffffff" '
               f'font-size="12">start / end</text>')

    length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                 for a, b in zip(plan, plan[1:]))
    out.append(f'<text x="14" y="24" fill="#e5e7eb" font-size="15" '
               f'font-weight="600">Capture walk — {length:.0f} m</text>')
    for i, phase in enumerate(
            (LAP, ORBIT_OBJECT, ORBIT_DOORWAY, THROUGH_DOORWAY, APPROACH)):
        colour, label = STYLE[phase]
        walked = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                     for a, b in zip(plan, plan[1:]) if b[4] == phase)
        y = H - 46 + 17 * (i % 3)
        x = 16 + 235 * (i // 3)
        out.append(f'<line x1="{x}" y1="{y - 4}" x2="{x + 22}" y2="{y - 4}" '
                   f'stroke="{colour}" stroke-width="3.4"/>')
        out.append(f'<text x="{x + 30}" y="{y}" fill="#9aa4b2" font-size="12">'
                   f'{label} — {walked:.0f} m</text>')

    out.append("</svg>")
    with open(args.out, "w") as f:
        f.write("\n".join(out))
    print(f"{args.out}: {length:.1f} m, {len(plan)} samples")
    return 0


if __name__ == "__main__":
    sys.exit(main())
