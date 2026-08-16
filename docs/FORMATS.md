# On-disk formats (normative)

Schema version: **1**. The Swift `SessionWriter`, the C++ session reader, and
`bs_synth` all implement exactly this document. Changes require a schema
version bump and a migration note here.

## Session directory

```
session_<yyyyMMdd-HHmmss>_<6hex>/
├── session.json          RAW   written by the app (start + finalize)
├── calibration.json      RAW   per-session camera summary + fitted model
├── frames/
│   ├── 000001/
│   │   ├── image.jpg     RAW   1920×1440 JPEG (quality ≈0.85), no EXIF
│   │   ├── lidar.depth   RAW   binary depth map, format below
│   │   └── meta.json     RAW   per-frame metadata, schema below
│   └── ...
├── masks/                DERIVED  optional, written by an external tool
│   └── NNNNNN.png       8-bit exclusion mask for frame NNNNNN, format below
├── live/                 LIVE  engine-owned, disposable
│   ├── poses.jsonl       one JSON object per processed frame (live pose);
│   │                     the final solve's only live input (init hint)
│   ├── poses_scout.jsonl same, for the scout circuit — separate so the
│   │                     capture pass does not erase the scout's evidence
│   ├── map.bin           scout scaffold: the circuit's keyframes + points,
│   │                     loaded by this session's capture pass
│   ├── map_end.bin       map at the end of the capture pass, loaded by the
│   │                     NEXT capture in the project (see Project chains)
│   └── ate_*.csv         per-frame error vs ground truth (synth only)
└── final/                FINAL engine-owned, rebuilt from RAW at will
    ├── cache/            resume cache — safe to delete, keyed by manifest
    │   ├── manifest.txt  config+frame-list hash; mismatch wipes the cache
    │   ├── feat_NNNNNN.bin   per-frame features (keypoints, descriptors,
    │   │                     undistorted coords, colors, gradients)
    │   └── matches.bin   verified pair matches for the whole session
    ├── report.json       solve quality + per-image table and flags
    ├── transforms.json   nerfstudio/instant-ngp poses+intrinsics (gsplat-ready)
    ├── dense.ply         fused LiDAR cloud (final poses only)
    └── colmap/
        ├── cameras.txt
        ├── images.txt
        ├── points3D.txt
        └── images/       exact JPEGs referenced by images.txt
```

Frame directories are zero-padded six-digit, starting at `000001`,
monotonically increasing, gaps allowed (non-stored frames never existed on
disk). RAW files are write-once: any process that would modify one is a bug.

## `lidar.depth` binary layout

Little-endian, 24-byte header followed by payload:

| offset | type | field |
|---|---|---|
| 0  | char[4] | magic `"BSDP"` |
| 4  | u16 | version = 1 |
| 6  | u16 | flags — bit0: payload is LZ4 block-compressed; bit1: a confidence plane follows the depth plane inside the payload |
| 8  | u16 | width (e.g. 320) |
| 10 | u16 | height (e.g. 240) |
| 12 | u16 | dtype — 0 = float16 meters |
| 14 | u16 | reserved = 0 |
| 16 | u32 | payload_bytes (compressed size when bit0 set) |
| 20 | u32 | crc32 of the payload bytes as stored |

When flags bit1 is set the payload is **two planes**: the float16 depth plane
(w×h×2 bytes) immediately followed by a uint8 confidence plane (w×h bytes),
compressed together as one blob when bit0 is also set. Inside the payload
rather than appended after it, so `payload_bytes` and the CRC still cover
every byte and the exact-size check still holds — a reader that does not know
the flag fails loudly on the size instead of silently ignoring half the file.

Confidence is the sensor's own per-pixel certainty, 0-255. AVFoundation
exposes none, so frames captured through it carry no plane and the flag stays
clear; ARKit's `ARDepthData.confidenceMap` does. Absent means "no opinion",
never "zero confidence" — see ARCHITECTURE.md, "Why not ARKit".
| 24 | ... | row-major payload |

Invalid samples are NaN (preferred) or 0. Uncompressed size is always
`width * height * 2` bytes; readers must verify CRC and decompressed size.

## `meta.json` (per frame)

```jsonc
{
  "schema_version": 1,
  "frame_id": 42,                    // matches directory name
  "t_capture": 12345.678901,        // s, host clock, RGB mid-exposure
  "t_depth": 12345.676543,          // s, host clock, depth timestamp
  "intrinsics":       { "fx": 1456.1, "fy": 1456.1, "cx": 959.4, "cy": 719.8,
                        "ref_w": 1920, "ref_h": 1440 },   // for image.jpg pixels
  "depth_intrinsics": { "fx": 242.6, "fy": 242.6, "cx": 159.9, "cy": 119.9,
                        "ref_w": 320, "ref_h": 240 },     // for lidar.depth pixels
  "distortion_ref": "session",      // tables live in calibration.json
  "exposure": { "duration_s": 0.008, "iso": 200, "bias_ev": 0.0 },
  "quality":  { "lap_var": 312.5, "overexp_frac": 0.003 },
  "is_keyframe": true,
  "store_reason": "kf",             // "gate" | "kf" | "burst"
  "pass": "capture"                 // "capture" | "scout"; absent => capture
}
```

Per-frame intrinsics are recorded even though AF is locked (OIS and focus
breathing shift them slightly); the reconstruction uses the session camera
from `calibration.json` unless per-frame drift exceeds its fit residual.

### Passes

`pass` marks which capture pass produced the frame.

- **`scout`** — the optional opening circuit: one fast lap of the whole
  space, back to the walls, camera aimed across each room. It exists to
  build a localization scaffold that later passes hold position against, and
  to establish the session's world frame and room list up front. Those
  frames are walked fast, far from every surface, and cover each room only
  briefly, so **the final solve excludes them** (`final_include_scout`
  overrides, for study only) and they never appear in `colmap/images/`.
- **`capture`** — ordinary detail capture, and the default for any frame
  written before this field existed. These are the frames reconstructed and
  exported.

Scout frames are still written to RAW like any other: they are real
measurements, the Linux replay needs them to reproduce a session, and
"excluded from the reconstruction" is a solve-time decision, never a reason
to discard captured data.

## `masks/NNNNNN.png` (derived layer, optional)

An 8-bit single-channel PNG per frame, naming the pixels the final solve may
detect features on. **Non-zero keeps a pixel, zero excludes it** — the COLMAP
convention, and the same sense a splat trainer's photometric loss wants, so
one file serves both without anyone inverting it in between. That direction is
deliberate rather than arbitrary: an inverted mask does not fail, it
reconstructs the room from the people walking through it and reports a
perfectly healthy solve.

`NNNNNN` is the frame's zero-padded id, matching its directory under
`frames/`. Any resolution is accepted; the solve resizes to the JPEG's grid
with **nearest-neighbour** interpolation, because the value is binary and
interpolating one invents fractional pixels along every boundary.

Four properties are normative:

- **It is a fourth layer, not part of RAW.** Masks sit beside `frames/`, never
  inside it. They are regenerated whenever the segmentation model or the class
  list changes, and RAW's whole value is that a byte comparison after
  processing still passes.
- **Absent means "no opinion".** A session with no `masks/` directory, or a
  frame with no mask, behaves bit-identically to one from before this existed.
  Absent never means "exclude everything".
- **It can only remove evidence, never add it.** The worst case of a bad mask
  is a thinner reconstruction; there is no case where one causes a pixel to be
  trusted more than it would have been.
- **In a chain, a mask lives with the session that owns its frame.** The
  lookup resolves through the same owner map as `frames/`, so masks for a
  parent session's frames belong in the parent (see Project chains).

`final_use_masks` (default true) turns the whole mechanism off. Masks are
fingerprinted by size and modification time into the feature cache key, so
regenerating them invalidates cached features rather than silently reusing the
ones extracted before the masks existed.

Nothing in this repository writes masks. They are produced by whatever
segmentation tool the operator prefers — GSplat Studio's pre-splatting cleanup
generates them from a COCO-trained instance segmenter — which is exactly why
the convention has to be written down here rather than agreed informally.

## `calibration.json` (per session)

```jsonc
{
  "schema_version": 1,
  "reference": { "width": 1920, "height": 1440 },
  "intrinsics_session": { "fx": 1456.1, "fy": 1456.1, "cx": 959.4, "cy": 719.8 },
  "distortion_lut": {                // Apple radial lookup table, verbatim
    "magnification": [ /* float[] */ ],
    "inverse": [ /* float[] */ ],
    "center": [959.4, 719.8]
  },
  "colmap_model": {                  // fitted in M2; absent until then
    "model": "OPENCV",
    "params": [1456.1, 1456.1, 959.4, 719.8, 0.011, -0.020, 0.0, 0.0],
    "fit_residual_px_max": 0.12
  },
  "depth_to_rgb": { "identity": true } // extrinsic RGB<->depth mapping
}
```

## `session.json`

```jsonc
{
  "schema_version": 1,
  "session_id": "session_20260809-142230_a3f2c1",
  "created_utc": "2026-08-09T14:22:30Z",
  "end_utc": "2026-08-09T14:26:05Z",          // added at finalize
  "device": { "model": "iPhone16,1", "ios": "18.6" },
  "video": { "w": 1920, "h": 1440, "fps": 30, "pixel_format": "420f" },
  "depth": { "w": 320, "h": 240, "format": "hdep", "filtering": false },
  // Locks actually achieved this session (recorded at finalize, not intent).
  // ae/awb/iso_locked absent in early schema-v1 sessions -> read as false.
  // ISO is frozen after the opening AE settle; shutter duration may still
  // move (`ae_locked` false, `iso_locked` true) so a lighting change does
  // not clip. Per-frame duration and ISO are in each frame's meta.json.
  "capture": { "af_locked": true, "ae_locked": false, "iso_locked": true,
               "awb_locked": true, "gdc_disabled": true, "stabilization": "off" },
  "frame_count": 412,                          // stored frames
  "keyframe_ids": [1, 9, 15, ...],
  "regions": [ { "id": 1, "name": "Room 1", "renamed": false } ],

  // The floor as the depth sensor measured it while the user aimed at it,
  // in that FRAME'S CAMERA coordinates — never world. The solve derives the
  // world plane from that frame's final pose; a world plane written here
  // would bake in whatever the live tracker believed at capture time.
  // Absent when the user skipped the step or the fit never converged.
  "floor_calibration": {
    "frame_id": 7, "normal": [0.01, -0.999, 0.02], "offset_m": 1.496,
    "rmse_m": 0.011, "incidence_deg": 12.4, "inliers": 17204
  },

  // --- project (a space captured over several sittings) ---
  // Stable across the whole chain; groups captures into one reopenable
  // thing. Absent in captures written before projects existed, which read
  // as standalone.
  "project_id": "proj_9f2a41c7",
  "project_name": "Oak Street house",
  // The sibling capture this one continues, by DIRECTORY NAME, not path —
  // a path would not survive the project being zipped here and unzipped
  // somewhere else. Empty for the first capture in a project.
  // Frame ids continue across the chain rather than restarting, so ids are
  // unique project-wide; a duplicate makes the reader refuse the chain.
  "parent_session": "session_20260809-141002_77bd0e",
  // World volumes this capture RE-COVERED, superseding what earlier
  // captures in the project recorded there. Axis-aligned, in the project's
  // shared world frame. Only ever applies to captures EARLIER in the chain.
  // The superseded frames are NOT deleted or modified — the final solve
  // declines to reconstruct from them, so removing a volume restores them.
  "supersedes": [
    { "min": [-2.1, -0.5, -3.0], "max": [4.4, 2.6, 5.2], "label": "Kitchen" }
  ],
  "app_version": "0.1.0"
}
```

### Project chains

A space bigger than one capture is a **chain**: sessions sharing a
`project_id`, linked oldest-to-newest by `parent_session`, all in one world
frame. The reader resolves a chain transparently — `frame_ids()` spans it and
every accessor resolves an id to whichever session holds it — so the final
solve reconstructs across a chain without knowing chains exist.

Rules the reader enforces, each guarding a silent corruption rather than a
crash:

| condition | behaviour |
|---|---|
| duplicate frame id across two sessions | chain refused to open |
| `parent_session` cycles or self-references | walk terminates, bounded |
| `parent_session` names a missing session | reconstructs what is present, warns |
| inverted/degenerate `supersedes` box | dropped at parse |

A capture pass also writes `live/map_end.bin`, which the NEXT capture in the
chain loads to inherit the world frame. It is deliberately not `map.bin`
(the scout scaffold): overwriting that would make a re-run of a capture pass
start from the previous run's result, so the same session would replay
differently every time.

`bs_synth` writes the same schema with `device.model = "synthetic"` plus an
extra RAW-adjacent `ground_truth/` directory (poses + points) that only tests
read.

## `live/poses.jsonl`

One line per processed frame:

```jsonc
{"frame_id": 42, "t": 12345.678901, "state": "tracking",
 "q": [0.99, 0.01, -0.02, 0.03], "p": [0.15, -0.02, 0.94]}
```

`q` (w,x,y,z) and `p` are **world-to-camera** (COLMAP convention:
`x_cam = R(q)·x_world + p`). Frames processed while tracking was lost carry
`"state": "lost"` and no pose.

## COLMAP export conventions

- `cameras.txt`: single camera, `OPENCV` model
  (`fx fy cx cy k1 k2 p1 p2`), or `PINHOLE` when the export falls back to
  undistorted images.
- `images.txt`: `IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME`, pose is
  world-to-camera; second line holds `X Y POINT3D_ID` triples with `-1` for
  features without a 3D point.
- `points3D.txt`: `POINT3D_ID X Y Z R G B ERROR (IMAGE_ID POINT2D_IDX)*`;
  every referenced observation exists in `images.txt` and vice versa.
- `images/` contains exactly the JPEGs named in `images.txt` (hard-linked or
  cloned from RAW, never re-encoded).
- Only feature-tracked points appear in `points3D.txt`; fused LiDAR geometry
  ships separately as `dense.ply`.

## `transforms.json` (nerfstudio / instant-ngp)

A convenience mirror of the COLMAP poses for Gaussian-splat trainers that
read the NeRF format directly. Top level carries the shared intrinsics
(`fl_x fl_y cx cy w h k1 k2 p1 p2`, `camera_model: "OPENCV"`); `frames[]`
holds one `{ file_path: "images/<name>", transform_matrix: 4×4 }` per
registered image. Each `transform_matrix` is **camera-to-world** in the
OpenGL/NeRF convention (camera looks down −Z, +Y up) — the COLMAP
world-to-camera pose inverted with its Y and Z axes negated. Derived from the
same solve as `colmap/`, so the two always agree (CI cross-checks camera
centres to < 1 mm).
