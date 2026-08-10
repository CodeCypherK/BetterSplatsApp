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
├── live/                 LIVE  engine-owned, disposable
│   └── poses.jsonl       one JSON object per processed frame (live pose);
│                         the final solve's only live input (init hint)
└── final/                FINAL engine-owned, rebuilt from RAW at will
    ├── cache/            resume cache — safe to delete, keyed by manifest
    │   ├── manifest.txt  config+frame-list hash; mismatch wipes the cache
    │   ├── feat_NNNNNN.bin   per-frame features (keypoints, descriptors,
    │   │                     undistorted coords, colors, gradients)
    │   └── matches.bin   verified pair matches for the whole session
    ├── report.json       readiness + solve quality report
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
| 6  | u16 | flags — bit0: payload is LZ4 block-compressed |
| 8  | u16 | width (e.g. 320) |
| 10 | u16 | height (e.g. 240) |
| 12 | u16 | dtype — 0 = float16 meters |
| 14 | u16 | reserved = 0 |
| 16 | u32 | payload_bytes (compressed size when bit0 set) |
| 20 | u32 | crc32 of the payload bytes as stored |
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
  "store_reason": "kf"              // "gate" | "kf" | "burst"
}
```

Per-frame intrinsics are recorded even though AF is locked (OIS and focus
breathing shift them slightly); the reconstruction uses the session camera
from `calibration.json` unless per-frame drift exceeds its fit residual.

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
  "capture": { "af_locked": true, "gdc_disabled": true, "stabilization": "off" },
  "frame_count": 412,                          // stored frames
  "keyframe_ids": [1, 9, 15, ...],
  "regions": [ { "id": 1, "name": "Room 1", "renamed": false } ],
  "app_version": "0.1.0"
}
```

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
