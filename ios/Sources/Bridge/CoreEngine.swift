import Foundation

/// Thin Swift facade over the C engine ABI (`bs_api.h`). One instance per
/// process; all heavy work happens on engine-owned threads behind the ABI.
final class CoreEngine: @unchecked Sendable {
    static let shared = CoreEngine()

    private let handle: OpaquePointer?

    private init() {
        handle = bs_create("{}")
    }

    deinit {
        bs_destroy(handle)
    }

    var isAvailable: Bool { handle != nil }

    static var version: String {
        String(cString: bs_version())
    }

    var lastError: String {
        String(cString: bs_last_error(handle))
    }

    /// Runs tiny known-answer problems through Eigen, OpenCV and Ceres on
    /// this device. Surfaced on the diagnostics screen so a side-loaded
    /// build can prove its dependency chain in the field.
    func selftest() -> (ok: Bool, report: String) {
        var buf = [CChar](repeating: 0, count: 512)
        let result = bs_selftest(&buf, buf.count)
        return (result == BS_OK, String(cString: buf))
    }

    // MARK: - Floor calibration

    /// One measurement of the surface the phone is pointed at, in that
    /// frame's camera coordinates.
    struct FloorPlane {
        var isUsable: Bool          // verdict == good
        var advice: String          // what to tell the user to do next
        var normal: (Double, Double, Double)
        var heightM: Double         // camera's height above the surface
        var rmseM: Double
        var incidenceDeg: Double
        var inliers: Int32
    }

    /// Fits the dominant plane in one depth map. Stateless — no engine
    /// session required, so this runs while the user is still lining the
    /// shot up. The decision about whether it is a usable floor is made in
    /// the engine, not here: the thresholds are claims about how a phone is
    /// held and how flat a floor is, and they belong with the geometry.
    static func fitFloorPlane(depth: UnsafePointer<Float>, width: Int32,
                              height: Int32, fx: Double, fy: Double,
                              cx: Double, cy: Double) -> FloorPlane? {
        var out = bs_floor_plane()
        guard bs_fit_floor_plane(depth, width, height, fx, fy, cx, cy, &out)
                == BS_OK else { return nil }
        return FloorPlane(
            isUsable: out.verdict == Int32(BS_FLOOR_GOOD.rawValue),
            advice: out.advice.map { String(cString: $0) } ?? "Point at the floor",
            normal: (out.normal.0, out.normal.1, out.normal.2),
            heightM: out.height_m,
            rmseM: out.rmse_m,
            incidenceDeg: out.incidence_deg,
            inliers: out.inliers)
    }

    // MARK: - Live session

    @discardableResult
    /// `pass` selects the scout circuit (build a localization scaffold) or
    /// ordinary capture (localize into the scaffold, when one exists).
    func liveBegin(sessionDir: String, pass: bs_pass_kind = BS_PASS_CAPTURE)
        -> bs_result {
        bs_live_begin(handle, sessionDir, pass)
    }

    /// Feeds one frame. `frame` must be fully populated with pointers that
    /// stay valid for the duration of the call (the engine copies).
    @discardableResult
    func liveFeed(_ frame: inout bs_frame_in) -> bs_result {
        bs_live_feed(handle, &frame)
    }

    func livePollStatus() -> bs_live_status {
        var status = bs_live_status()
        _ = bs_live_poll_status(handle, &status)
        return status
    }

    @discardableResult
    func liveEnd() -> bs_result {
        bs_live_end(handle)
    }

    @discardableResult
    func renameRegion(id: UInt32, name: String) -> bs_result {
        bs_region_rename(handle, id, name)
    }

    // MARK: - Final solve

    @discardableResult
    func finalStart(sessionDir: String, preset: String) -> bs_result {
        bs_final_start(handle, sessionDir, preset)
    }

    func finalPoll() -> bs_final_progress {
        var progress = bs_final_progress()
        _ = bs_final_poll(handle, &progress)
        return progress
    }

    @discardableResult
    func finalCancel() -> bs_result {
        bs_final_cancel(handle)
    }

    @discardableResult
    func thermalHint(level: Int32) -> bs_result {
        bs_thermal_hint(handle, level)
    }

    // MARK: - Snapshot

    struct Snapshot {
        struct Region {
            let id: UInt32
            let name: String
            let score: Float
            let sub: [Float]  // geometry, pose, texture, lidar, view
            let areaM2: Float
            let patchCount: UInt32
            let weakAreaCount: UInt32
            let worstDeficiency: Int  // axis of the worst weak spot, -1 none
        }
        struct WeakArea {
            let center: SIMD3<Float>
            let radiusM: Float
            let regionID: UInt32
            let deficiency: Int  // 0 geom, 1 pose, 2 texture, 3 lidar, 4 view
            let surfaceKind: Int // 0 wall, 1 floor, 2 ceiling, 3 object
            let surfaceSide: Int // wall: 0 unknown, 1 back, 2 left, 3 right, 4 front
            let moveDir: SIMD3<Float>
            let moveDistM: Float
            let score: Float
        }

        var points: [SIMD4<Float>] = []      // xyz + packed rgba in w
        var pointColors: [SIMD4<Float>] = []
        var cameras: [(q: SIMD4<Float>, t: SIMD3<Float>)] = []
        var patches: [(center: SIMD3<Float>, extent: Float, score: Float)] = []
        var regions: [Region] = []
        var weakAreas: [WeakArea] = []
        var boundsMin = SIMD3<Float>(0, 0, 0)
        var boundsMax = SIMD3<Float>(0, 0, 0)
    }

    /// Copies the current reconstruction snapshot out of the engine.
    func snapshot() -> Snapshot {
        var raw = bs_snapshot()
        guard bs_snapshot_acquire(handle, &raw) == BS_OK else { return Snapshot() }
        defer { bs_snapshot_release(handle, &raw) }

        var snap = Snapshot()
        if let points = raw.points {
            snap.points.reserveCapacity(Int(raw.point_count))
            snap.pointColors.reserveCapacity(Int(raw.point_count))
            for i in 0..<Int(raw.point_count) {
                let p = points[i]
                snap.points.append(SIMD4(p.x, p.y, p.z, 1))
                let dim: Float = (p.flags & 1) != 0 ? 0.45 : 1.0
                snap.pointColors.append(SIMD4(
                    Float(p.r) / 255 * dim, Float(p.g) / 255 * dim,
                    Float(p.b) / 255 * dim, 1))
            }
        }
        if let cameras = raw.cameras {
            for i in 0..<Int(raw.camera_count) {
                let c = cameras[i]
                snap.cameras.append((
                    q: SIMD4(c.q.0, c.q.1, c.q.2, c.q.3),
                    t: SIMD3(c.t.0, c.t.1, c.t.2)))
            }
        }
        if let patches = raw.patches {
            for i in 0..<Int(raw.patch_count) {
                let p = patches[i]
                snap.patches.append((SIMD3(p.cx, p.cy, p.cz), p.extent, p.score))
            }
        }
        if let regions = raw.regions {
            for i in 0..<Int(raw.region_count) {
                var r = regions[i]
                let name = withUnsafeBytes(of: &r.name) { rawBuf -> String in
                    let data = rawBuf.prefix(while: { $0 != 0 })
                    return String(decoding: data, as: UTF8.self)
                }
                snap.regions.append(Snapshot.Region(
                    id: r.region_id, name: name, score: r.score,
                    sub: [r.sub.0, r.sub.1, r.sub.2, r.sub.3, r.sub.4],
                    areaM2: r.area_m2, patchCount: r.patch_count,
                    weakAreaCount: r.weak_area_count,
                    worstDeficiency: Int(r.worst_deficiency)))
            }
        }
        if let weak = raw.weak_areas {
            for i in 0..<Int(raw.weak_area_count) {
                let w = weak[i]
                snap.weakAreas.append(Snapshot.WeakArea(
                    center: SIMD3(w.cx, w.cy, w.cz), radiusM: w.radius_m,
                    regionID: w.region_id,
                    deficiency: Int(w.deficiency),
                    surfaceKind: Int(w.surface_kind),
                    surfaceSide: Int(w.surface_side),
                    moveDir: SIMD3(w.move_dir.0, w.move_dir.1, w.move_dir.2),
                    moveDistM: w.move_dist_m, score: w.score))
            }
        }
        snap.boundsMin = SIMD3(raw.bounds_min.0, raw.bounds_min.1, raw.bounds_min.2)
        snap.boundsMax = SIMD3(raw.bounds_max.0, raw.bounds_max.1, raw.bounds_max.2)
        return snap
    }
}
