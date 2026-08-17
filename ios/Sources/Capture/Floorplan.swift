import Foundation
import simd

/// A 2D layout of the scout route, in metres, plus rooms the user will scan.
///
/// Built from live poses after the circuit: camera centres projected onto
/// the two axes the walk actually covered, dropping the "up" axis (the one
/// with the smallest spread). That is a floorplan even before the floor
/// itself has been measured.
struct Floorplan: Equatable {
    struct Pose: Equatable {
        var frameId: UInt32
        var world: SIMD3<Double>
        var plan: SIMD2<Double>
    }

    struct Room: Identifiable, Equatable {
        var id: UInt32
        var name: String
        /// Vertices in plan (metre) coordinates, CCW or CW.
        var polygon: [SIMD2<Double>]
        var scoutFrameIds: [UInt32]
        /// Extra photos stored while this room was the active capture.
        var captureCount: UInt32
        var scanned: Bool { captureCount > 0 }
    }

    var poses: [Pose]
    var rooms: [Room]
    /// World-space origin of the plan, and which two axes are the floor.
    var origin: SIMD3<Double>
    var axisU: SIMD3<Double>
    var axisV: SIMD3<Double>

    var path: [SIMD2<Double>] { poses.map(\.plan) }

    var bounds: (min: SIMD2<Double>, max: SIMD2<Double>) {
        guard let first = poses.first?.plan else {
            return (.zero, SIMD2(1, 1))
        }
        var lo = first, hi = first
        for p in poses {
            lo = simd_min(lo, p.plan)
            hi = simd_max(hi, p.plan)
        }
        for room in rooms {
            for v in room.polygon {
                lo = simd_min(lo, v)
                hi = simd_max(hi, v)
            }
        }
        let pad = 0.6
        return (lo - SIMD2(repeating: pad), hi + SIMD2(repeating: pad))
    }

    func toPlan(_ world: SIMD3<Double>) -> SIMD2<Double> {
        let d = world - origin
        return SIMD2(simd_dot(d, axisU), simd_dot(d, axisV))
    }
}

enum FloorplanBuilder {
    /// One line of `live/poses_scout.jsonl`.
    private struct PoseLine: Decodable {
        var frame_id: UInt32
        var state: String?
        var q: [Double]?
        var p: [Double]?
    }

    static func load(sessionDirectory: URL,
                     snapshot: CoreEngine.Snapshot) -> Floorplan {
        let posesURL = sessionDirectory
            .appendingPathComponent("live/poses_scout.jsonl")
        var worldCentres: [(UInt32, SIMD3<Double>)] = []
        if let text = try? String(contentsOf: posesURL, encoding: .utf8) {
            let decoder = JSONDecoder()
            for line in text.split(separator: "\n") where !line.isEmpty {
                guard let data = line.data(using: .utf8),
                      let row = try? decoder.decode(PoseLine.self, from: data),
                      row.state != "lost",
                      let q = row.q, q.count == 4,
                      let p = row.p, p.count == 3
                else { continue }
                let quat = simd_quatd(ix: q[1], iy: q[2], iz: q[3], r: q[0])
                let t = SIMD3(p[0], p[1], p[2])
                let center = -(quat.inverse.act(t))
                worldCentres.append((row.frame_id, center))
            }
        }
        if worldCentres.isEmpty {
            // Snapshot cameras have no frame ids; still enough to draw a path.
            for (i, cam) in snapshot.cameras.enumerated() {
                let quat = simd_quatd(ix: Double(cam.q.y), iy: Double(cam.q.z),
                                      iz: Double(cam.q.w), r: Double(cam.q.x))
                let t = SIMD3(Double(cam.t.x), Double(cam.t.y), Double(cam.t.z))
                worldCentres.append((UInt32(i + 1), -(quat.inverse.act(t))))
            }
        }

        let (origin, axisU, axisV) = basis(from: worldCentres.map(\.1))
        let poses: [Floorplan.Pose] = worldCentres.map { id, world in
            let d = world - origin
            return Floorplan.Pose(
                frameId: id, world: world,
                plan: SIMD2(simd_dot(d, axisU), simd_dot(d, axisV)))
        }

        var plan = Floorplan(poses: poses, rooms: [], origin: origin,
                             axisU: axisU, axisV: axisV)
        plan.rooms = autoRooms(in: plan, snapshot: snapshot)
        return plan
    }

    /// Drop the axis the walk barely moved on (height) and keep the other two
    /// as the plan. A house walked on one floor is then a top-down drawing.
    private static func basis(from centres: [SIMD3<Double>])
        -> (SIMD3<Double>, SIMD3<Double>, SIMD3<Double>) {
        guard !centres.isEmpty else {
            return (.zero, SIMD3(1, 0, 0), SIMD3(0, 0, 1))
        }
        var mean = SIMD3<Double>.zero
        for c in centres { mean += c }
        mean /= Double(centres.count)
        var varX = 0.0, varY = 0.0, varZ = 0.0
        for c in centres {
            let d = c - mean
            varX += d.x * d.x
            varY += d.y * d.y
            varZ += d.z * d.z
        }
        // Smallest spread is up. Remaining axes, in world order, become U,V.
        if varY <= varX && varY <= varZ {
            return (mean, SIMD3(1, 0, 0), SIMD3(0, 0, 1))
        }
        if varX <= varY && varX <= varZ {
            return (mean, SIMD3(0, 0, 1), SIMD3(0, 1, 0))
        }
        return (mean, SIMD3(1, 0, 0), SIMD3(0, 1, 0))
    }

    /// Prefer engine regions (covisibility clusters) when the scout mapped
    /// more than one; otherwise split the path where it thins — doorways.
    static func autoRooms(in plan: Floorplan,
                          snapshot: CoreEngine.Snapshot) -> [Floorplan.Room] {
        if snapshot.regions.count >= 2, !snapshot.patches.isEmpty {
            return roomsFromRegions(plan: plan, snapshot: snapshot)
        }
        return roomsFromPath(plan)
    }

    private static func roomsFromRegions(plan: Floorplan,
                                         snapshot: CoreEngine.Snapshot)
        -> [Floorplan.Room] {
        // Patches are not tagged with a region id in the snapshot. Assign
        // each to the nearest region by using weak-area region ids when
        // present, otherwise cluster patches by proximity to path density.
        // Practical fallback: one convex hull of path points nearest each
        // region's weak areas; if none, split the path evenly.
        var byRegion: [UInt32: [SIMD2<Double>]] = [:]
        for area in snapshot.weakAreas {
            byRegion[area.regionID, default: []].append(plan.toPlan(SIMD3(
                Double(area.center.x), Double(area.center.y),
                Double(area.center.z))))
        }
        if byRegion.isEmpty {
            for patch in snapshot.patches {
                let p = plan.toPlan(SIMD3(Double(patch.center.x),
                                          Double(patch.center.y),
                                          Double(patch.center.z)))
                // Nearest path pose's... just dump into one bucket per region
                // by hashing position into region count.
                guard !snapshot.regions.isEmpty else { continue }
                let i = abs(Int(p.x * 10) + Int(p.y * 10))
                    % snapshot.regions.count
                let id = snapshot.regions[i].id
                byRegion[id, default: []].append(p)
            }
        }
        var rooms: [Floorplan.Room] = []
        for region in snapshot.regions {
            var pts = byRegion[region.id] ?? []
            if pts.count < 3 {
                pts.append(contentsOf: plan.path)
            }
            let hull = convexHull(pts)
            guard hull.count >= 3 else { continue }
            let padded = pad(hull, by: 0.45)
            var room = Floorplan.Room(
                id: region.id, name: region.name, polygon: padded,
                scoutFrameIds: [], captureCount: 0)
            room.scoutFrameIds = plan.poses.filter {
                pointInPolygon($0.plan, padded)
            }.map(\.frameId)
            rooms.append(room)
        }
        return rooms.isEmpty ? roomsFromPath(plan) : rooms
    }

    /// Occupancy grid of the walk; thin corridors between fat blobs are
    /// treated as doorways and the blobs become rooms.
    private static func roomsFromPath(_ plan: Floorplan) -> [Floorplan.Room] {
        let pts = plan.path
        guard pts.count >= 8 else {
            if pts.count >= 3 {
                let hull = pad(convexHull(pts), by: 0.5)
                let ids = plan.poses.map(\.frameId)
                return [Floorplan.Room(id: 1, name: "Room 1", polygon: hull,
                                       scoutFrameIds: ids, captureCount: 0)]
            }
            return []
        }
        let cell: Double = 0.55
        let b = plan.bounds
        let w = max(1, Int(ceil((b.max.x - b.min.x) / cell)))
        let h = max(1, Int(ceil((b.max.y - b.min.y) / cell)))
        var occ = Array(repeating: Array(repeating: 0, count: w), count: h)
        func cellOf(_ p: SIMD2<Double>) -> (Int, Int) {
            let x = min(w - 1, max(0, Int((p.x - b.min.x) / cell)))
            let y = min(h - 1, max(0, Int((p.y - b.min.y) / cell)))
            return (x, y)
        }
        for p in pts {
            let (x, y) = cellOf(p)
            occ[y][x] += 1
        }
        // Dilate then erode: fills holes inside rooms, keeps doorways thin.
        occ = dilate(occ)
        occ = erode(occ)
        occ = erode(occ)
        let labels = connectedComponents(occ)
        var buckets: [Int: [SIMD2<Double>]] = [:]
        for pose in plan.poses {
            let (x, y) = cellOf(pose.plan)
            let lab = labels[y][x]
            if lab > 0 { buckets[lab, default: []].append(pose.plan) }
        }
        if buckets.isEmpty {
            let hull = pad(convexHull(pts), by: 0.5)
            return [Floorplan.Room(id: 1, name: "Room 1", polygon: hull,
                                   scoutFrameIds: plan.poses.map(\.frameId),
                                   captureCount: 0)]
        }
        var rooms: [Floorplan.Room] = []
        var nextId: UInt32 = 1
        for lab in buckets.keys.sorted() {
            guard let cloud = buckets[lab], cloud.count >= 3 else { continue }
            let hull = pad(convexHull(cloud), by: 0.4)
            guard hull.count >= 3 else { continue }
            var room = Floorplan.Room(
                id: nextId, name: "Room \(nextId)", polygon: hull,
                scoutFrameIds: [], captureCount: 0)
            room.scoutFrameIds = plan.poses.filter {
                pointInPolygon($0.plan, hull)
            }.map(\.frameId)
            rooms.append(room)
            nextId += 1
        }
        return rooms
    }

    static func assignScoutFrames(_ plan: inout Floorplan) {
        for i in plan.rooms.indices {
            let poly = plan.rooms[i].polygon
            plan.rooms[i].scoutFrameIds = plan.poses.filter {
                pointInPolygon($0.plan, poly)
            }.map(\.frameId)
        }
    }

    static func pointInPolygon(_ p: SIMD2<Double>,
                               _ poly: [SIMD2<Double>]) -> Bool {
        guard poly.count >= 3 else { return false }
        var inside = false
        var j = poly.count - 1
        for i in 0..<poly.count {
            let a = poly[i], b = poly[j]
            let intersect = ((a.y > p.y) != (b.y > p.y))
                && (p.x < (b.x - a.x) * (p.y - a.y) / max(1e-9, b.y - a.y) + a.x)
            if intersect { inside.toggle() }
            j = i
        }
        return inside
    }

    static func convexHull(_ pts: [SIMD2<Double>]) -> [SIMD2<Double>] {
        let unique = pts
        guard unique.count >= 3 else { return unique }
        let sorted = unique.sorted {
            $0.x == $1.x ? $0.y < $1.y : $0.x < $1.x
        }
        func cross(_ o: SIMD2<Double>, _ a: SIMD2<Double>,
                   _ b: SIMD2<Double>) -> Double {
            (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
        }
        var lower: [SIMD2<Double>] = []
        for p in sorted {
            while lower.count >= 2,
                  cross(lower[lower.count - 2], lower[lower.count - 1], p) <= 0 {
                lower.removeLast()
            }
            lower.append(p)
        }
        var upper: [SIMD2<Double>] = []
        for p in sorted.reversed() {
            while upper.count >= 2,
                  cross(upper[upper.count - 2], upper[upper.count - 1], p) <= 0 {
                upper.removeLast()
            }
            upper.append(p)
        }
        lower.removeLast()
        upper.removeLast()
        return lower + upper
    }

    private static func pad(_ hull: [SIMD2<Double>],
                            by m: Double) -> [SIMD2<Double>] {
        guard hull.count >= 3 else { return hull }
        var c = SIMD2<Double>.zero
        for p in hull { c += p }
        c /= Double(hull.count)
        return hull.map { p in
            let d = p - c
            let n = simd_length(d)
            return n < 1e-6 ? p : c + d * ((n + m) / n)
        }
    }

    private static func dilate(_ g: [[Int]]) -> [[Int]] {
        morph(g, keep: true)
    }
    private static func erode(_ g: [[Int]]) -> [[Int]] {
        morph(g, keep: false)
    }
    private static func morph(_ g: [[Int]], keep: Bool) -> [[Int]] {
        let h = g.count
        guard h > 0 else { return g }
        let w = g[0].count
        var o = g
        for y in 0..<h {
            for x in 0..<w {
                var any = g[y][x] > 0
                var all = g[y][x] > 0
                for dy in -1...1 {
                    for dx in -1...1 {
                        let ny = y + dy, nx = x + dx
                        guard ny >= 0, ny < h, nx >= 0, nx < w else {
                            all = false
                            continue
                        }
                        if g[ny][nx] > 0 { any = true } else { all = false }
                    }
                }
                o[y][x] = (keep ? any : all) ? 1 : 0
            }
        }
        return o
    }

    private static func connectedComponents(_ g: [[Int]]) -> [[Int]] {
        let h = g.count
        guard h > 0 else { return g }
        let w = g[0].count
        var lab = Array(repeating: Array(repeating: 0, count: w), count: h)
        var cur = 0
        for y in 0..<h {
            for x in 0..<w where g[y][x] > 0 && lab[y][x] == 0 {
                cur += 1
                var stack = [(x, y)]
                lab[y][x] = cur
                while let (cx, cy) = stack.popLast() {
                    for (nx, ny) in [(cx - 1, cy), (cx + 1, cy),
                                     (cx, cy - 1), (cx, cy + 1)] {
                        guard ny >= 0, ny < h, nx >= 0, nx < w,
                              g[ny][nx] > 0, lab[ny][nx] == 0 else { continue }
                        lab[ny][nx] = cur
                        stack.append((nx, ny))
                    }
                }
            }
        }
        return lab
    }
}
