import AVFoundation
import CoreImage
import Foundation

/// RAW-layer writer: owns a session directory under Documents and writes the
/// immutable per-frame files (image.jpg, lidar.depth, meta.json) plus
/// session.json / calibration.json, exactly per docs/FORMATS.md.
///
/// Actor isolation gives a single writer with an implicit queue; frames are
/// appended in submission order and never overwritten (RAW is write-once —
/// the engine only ever reads frames/ and owns live/ + final/ instead).
actor SessionStore {
    struct FramePayload {
        var frameId: UInt32
        var jpeg: Data
        var depthF16: [UInt16]
        var depthWidth: Int
        var depthHeight: Int
        var meta: FrameMetaJSON
    }

    enum StoreError: Error {
        case sessionExists
        case ioFailure(String)
        case depthEncodeFailed
    }

    let directory: URL
    private var sessionDoc: SessionJSON
    private var frameCount: UInt32 = 0
    /// Where this capture starts numbering; see init.
    private(set) var firstFrameId: UInt32 = 1
    private var bytesWritten: Int64 = 0
    private var calibrationWritten = false
    private let encoder: JSONEncoder

    static func documentsDirectory() -> URL {
        FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
    }

    static func makeProjectId() -> String {
        String(format: "proj_%08x", UInt32.random(in: 0...UInt32.max))
    }

    static func makeSessionId(date: Date = Date()) -> String {
        let fmt = DateFormatter()
        fmt.dateFormat = "yyyyMMdd-HHmmss"
        fmt.timeZone = TimeZone(identifier: "UTC")
        let suffix = String(format: "%06x", UInt32.random(in: 0...0xFFFFFF))
        return "session_\(fmt.string(from: date))_\(suffix)"
    }

    /// `continuing` links this capture to an existing project: it inherits
    /// the project's identity and names its predecessor, so the engine loads
    /// that capture's map and both share one world frame.
    /// `osVersion` is passed in rather than read here: UIDevice is
    /// MainActor-isolated and an actor's init is not, which is a warning
    /// today and an error under Swift 6.
    init(videoW: Int, videoH: Int, depthW: Int, depthH: Int, fps: Int,
         osVersion: String,
         continuing: ProjectStore.Project? = nil,
         newProjectName: String? = nil) throws {
        let id = Self.makeSessionId()
        directory = Self.documentsDirectory().appendingPathComponent(id)

        let fm = FileManager.default
        if fm.fileExists(atPath: directory.path) {
            throw StoreError.sessionExists
        }
        try fm.createDirectory(
            at: directory.appendingPathComponent("frames"),
            withIntermediateDirectories: true)

        encoder = JSONEncoder()
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]

        let iso = ISO8601DateFormatter()
        sessionDoc = SessionJSON(
            sessionId: id,
            createdUtc: iso.string(from: Date()),
            endUtc: nil,
            device: .init(
                model: Self.deviceModelIdentifier(),
                ios: osVersion),
            video: .init(w: videoW, h: videoH, fps: fps, pixelFormat: "420f"),
            depth: .init(w: depthW, h: depthH, format: "hdep", filtering: false),
            // Lock flags are provisional here — the real state is recorded at
            // finalize(), after the settle window has actually locked them.
            capture: .init(afLocked: false, aeLocked: false, awbLocked: false,
                           gdcDisabled: true, stabilization: "off"),
            frameCount: 0,
            keyframeIds: [],
            // Empty, not a seeded "Room 1". Regions are discovered by the
            // engine from the covisibility graph; this list records the names
            // the USER gave them, and claiming one exists before anything has
            // been mapped is a small lie in a file that is meant to be the
            // record of what happened.
            regions: [],
            projectId: continuing?.id ?? Self.makeProjectId(),
            projectName: continuing?.name ?? (newProjectName ?? "Untitled"),
            parentSession: continuing?.latestSessionName ?? "",
            supersedes: [],
            appVersion: Bundle.main.infoDictionary?["CFBundleShortVersionString"]
                as? String ?? "0")
        // Frame ids continue across the whole project rather than restarting.
        // Two captures both numbering from 1 would collide, and the reader
        // rejects a chain with duplicate ids outright — every downstream id,
        // from COLMAP image ids to the exported filenames, would otherwise
        // describe the wrong picture.
        firstFrameId = (continuing?.lastFrameId ?? 0) + 1
        try Self.writeSessionDoc(sessionDoc, encoder: encoder,
                                 to: directory)
    }

    /// Records the session camera model from the first frame's calibration:
    /// intrinsics at reference dimensions plus Apple's radial distortion LUT.
    func writeCalibrationIfNeeded(_ calibration: AVCameraCalibrationData) throws {
        guard !calibrationWritten else { return }

        let m = calibration.intrinsicMatrix  // simd_float3x3, column-major
        let ref = calibration.intrinsicMatrixReferenceDimensions

        func floats(_ data: Data?) -> [Float] {
            guard let data else { return [] }
            return data.withUnsafeBytes { raw in
                Array(raw.bindMemory(to: Float32.self))
            }
        }

        let doc = CalibrationJSON(
            reference: .init(width: Int(ref.width), height: Int(ref.height)),
            intrinsicsSession: PinholeIntrinsicsJSON(
                fx: Double(m.columns.0.x), fy: Double(m.columns.1.y),
                cx: Double(m.columns.2.x), cy: Double(m.columns.2.y),
                refW: Int(ref.width), refH: Int(ref.height)),
            distortionLut: DistortionLutJSON(
                magnification: floats(calibration.lensDistortionLookupTable),
                inverse: floats(calibration.inverseLensDistortionLookupTable),
                center: [
                    Double(calibration.lensDistortionCenter.x),
                    Double(calibration.lensDistortionCenter.y),
                ]))
        let data = try encoder.encode(doc)
        try Self.write(data, to: directory.appendingPathComponent("calibration.json"))
        // Only now. Setting this first meant a failed write was never retried,
        // and a session with no calibration.json is one the reader rejects
        // outright — the whole capture lost to one transient error.
        calibrationWritten = true
    }

    /// Appends one immutable frame. Fails rather than overwrites.
    ///
    /// `meta.json` is written LAST and is the completion marker: readers
    /// enumerate `frames/` on disk rather than trusting `frame_count`, so a
    /// directory without it is a frame that never finished. On failure the
    /// partial directory is removed here — that is not a violation of RAW
    /// immutability, which protects frames that were written, not the debris
    /// of one that was not. A crash between the first write and the last
    /// leaves the same debris with nothing running to clear it, so the reader
    /// skips incomplete directories too.
    func writeFrame(_ payload: FramePayload) throws {
        let frameDir = directory
            .appendingPathComponent("frames")
            .appendingPathComponent(String(format: "%06u", payload.frameId))
        let fm = FileManager.default
        guard !fm.fileExists(atPath: frameDir.path) else {
            throw StoreError.ioFailure("frame \(payload.frameId) already exists")
        }
        try fm.createDirectory(at: frameDir, withIntermediateDirectories: true)
        var complete = false
        defer { if !complete { try? fm.removeItem(at: frameDir) } }

        try Self.write(payload.jpeg, to: frameDir.appendingPathComponent("image.jpg"))

        var encodedLen = 0
        let encoded = payload.depthF16.withUnsafeBufferPointer { buf in
            bs_depth_encode(
                buf.baseAddress, Int32(payload.depthWidth),
                Int32(payload.depthHeight), &encodedLen)
        }
        guard let encoded else { throw StoreError.depthEncodeFailed }
        defer { bs_buffer_release(encoded) }
        try Self.write(
            Data(bytes: encoded, count: encodedLen),
            to: frameDir.appendingPathComponent("lidar.depth"))

        let metaData = try encoder.encode(payload.meta)
        try Self.write(metaData, to: frameDir.appendingPathComponent("meta.json"))
        complete = true

        frameCount += 1
        bytesWritten += Int64(payload.jpeg.count + encodedLen + metaData.count)
        if payload.meta.isKeyframe {
            sessionDoc.keyframeIds.append(payload.frameId)
        }
    }

    /// `locks`: what the camera actually froze for this session (focus,
    /// exposure, white balance) — recorded honestly, including partial or
    /// failed locks, so the reconstruction side knows what to assume.
    func finalize(locks: (focus: Bool, exposure: Bool, iso: Bool,
                          whiteBalance: Bool) = (false, false, false, false)) throws {
        sessionDoc.endUtc = ISO8601DateFormatter().string(from: Date())
        sessionDoc.frameCount = frameCount
        sessionDoc.capture.afLocked = locks.focus
        sessionDoc.capture.aeLocked = locks.exposure
        sessionDoc.capture.isoLocked = locks.iso
        sessionDoc.capture.awbLocked = locks.whiteBalance
        try writeSessionJson()
    }

    /// Records the floor the user measured, against the frame it was
    /// measured from. Called once, while capture is running: the frame must
    /// already be stored, or the final solve will find a calibration
    /// pointing at nothing and fall back to inferring the floor.
    func setFloorCalibration(frameId: UInt32, normal: (Double, Double, Double),
                             offsetM: Double, rmseM: Double,
                             incidenceDeg: Double, inliers: Int32) {
        sessionDoc.floorCalibration = SessionJSON.FloorCalibrationJSON(
            frameId: frameId,
            normal: [normal.0, normal.1, normal.2],
            offsetM: offsetM,
            rmseM: rmseM,
            incidenceDeg: incidenceDeg,
            inliers: inliers)
    }

    var hasFloorCalibration: Bool { sessionDoc.floorCalibration != nil }

    /// Records that the engine keyframed these frames.
    ///
    /// The decision arrives AFTER the frame is written — the tracker has to
    /// see the frame before it can judge it — so meta.json's `is_keyframe`
    /// cannot carry it and session.json's `keyframe_ids` is filled at
    /// finalize instead. Before this, every frame said `is_keyframe: false`
    /// and `keyframe_ids` was always empty: a documented field that always
    /// lied, on every device session ever captured.
    /// Persists a user's name for a region.
    ///
    /// docs/FORMATS.md documents `regions` as the user's renames, and until
    /// now renaming a room only reached the engine's in-memory map — so the
    /// name was lost the moment the session ended, on a screen that gives no
    /// hint the edit is temporary.
    func setRegionName(id: UInt32, name: String) {
        if let at = sessionDoc.regions.firstIndex(where: { $0.id == id }) {
            sessionDoc.regions[at].name = name
            sessionDoc.regions[at].renamed = true
        } else {
            sessionDoc.regions.append(
                RegionJSON(id: id, name: name, renamed: true))
        }
    }

    func addKeyframeIds(_ ids: [UInt32]) {
        for id in ids where !sessionDoc.keyframeIds.contains(id) {
            sessionDoc.keyframeIds.append(id)
        }
    }

    /// Removes extra photos tagged with this room. Scout/route frames stay —
    /// those are the localization scaffold, not the room's detail pass.
    @discardableResult
    func deleteCaptureFrames(roomId: UInt32) -> Int {
        let framesDir = directory.appendingPathComponent("frames")
        let fm = FileManager.default
        let decoder = JSONDecoder()
        let names = (try? fm.contentsOfDirectory(atPath: framesDir.path)) ?? []
        var removed = 0
        var removedIds: [UInt32] = []
        for name in names {
            let dir = framesDir.appendingPathComponent(name)
            let metaURL = dir.appendingPathComponent("meta.json")
            guard let data = try? Data(contentsOf: metaURL),
                  let meta = try? decoder.decode(FrameMetaJSON.self, from: data)
            else { continue }
            guard meta.pass != "scout", meta.roomId == roomId else { continue }
            try? fm.removeItem(at: dir)
            removedIds.append(meta.frameId)
            removed += 1
            if frameCount > 0 { frameCount -= 1 }
        }
        if removed > 0 {
            sessionDoc.keyframeIds.removeAll { removedIds.contains($0) }
        }
        return removed
    }

    /// Floorplan rooms live next to RAW, not inside it: the route photos are
    /// immutable, the room outlines are something the user can redraw.
    func writeRooms(_ plan: Floorplan) {
        struct File: Codable {
            var rooms: [RoomJSON]
        }
        struct RoomJSON: Codable {
            var id: UInt32
            var name: String
            var polygon: [[Double]]
            var scoutFrameIds: [UInt32]
            var captureCount: UInt32
            enum CodingKeys: String, CodingKey {
                case id, name, polygon
                case scoutFrameIds = "scout_frame_ids"
                case captureCount = "capture_count"
            }
        }
        let payload = File(rooms: plan.rooms.map {
            RoomJSON(id: $0.id, name: $0.name,
                     polygon: $0.polygon.map { [$0.x, $0.y] },
                     scoutFrameIds: $0.scoutFrameIds,
                     captureCount: $0.captureCount)
        })
        let enc = JSONEncoder()
        enc.outputFormatting = [.prettyPrinted, .sortedKeys]
        guard let data = try? enc.encode(payload) else { return }
        try? Self.write(data, to: directory.appendingPathComponent("rooms.json"))
    }

    /// Records that this capture re-covered a volume, superseding whatever
    /// earlier captures in the project recorded there.
    ///
    /// The volume is OBSERVED — the box the camera actually moved through
    /// during this capture — rather than predicted from a stored room
    /// outline. That is the more honest definition of "I redid this area",
    /// it is self-correcting (walk further, replace more), and it does not
    /// depend on region bounds that may have been computed from a solve two
    /// versions ago.
    ///
    /// Nothing is deleted. The superseded frames stay on disk exactly as
    /// recorded and the final solve declines to reconstruct from them, so
    /// removing this volume brings them back.
    func setSupersededVolume(min: (Double, Double, Double),
                             max: (Double, Double, Double),
                             label: String) {
        sessionDoc.supersedes = [SessionJSON.SupersedeJSON(
            min: [min.0, min.1, min.2],
            max: [max.0, max.1, max.2],
            label: label)]
    }

    var storedFrames: UInt32 { frameCount }
    var storedBytes: Int64 { bytesWritten }

    private func writeSessionJson() throws {
        try Self.writeSessionDoc(sessionDoc, encoder: encoder, to: directory)
    }

    /// Static so the actor's (nonisolated) init can call it. Calling the
    /// isolated instance method from init is a warning today and an error
    /// under Swift 6.
    private static func writeSessionDoc(_ doc: SessionJSON,
                                        encoder: JSONEncoder,
                                        to directory: URL) throws {
        let data = try encoder.encode(doc)
        try Self.write(data, to: directory.appendingPathComponent("session.json"))
    }

    private static func write(_ data: Data, to url: URL) throws {
        do {
            try data.write(to: url, options: .atomic)
        } catch {
            throw StoreError.ioFailure("\(url.lastPathComponent): \(error.localizedDescription)")
        }
    }

    private static func deviceModelIdentifier() -> String {
        var systemInfo = utsname()
        uname(&systemInfo)
        return withUnsafeBytes(of: &systemInfo.machine) { raw in
            String(decoding: raw.prefix(while: { $0 != 0 }), as: UTF8.self)
        }
    }
}
