import Foundation
import simd
import UIKit

/// Image-only session for desktop Depth Anything: `session.json` +
/// `frames/NNNNNN/image.jpg` (+ light meta). No LiDAR, no live map.
actor ImageSessionStore {
    enum StoreError: Error {
        case sessionExists
        case ioFailure(String)
    }

    let directory: URL
    private var sessionDoc: SessionJSON
    private var frameCount: UInt32 = 0
    private var bytesWritten: Int64 = 0
    private let encoder: JSONEncoder
    private(set) var nextFrameId: UInt32

    var storedFrames: UInt32 { frameCount }
    var megabytes: Double { Double(bytesWritten) / 1_048_576.0 }

    init(osVersion: String,
         continuing: ProjectStore.Project? = nil,
         newProjectName: String? = nil,
         videoW: Int = 1920, videoH: Int = 1440) throws {
        let id = SessionStore.makeSessionId()
        directory = SessionStore.documentsDirectory().appendingPathComponent(id)
        if FileManager.default.fileExists(atPath: directory.path) {
            throw StoreError.sessionExists
        }
        try FileManager.default.createDirectory(
            at: directory.appendingPathComponent("frames"),
            withIntermediateDirectories: true)

        encoder = JSONEncoder()
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]

        let startId: UInt32 = (continuing?.lastFrameId ?? 0) + 1
        nextFrameId = startId

        let iso = ISO8601DateFormatter()
        sessionDoc = SessionJSON(
            sessionId: id,
            createdUtc: iso.string(from: Date()),
            endUtc: nil,
            device: .init(model: Self.deviceModel(), ios: osVersion),
            video: .init(w: videoW, h: videoH, fps: 30, pixelFormat: "420f"),
            // Depth comes from desktop DA3 — declare absent on device.
            depth: .init(w: 0, h: 0, format: "none", filtering: false),
            capture: .init(afLocked: false, aeLocked: false, awbLocked: false,
                           gdcDisabled: true, stabilization: "arkit"),
            frameCount: 0,
            keyframeIds: [],
            regions: [],
            projectId: continuing?.id ?? SessionStore.makeProjectId(),
            projectName: continuing?.name ?? (newProjectName ?? "Untitled"),
            parentSession: continuing?.latestSessionName ?? "",
            supersedes: [],
            appVersion: Bundle.main.infoDictionary?["CFBundleShortVersionString"]
                as? String ?? "0")
        try writeSessionJson()
    }

    func writeJPEG(_ jpeg: Data, transform: simd_float4x4) throws -> UInt32 {
        let frameId = nextFrameId
        nextFrameId += 1
        let frameDir = directory
            .appendingPathComponent("frames")
            .appendingPathComponent(String(format: "%06u", frameId))
        let fm = FileManager.default
        try fm.createDirectory(at: frameDir, withIntermediateDirectories: true)
        var complete = false
        defer { if !complete { try? fm.removeItem(at: frameDir) } }

        try jpeg.write(to: frameDir.appendingPathComponent("image.jpg"),
                       options: .atomic)

        // Tiny sidecar for tooling — not required by DA3.
        let t = transform
        let pose: [[Float]] = [
            [t.columns.0.x, t.columns.1.x, t.columns.2.x, t.columns.3.x],
            [t.columns.0.y, t.columns.1.y, t.columns.2.y, t.columns.3.y],
            [t.columns.0.z, t.columns.1.z, t.columns.2.z, t.columns.3.z],
            [t.columns.0.w, t.columns.1.w, t.columns.2.w, t.columns.3.w],
        ]
        struct Side: Encodable {
            var frame_id: UInt32
            var arkit_pose: [[Float]]
        }
        let meta = try encoder.encode(Side(frame_id: frameId, arkit_pose: pose))
        try meta.write(to: frameDir.appendingPathComponent("meta.json"),
                       options: .atomic)

        complete = true
        frameCount += 1
        bytesWritten += Int64(jpeg.count + meta.count)
        sessionDoc.frameCount = frameCount
        sessionDoc.keyframeIds.append(frameId)
        try writeSessionJson()
        return frameId
    }

    func finalize() throws {
        sessionDoc.endUtc = ISO8601DateFormatter().string(from: Date())
        sessionDoc.frameCount = frameCount
        try writeSessionJson()
    }

    private func writeSessionJson() throws {
        let data = try encoder.encode(sessionDoc)
        try data.write(to: directory.appendingPathComponent("session.json"),
                       options: .atomic)
    }

    private static func deviceModel() -> String {
        var sys = utsname()
        uname(&sys)
        return withUnsafePointer(to: &sys.machine) {
            $0.withMemoryRebound(to: CChar.self, capacity: 1) {
                String(cString: $0)
            }
        }
    }
}
