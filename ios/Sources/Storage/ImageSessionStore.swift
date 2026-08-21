import Foundation
import simd
import UIKit

/// One folder per room: flat JPEGs (`000001.jpg`, …) plus a small
/// `session.json` for project bookkeeping. No `frames/` tree.
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

    /// Creates a new room folder, or reopens the latest capture when
    /// continuing a project so all images stay in one place.
    init(osVersion: String,
         continuing: ProjectStore.Project? = nil,
         newProjectName: String? = nil,
         videoW: Int = 1920, videoH: Int = 1440) throws {
        encoder = JSONEncoder()
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]

        let fm = FileManager.default
        let docs = SessionStore.documentsDirectory()

        if let existing = continuing?.latestDirectory,
           fm.fileExists(atPath: existing.path) {
            directory = existing
            let data = try Data(contentsOf: existing.appendingPathComponent("session.json"))
            sessionDoc = try JSONDecoder().decode(SessionJSON.self, from: data)
            let highest = Self.highestImageId(in: existing)
            nextFrameId = highest + 1
            frameCount = sessionDoc.frameCount
            bytesWritten = Self.directoryBytes(existing)
            sessionDoc.video = .init(w: videoW, h: videoH, fps: 30,
                                     pixelFormat: "420f")
            try writeSessionJson()
            return
        }

        let roomName = Self.uniqueRoomName(
            continuing?.name ?? newProjectName ?? "Room", under: docs)
        directory = docs.appendingPathComponent(roomName)
        if fm.fileExists(atPath: directory.path) {
            throw StoreError.sessionExists
        }
        try fm.createDirectory(at: directory, withIntermediateDirectories: true)

        nextFrameId = 1
        let iso = ISO8601DateFormatter()
        sessionDoc = SessionJSON(
            sessionId: roomName,
            createdUtc: iso.string(from: Date()),
            endUtc: nil,
            device: .init(model: Self.deviceModel(), ios: osVersion),
            video: .init(w: videoW, h: videoH, fps: 30, pixelFormat: "420f"),
            depth: .init(w: 0, h: 0, format: "none", filtering: false),
            capture: .init(afLocked: false, aeLocked: false, awbLocked: false,
                           gdcDisabled: true, stabilization: "off"),
            frameCount: 0,
            keyframeIds: [],
            regions: [],
            projectId: continuing?.id ?? SessionStore.makeProjectId(),
            projectName: continuing?.name ?? (newProjectName ?? roomName),
            parentSession: continuing?.latestSessionName ?? "",
            supersedes: [],
            appVersion: Bundle.main.infoDictionary?["CFBundleShortVersionString"]
                as? String ?? "0")
        try writeSessionJson()
    }

    func writeJPEG(_ jpeg: Data, transform: simd_float4x4) throws -> UInt32 {
        let frameId = nextFrameId
        nextFrameId += 1
        let name = String(format: "%06u.jpg", frameId)
        let url = directory.appendingPathComponent(name)
        try jpeg.write(to: url, options: .atomic)
        frameCount += 1
        bytesWritten += Int64(jpeg.count)
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

    private static func uniqueRoomName(_ raw: String, under docs: URL) -> String {
        let base = DesktopSender.sanitize(raw)
        let fm = FileManager.default
        var name = base
        var n = 2
        while fm.fileExists(atPath: docs.appendingPathComponent(name).path) {
            name = "\(base)_\(n)"
            n += 1
        }
        return name
    }

    nonisolated static func highestImageId(in directory: URL) -> UInt32 {
        let names = (try? FileManager.default.contentsOfDirectory(
            atPath: directory.path)) ?? []
        var highest: UInt32 = 0
        for name in names where name.lowercased().hasSuffix(".jpg") {
            let stem = (name as NSString).deletingPathExtension
            if let id = UInt32(stem), id > highest { highest = id }
        }
        // Legacy frames/NNNNNN/image.jpg
        let frames = directory.appendingPathComponent("frames")
        let legacy = (try? FileManager.default.contentsOfDirectory(
            atPath: frames.path)) ?? []
        for name in legacy {
            if let id = UInt32(name), id > highest { highest = id }
        }
        return highest
    }

    private static func directoryBytes(_ url: URL) -> Int64 {
        let fm = FileManager.default
        let names = (try? fm.contentsOfDirectory(atPath: url.path)) ?? []
        var total: Int64 = 0
        for name in names {
            let u = url.appendingPathComponent(name)
            if let n = try? u.resourceValues(forKeys: [.fileSizeKey]).fileSize {
                total += Int64(n)
            }
        }
        return total
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
