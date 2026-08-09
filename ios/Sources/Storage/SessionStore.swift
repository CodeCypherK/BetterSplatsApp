import AVFoundation
import CoreImage
import Foundation
import UIKit

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
    private var bytesWritten: Int64 = 0
    private var calibrationWritten = false
    private let encoder: JSONEncoder

    static func documentsDirectory() -> URL {
        FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
    }

    static func makeSessionId(date: Date = Date()) -> String {
        let fmt = DateFormatter()
        fmt.dateFormat = "yyyyMMdd-HHmmss"
        fmt.timeZone = TimeZone(identifier: "UTC")
        let suffix = String(format: "%06x", UInt32.random(in: 0...0xFFFFFF))
        return "session_\(fmt.string(from: date))_\(suffix)"
    }

    init(videoW: Int, videoH: Int, depthW: Int, depthH: Int, fps: Int) throws {
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
                ios: UIDevice.current.systemVersion),
            video: .init(w: videoW, h: videoH, fps: fps, pixelFormat: "420f"),
            depth: .init(w: depthW, h: depthH, format: "hdep", filtering: false),
            capture: .init(afLocked: true, gdcDisabled: true, stabilization: "off"),
            frameCount: 0,
            keyframeIds: [],
            regions: [RegionJSON(id: 1, name: "Room 1", renamed: false)],
            appVersion: Bundle.main.infoDictionary?["CFBundleShortVersionString"]
                as? String ?? "0")
        try writeSessionJson()
    }

    /// Records the session camera model from the first frame's calibration:
    /// intrinsics at reference dimensions plus Apple's radial distortion LUT.
    func writeCalibrationIfNeeded(_ calibration: AVCameraCalibrationData) throws {
        guard !calibrationWritten else { return }
        calibrationWritten = true

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
        try write(data, to: directory.appendingPathComponent("calibration.json"))
    }

    /// Appends one immutable frame. Fails rather than overwrites.
    func writeFrame(_ payload: FramePayload) throws {
        let frameDir = directory
            .appendingPathComponent("frames")
            .appendingPathComponent(String(format: "%06u", payload.frameId))
        let fm = FileManager.default
        guard !fm.fileExists(atPath: frameDir.path) else {
            throw StoreError.ioFailure("frame \(payload.frameId) already exists")
        }
        try fm.createDirectory(at: frameDir, withIntermediateDirectories: true)

        try write(payload.jpeg, to: frameDir.appendingPathComponent("image.jpg"))

        var encodedLen = 0
        let encoded = payload.depthF16.withUnsafeBufferPointer { buf in
            bs_depth_encode(
                buf.baseAddress, Int32(payload.depthWidth),
                Int32(payload.depthHeight), &encodedLen)
        }
        guard let encoded else { throw StoreError.depthEncodeFailed }
        defer { bs_buffer_release(encoded) }
        try write(
            Data(bytes: encoded, count: encodedLen),
            to: frameDir.appendingPathComponent("lidar.depth"))

        let metaData = try encoder.encode(payload.meta)
        try write(metaData, to: frameDir.appendingPathComponent("meta.json"))

        frameCount += 1
        bytesWritten += Int64(payload.jpeg.count + encodedLen + metaData.count)
        if payload.meta.isKeyframe {
            sessionDoc.keyframeIds.append(payload.frameId)
        }
    }

    func finalize() throws {
        sessionDoc.endUtc = ISO8601DateFormatter().string(from: Date())
        sessionDoc.frameCount = frameCount
        try writeSessionJson()
    }

    var storedFrames: UInt32 { frameCount }
    var storedBytes: Int64 { bytesWritten }

    private func writeSessionJson() throws {
        let data = try encoder.encode(sessionDoc)
        try write(data, to: directory.appendingPathComponent("session.json"))
    }

    private func write(_ data: Data, to url: URL) throws {
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
