import Foundation

/// Codable mirrors of the RAW-layer JSON schemas. docs/FORMATS.md is
/// normative — key names here must match it exactly; the C++ session reader
/// and `bs_synth` implement the same document.
enum SchemaV { static let version = 1 }

struct PinholeIntrinsicsJSON: Codable {
    var fx: Double
    var fy: Double
    var cx: Double
    var cy: Double
    var refW: Int
    var refH: Int

    enum CodingKeys: String, CodingKey {
        case fx, fy, cx, cy
        case refW = "ref_w"
        case refH = "ref_h"
    }
}

struct ExposureJSON: Codable {
    var durationS: Double
    var iso: Double
    var biasEv: Double

    enum CodingKeys: String, CodingKey {
        case durationS = "duration_s"
        case iso
        case biasEv = "bias_ev"
    }
}

struct QualityJSON: Codable {
    var lapVar: Double
    var overexpFrac: Double

    enum CodingKeys: String, CodingKey {
        case lapVar = "lap_var"
        case overexpFrac = "overexp_frac"
    }
}

struct FrameMetaJSON: Codable {
    var schemaVersion = SchemaV.version
    var frameId: UInt32
    var tCapture: Double
    var tDepth: Double
    var intrinsics: PinholeIntrinsicsJSON
    var depthIntrinsics: PinholeIntrinsicsJSON
    var distortionRef = "session"
    var exposure: ExposureJSON
    var quality: QualityJSON
    var isKeyframe: Bool
    var storeReason: String

    enum CodingKeys: String, CodingKey {
        case schemaVersion = "schema_version"
        case frameId = "frame_id"
        case tCapture = "t_capture"
        case tDepth = "t_depth"
        case intrinsics
        case depthIntrinsics = "depth_intrinsics"
        case distortionRef = "distortion_ref"
        case exposure, quality
        case isKeyframe = "is_keyframe"
        case storeReason = "store_reason"
    }
}

struct DistortionLutJSON: Codable {
    var magnification: [Float]
    var inverse: [Float]
    var center: [Double]
}

struct CalibrationJSON: Codable {
    var schemaVersion = SchemaV.version
    var reference: Reference
    var intrinsicsSession: PinholeIntrinsicsJSON
    var distortionLut: DistortionLutJSON

    struct Reference: Codable {
        var width: Int
        var height: Int
    }

    enum CodingKeys: String, CodingKey {
        case schemaVersion = "schema_version"
        case reference
        case intrinsicsSession = "intrinsics_session"
        case distortionLut = "distortion_lut"
    }
}

struct RegionJSON: Codable {
    var id: UInt32
    var name: String
    var renamed: Bool
}

struct SessionJSON: Codable {
    var schemaVersion = SchemaV.version
    var sessionId: String
    var createdUtc: String
    var endUtc: String?
    var device: Device
    var video: Video
    var depth: Depth
    var capture: Capture
    var frameCount: UInt32
    var keyframeIds: [UInt32]
    var regions: [RegionJSON]
    var appVersion: String

    struct Device: Codable {
        var model: String
        var ios: String
    }
    struct Video: Codable {
        var w: Int
        var h: Int
        var fps: Int
        var pixelFormat: String
        enum CodingKeys: String, CodingKey {
            case w, h, fps
            case pixelFormat = "pixel_format"
        }
    }
    struct Depth: Codable {
        var w: Int
        var h: Int
        var format: String
        var filtering: Bool
    }
    struct Capture: Codable {
        var afLocked: Bool
        var gdcDisabled: Bool
        var stabilization: String
        enum CodingKeys: String, CodingKey {
            case afLocked = "af_locked"
            case gdcDisabled = "gdc_disabled"
            case stabilization
        }
    }

    enum CodingKeys: String, CodingKey {
        case schemaVersion = "schema_version"
        case sessionId = "session_id"
        case createdUtc = "created_utc"
        case endUtc = "end_utc"
        case device, video, depth, capture
        case frameCount = "frame_count"
        case keyframeIds = "keyframe_ids"
        case regions
        case appVersion = "app_version"
    }
}
