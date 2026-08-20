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
    /// "capture" or "scout". Scout frames come from the opening circuit and
    /// exist to localize against, not to reconstruct from — the final solve
    /// drops them. They are still written to RAW like every other
    /// measurement: "not reconstructed from" is a solve-time decision, never
    /// a licence to discard what the sensor saw.
    var pass = "capture"
    /// Which floorplan room this extra photo belongs to. Absent on scout
    /// frames and on single-room captures.
    var roomId: UInt32?

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
        case pass
        case roomId = "room_id"
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

/// Floorplan rooms written beside RAW (`rooms.json`). Outlines and names are
/// editable; the frames they point at are not.
struct RoomsFileJSON: Codable {
    var rooms: [RoomOutlineJSON]
}

struct RoomOutlineJSON: Codable {
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

enum RoomsDocument {
    static func load(from sessionDirectory: URL) -> RoomsFileJSON? {
        let url = sessionDirectory.appendingPathComponent("rooms.json")
        guard let data = try? Data(contentsOf: url) else { return nil }
        return try? JSONDecoder().decode(RoomsFileJSON.self, from: data)
    }

    static func save(_ file: RoomsFileJSON, to sessionDirectory: URL) {
        let enc = JSONEncoder()
        enc.outputFormatting = [.prettyPrinted, .sortedKeys]
        guard let data = try? enc.encode(file) else { return }
        try? data.write(
            to: sessionDirectory.appendingPathComponent("rooms.json"),
            options: .atomic)
    }

    static func rename(roomId: UInt32, to name: String,
                       in sessionDirectory: URL) {
        guard var file = load(from: sessionDirectory),
              let i = file.rooms.firstIndex(where: { $0.id == roomId })
        else { return }
        let trimmed = name.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmed.isEmpty else { return }
        file.rooms[i].name = trimmed
        save(file, to: sessionDirectory)
    }
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
    /// Optional: the floor as the depth sensor measured it while the user
    /// aimed at it, in that frame's CAMERA coordinates. Stored with the
    /// frame id rather than in world coordinates so the final solve derives
    /// the world plane from that frame's final pose — a world plane written
    /// here would bake in whatever the live tracker believed at the time.
    var floorCalibration: FloorCalibrationJSON?
    /// The project this capture belongs to, and the sibling capture it
    /// continues. A space bigger than one capture is a chain of these, all
    /// sharing one world frame.
    ///
    /// `parentSession` is a NAME, not a path, so a project survives being
    /// zipped up here and unzipped on another machine.
    ///
    /// Optional because Swift's synthesized Codable requires every
    /// non-optional key to be PRESENT: declaring these outright would make
    /// every session written before projects existed fail to decode, and
    /// vanish from the app. Defaults live at the use site, matching how the
    /// C++ reader treats the same keys.
    var projectId: String?
    var projectName: String?
    var parentSession: String?
    /// World volumes this capture re-covered, superseding what earlier
    /// captures in the project recorded there. The frames themselves are
    /// never touched — the final solve just declines to reconstruct from
    /// them, so the decision is reversible.
    var supersedes: [SupersedeJSON]?
    var appVersion: String

    struct SupersedeJSON: Codable {
        var min: [Double]  // x, y, z
        var max: [Double]
        var label: String
    }

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
    struct FloorCalibrationJSON: Codable {
        var frameId: UInt32
        var normal: [Double]
        var offsetM: Double
        var rmseM: Double
        var incidenceDeg: Double
        var inliers: Int32
        enum CodingKeys: String, CodingKey {
            case frameId = "frame_id"
            case normal
            case offsetM = "offset_m"
            case rmseM = "rmse_m"
            case incidenceDeg = "incidence_deg"
            case inliers
        }
    }

    struct Capture: Codable {
        var afLocked: Bool
        var aeLocked: Bool
        var isoLocked: Bool
        var awbLocked: Bool
        var gdcDisabled: Bool
        var stabilization: String
        enum CodingKeys: String, CodingKey {
            case afLocked = "af_locked"
            case aeLocked = "ae_locked"
            case isoLocked = "iso_locked"
            case awbLocked = "awb_locked"
            case gdcDisabled = "gdc_disabled"
            case stabilization
        }

        init(afLocked: Bool, aeLocked: Bool, isoLocked: Bool = false,
             awbLocked: Bool, gdcDisabled: Bool, stabilization: String) {
            self.afLocked = afLocked
            self.aeLocked = aeLocked
            self.isoLocked = isoLocked
            self.awbLocked = awbLocked
            self.gdcDisabled = gdcDisabled
            self.stabilization = stabilization
        }

        init(from decoder: Decoder) throws {
            let c = try decoder.container(keyedBy: CodingKeys.self)
            afLocked = try c.decode(Bool.self, forKey: .afLocked)
            aeLocked = try c.decodeIfPresent(Bool.self, forKey: .aeLocked) ?? false
            isoLocked = try c.decodeIfPresent(Bool.self, forKey: .isoLocked) ?? false
            awbLocked = try c.decodeIfPresent(Bool.self, forKey: .awbLocked) ?? false
            gdcDisabled = try c.decode(Bool.self, forKey: .gdcDisabled)
            stabilization = try c.decode(String.self, forKey: .stabilization)
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
        case floorCalibration = "floor_calibration"
        case projectId = "project_id"
        case projectName = "project_name"
        case parentSession = "parent_session"
        case supersedes
        case appVersion = "app_version"
    }
}
