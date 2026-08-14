import Foundation

/// `final/report.json`, read back so the app can say how the scan came out.
///
/// The engine measures a great deal about a finished reconstruction and,
/// until now, all of it died in a file the user was never going to open. This
/// is the last screen before export — the moment where "is this good enough,
/// or should I rescan?" actually gets decided — so it is the one place the
/// measurements are worth anything.
///
/// Every field is optional. A report written by an older build, or truncated
/// by a solve that died mid-write, has to degrade to "we know less" rather
/// than to a parse failure that hides the whole thing.
struct SolveReport: Decodable {
    struct ImageFlags: Decodable {
        var blurry: Int?
        var overexposed: Int?
        var weaklyObserved: Int?
        var unregistered: Int?
        var medianLapVar: Double?
        var medianObservations: Double?

        enum CodingKeys: String, CodingKey {
            case blurry, overexposed
            case weaklyObserved = "weakly_observed"
            case unregistered
            case medianLapVar = "median_lap_var"
            case medianObservations = "median_observations"
        }
    }

    struct Image: Decodable, Identifiable {
        var frameId: UInt32
        var name: String
        var registered: Bool
        var observations: Int
        var reprojRmsePx: Double
        var lapVar: Double
        var overexpFrac: Double
        var flags: [String]

        var id: UInt32 { frameId }

        enum CodingKeys: String, CodingKey {
            case frameId = "frame_id"
            case name, registered, observations
            case reprojRmsePx = "reproj_rmse_px"
            case lapVar = "lap_var"
            case overexpFrac = "overexp_frac"
            case flags
        }
    }

    var imagesTotal: Int?
    var imagesRegistered: Int?
    var points: Int?
    var reprojRmsePx: Double?
    var meanTrackLen: Double?
    var floatersRemoved: Int?
    var levelled: Bool?
    var floorMeasured: Bool?
    var levelCameraHeightM: Double?
    var levelCameraHeightSpreadM: Double?
    var imageFlags: ImageFlags?
    var images: [Image]?
    var readiness: Readiness?

    /// Splat readiness recomputed from the FINAL reconstruction — the same
    /// five axes shown live, but from globally adjusted geometry with the
    /// outliers pruned. This is the version worth acting on: it answers "is
    /// the data I am about to train on any good", where the live scores
    /// answer "is this room worth more of my time right now".
    struct Readiness: Decodable {
        var present: Bool
        var overall: Double?
        var overallSub: [Double]?
        var regions: [Region]?

        enum CodingKeys: String, CodingKey {
            case present, overall, regions
            case overallSub = "overall_sub"
        }

        struct Region: Decodable, Identifiable {
            var id: UInt32
            var name: String
            var score: Double
            var sub: [Double]
            var areaM2: Double
            var weakAreas: UInt32
            var worstDeficiency: Int
            /// World bounds of the room. The only place this exists after a
            /// solve, and what a rescan of it would have to cover.
            var min: [Double]
            var max: [Double]

            enum CodingKeys: String, CodingKey {
                case id, name, score, sub, min, max
                case areaM2 = "area_m2"
                case weakAreas = "weak_areas"
                case worstDeficiency = "worst_deficiency"
            }

            /// Which axis is dragging this room down, named.
            var worstAxisName: String? {
                guard worstDeficiency >= 0,
                      worstDeficiency < WeakAreaGuidance.axisNames.count
                else { return nil }
                return WeakAreaGuidance.axisNames[worstDeficiency]
            }
        }
    }

    enum CodingKeys: String, CodingKey {
        case imagesTotal = "images_total"
        case imagesRegistered = "images_registered"
        case points
        case reprojRmsePx = "reproj_rmse_px"
        case meanTrackLen = "mean_track_len"
        case floatersRemoved = "floaters_removed"
        case levelled
        case floorMeasured = "floor_measured"
        case levelCameraHeightM = "level_camera_height_m"
        case levelCameraHeightSpreadM = "level_camera_height_spread_m"
        case imageFlags = "image_flags"
        case images
        case readiness
    }

    static func read(sessionURL: URL) -> SolveReport? {
        let url = sessionURL.appendingPathComponent("final/report.json")
        guard let data = try? Data(contentsOf: url) else { return nil }
        return try? JSONDecoder().decode(SolveReport.self, from: data)
    }

    // MARK: - Plain-language verdicts

    /// Fraction of images the solve managed to place, 0...1.
    var registrationFraction: Double? {
        guard let total = imagesTotal, total > 0,
              let registered = imagesRegistered else { return nil }
        return Double(registered) / Double(total)
    }

    /// Images carrying at least one flag. Not the sum of the flag counts —
    /// one frame can be blurry AND weakly observed, and telling someone
    /// "14 problems" when 8 frames are involved overstates it.
    var flaggedImages: [Image] {
        (images ?? []).filter { !$0.flags.isEmpty }
    }

    /// One line for the top of the summary. Deliberately about the scan, not
    /// about the solver: "94% of your photos were placed" is a fact the user
    /// can act on, "reprojection RMSE 0.45 px" is not.
    enum Verdict {
        case good, fair, poor, unknown
    }

    var verdict: Verdict {
        guard let fraction = registrationFraction else { return .unknown }
        // Registration is the dominant term because an unplaced image
        // contributes nothing at all, while a slightly soft one still
        // contributes most of its detail.
        if fraction >= 0.95 { return .good }
        if fraction >= 0.80 { return .fair }
        return .poor
    }

    var headline: String {
        switch verdict {
        case .good: return "Good scan"
        case .fair: return "Usable scan"
        case .poor: return "Patchy scan"
        case .unknown: return "Scan complete"
        }
    }

    var summary: String {
        guard let fraction = registrationFraction,
              let registered = imagesRegistered, let total = imagesTotal else {
            return "The reconstruction finished."
        }
        let placed = "\(registered) of \(total) photos placed "
            + "(\(Int((fraction * 100).rounded()))%)"
        switch verdict {
        case .good:
            return placed + ". Ready to train."
        case .fair:
            return placed + ". Trainable, though the unplaced ones leave gaps."
        case .poor:
            return placed + ". Expect holes — worth rescanning the areas that "
                 + "were hardest to track."
        case .unknown:
            return placed + "."
        }
    }

    /// Concrete follow-ups, most useful first. Empty when there is nothing
    /// worth saying — silence is a legitimate result and better than padding
    /// the list to look thorough.
    var advice: [String] {
        var out: [String] = []
        if let fraction = registrationFraction, fraction < 0.95,
           let total = imagesTotal, let registered = imagesRegistered {
            out.append("\(total - registered) photos could not be placed. "
                     + "They were usually taken while moving or turning too "
                     + "fast for the tracker to keep up.")
        }
        let flagged = flaggedImages
        let blurry = flagged.filter { $0.flags.contains("blurry") }.count
        let overexposed = flagged.filter { $0.flags.contains("overexposed") }.count
        if blurry > 0 {
            out.append("\(blurry) photos are noticeably softer than the rest "
                     + "of the session. Moving more slowly is the fix.")
        }
        if overexposed > 0 {
            out.append("\(overexposed) photos have blown-out highlights. "
                     + "Bright windows are the usual cause; a splat cannot "
                     + "recover detail that was clipped at capture.")
        }
        // Levelling has three outcomes and they need different things said.
        if levelled == false {
            out.append("The floor could not be found, so the model is not "
                     + "levelled — it will import at an arbitrary tilt. "
                     + "Measuring the floor at the start of a scan fixes this.")
        } else if let spread = levelCameraHeightSpreadM, spread > 0.15 {
            // Camera height should barely vary across a handheld walk. A wide
            // spread means the fitted floor does not agree with where the
            // camera actually was, i.e. the levelling is resting on geometry
            // that is itself off — measured at 1.12 +- 0.41 m on a two-room
            // solve carrying 0.2 m of error, against 1.494 +- 0.011 on a
            // clean one.
            out.append("The levelling is uncertain — camera height varies by "
                     + String(format: "%.0f cm", spread * 100)
                     + " across the scan, which usually means the "
                     + "reconstruction itself is bent rather than the floor.")
        } else if floorMeasured == false {
            out.append("The floor was inferred rather than measured. Pointing "
                     + "the phone at the floor at the start of the next scan "
                     + "makes the result sit level more reliably.")
        }
        return out
    }
}
