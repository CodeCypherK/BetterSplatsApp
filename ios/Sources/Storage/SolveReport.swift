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
        // Say what happened. Do NOT say why unless it is actually known.
        //
        // This used to attribute every unplaced photo to "moving or turning
        // too fast for the tracker to keep up" — a guess, stated as fact, and
        // one that blames the person holding the phone. On a scan where 2 of
        // 429 photos placed it was not merely unhelpful but wrong: a
        // reconstruction that fails almost completely has failed for a
        // structural reason, and telling someone to walk slower sends them
        // to repeat a scan that was never the problem.
        if let fraction = registrationFraction, fraction < 0.95,
           let total = imagesTotal, let registered = imagesRegistered {
            let missing = total - registered
            if fraction < 0.2 {
                out.append("Only \(registered) of \(total) photos could be "
                         + "placed, which is a failure of the reconstruction "
                         + "rather than of the capture — a scan this size "
                         + "does not normally degrade, it either works or it "
                         + "does not. Share the full session (not just the "
                         + "COLMAP export) so it can be diagnosed; the photos "
                         + "themselves are intact and can be re-solved.")
            } else if fraction < 0.7 {
                out.append("\(missing) of \(total) photos could not be "
                         + "placed. With this many missing the cause is worth "
                         + "looking into rather than guessing at — sharing "
                         + "the full session preserves everything needed to.")
            } else {
                out.append("\(missing) photos could not be placed. At this "
                         + "level that is normal: photos at the very start, "
                         + "or looking at a blank surface with nothing to "
                         + "match against, often cannot be tied to the rest.")
            }
        }
        let flagged = flaggedImages
        let blurry = flagged.filter { $0.flags.contains("blurry") }.count
        let overexposed = flagged.filter { $0.flags.contains("overexposed") }.count
        let total = imagesTotal ?? 0
        // Only worth raising when it is a large enough share to change the
        // result. "110 photos are softer" out of 429 reads as an accusation;
        // it is also just what a handheld walk looks like.
        if total > 0, Double(blurry) / Double(total) > 0.35 {
            out.append("\(blurry) of \(total) photos are softer than the "
                     + "rest of this session. That is a large enough share to "
                     + "cost detail — more light, or a slower walk, gives the "
                     + "camera a shorter exposure to work with.")
        }
        // Blown highlights are NOT a mistake. Any room with a window on a
        // sunny day clips somewhere, and there is nothing the person holding
        // the phone can reasonably do about it. Worth a line only when it is
        // widespread enough to actually cost surface.
        if total > 0, Double(overexposed) / Double(total) > 0.4 {
            out.append("\(overexposed) of \(total) photos have large "
                     + "blown-out areas. Windows do this and it is expected; "
                     + "it only matters because a splat cannot invent detail "
                     + "that was clipped, so those surfaces will be soft.")
        }
        // Levelling has three outcomes and they need different things said.
        if levelled == false {
            // Three different situations, and the old text told all of them
            // to go and measure the floor — including the case where the user
            // HAD measured it and was told to do the thing they just did.
            if floorMeasured == true {
                out.append("The floor was measured during the scan, but the "
                         + "photo it was measured from is one of the ones "
                         + "that could not be placed — so there was no pose "
                         + "to resolve it through. Fixing the placement above "
                         + "fixes this too; it is not a separate problem.")
            } else {
                out.append("The floor could not be found, so the model is not "
                         + "levelled — it will import at an arbitrary tilt. "
                         + "Measuring the floor at the start of a scan fixes "
                         + "this.")
            }
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
