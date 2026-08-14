import SwiftUI

/// The photos the solve thinks are dragging the model down, and why.
///
/// Worth showing individually rather than as a count, because the useful
/// question is not "how many" but "where were they" — a run of consecutive
/// flagged frame ids means one bad stretch of the walk, which is a thing
/// someone can go back and redo. Scattered ids mean a systemic problem
/// (light level, walking speed) that rescanning the same way will reproduce.
struct FlaggedImagesView: View {
    let images: [SolveReport.Image]

    var body: some View {
        List {
            Section {
                Text(runsDescription)
                    .font(.callout)
                    .foregroundStyle(.secondary)
            }

            ForEach(images.sorted { $0.frameId < $1.frameId }) { image in
                VStack(alignment: .leading, spacing: 4) {
                    HStack {
                        Text(image.name)
                            .font(.subheadline.weight(.medium).monospacedDigit())
                        Spacer()
                        if !image.registered {
                            Text("not placed")
                                .font(.caption2.weight(.semibold))
                                .foregroundStyle(.red)
                        }
                    }
                    Text(image.flags.map(Self.label).joined(separator: " · "))
                        .font(.caption)
                        .foregroundStyle(.orange)
                    if image.registered {
                        Text(String(format: "%d points · %.2f px error",
                                    image.observations, image.reprojRmsePx))
                            .font(.caption2.monospacedDigit())
                            .foregroundStyle(.secondary)
                    }
                }
                .padding(.vertical, 2)
            }
        }
        .navigationTitle("Photos worth a look")
        .navigationBarTitleDisplayMode(.inline)
    }

    /// Consecutive flagged frames are one bad stretch of walking; scattered
    /// ones are a problem with the whole session. Different fixes, so it is
    /// worth saying which this is.
    private var runsDescription: String {
        let ids = images.map(\.frameId).sorted()
        guard ids.count > 1 else {
            return "One photo is worth checking. A single frame rarely matters."
        }
        var runs = 1
        for (a, b) in zip(ids, ids.dropFirst()) where b > a + 3 { runs += 1 }
        if runs <= max(2, ids.count / 5) {
            return "These fall into \(runs) stretch\(runs == 1 ? "" : "es") of "
                 + "the walk — go back over those areas and they are fixable."
        }
        return "These are spread across the whole session, which usually means "
             + "the light level or the walking pace rather than one bad spot."
    }

    private static func label(_ flag: String) -> String {
        switch flag {
        case "blurry": return "soft"
        case "overexposed": return "blown highlights"
        case "weakly_observed": return "few matched points"
        case "unregistered": return "could not be placed"
        default: return flag
        }
    }
}
