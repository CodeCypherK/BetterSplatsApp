import simd
import SwiftUI

/// The splat-readiness dashboard: per-region score card with the five
/// sub-score bars and the ranked weak-area list with actionable guidance.
struct ReadinessDashboardView: View {
    let model: CaptureViewModel
    @State private var renameText = ""
    @State private var renaming = false

    private static let axisNames = ["Geometry", "Camera poses", "Texture",
                                    "LiDAR coverage", "View overlap"]

    var body: some View {
        List {
            if model.snapshot.regions.isEmpty {
                ContentUnavailableView(
                    "No readiness data yet",
                    systemImage: "chart.bar.doc.horizontal",
                    description: Text("Scores appear once the map bootstraps."))
            }

            ForEach(model.snapshot.regions, id: \.id) { region in
                Section {
                    HStack {
                        Text(region.name.uppercased())
                            .font(.headline)
                        Spacer()
                        Text("\(Int(region.score))%")
                            .font(.title2.weight(.bold).monospacedDigit())
                            .foregroundStyle(scoreColor(region.score))
                        Button {
                            renameText = region.name
                            renaming = true
                        } label: {
                            Image(systemName: "pencil")
                        }
                        .buttonStyle(.borderless)
                    }
                    ScoreBar(value: region.score)

                    ForEach(0..<5, id: \.self) { axis in
                        HStack {
                            Text(Self.axisNames[axis])
                                .font(.subheadline)
                                .frame(width: 130, alignment: .leading)
                            ScoreBar(value: region.sub[axis])
                            Text("\(Int(region.sub[axis]))%")
                                .font(.caption.monospacedDigit())
                                .frame(width: 40, alignment: .trailing)
                        }
                    }

                    LabeledContent("Coverage",
                                   value: String(format: "%.1f m² · %d patches",
                                                 region.areaM2,
                                                 region.patchCount))
                        .font(.caption)
                } header: {
                    Text("Region")
                }
            }

            if !model.snapshot.weakAreas.isEmpty {
                Section("Weak areas — fix these for a clean splat") {
                    ForEach(Array(model.snapshot.weakAreas.enumerated()),
                            id: \.offset) { _, area in
                        VStack(alignment: .leading, spacing: 4) {
                            HStack {
                                Image(systemName: "exclamationmark.triangle.fill")
                                    .foregroundStyle(.orange)
                                Text(Self.title(for: area))
                                    .font(.subheadline.weight(.medium))
                                Spacer()
                                Text("\(Int(area.score))%")
                                    .font(.caption.monospacedDigit())
                                    .foregroundStyle(.secondary)
                            }
                            Text(Self.advice(for: area))
                                .font(.caption)
                                .foregroundStyle(.secondary)
                        }
                        .padding(.vertical, 2)
                    }
                }
            }
        }
        .navigationTitle("Splat Readiness")
        .navigationBarTitleDisplayMode(.inline)
        .onAppear { model.refreshSnapshot() }
        .alert("Rename region", isPresented: $renaming) {
            TextField("Name", text: $renameText)
            Button("Rename") { model.renameRegion(name: renameText) }
            Button("Cancel", role: .cancel) {}
        }
    }

    private func scoreColor(_ score: Float) -> Color {
        score >= 85 ? .green : (score >= 60 ? .orange : .red)
    }

    static func title(for area: CoreEngine.Snapshot.WeakArea) -> String {
        let surface = ["Wall", "Floor", "Ceiling", "Object"][
            min(3, max(0, area.surfaceKind))]
        let distance = simd_length(area.center)
        return String(format: "%@ · %.1f m away", surface, distance)
    }

    static func advice(for area: CoreEngine.Snapshot.WeakArea) -> String {
        let feet = max(1, Int((area.moveDistM * 3.28).rounded()))
        switch area.deficiency {
        case 0:
            return "Insufficient visual geometry — move \(feet) ft sideways and capture this surface again."
        case 1:
            return "Weak camera support here — re-approach this area from a well-tracked direction, keeping overlap."
        case 2:
            return "Low texture detail — LiDAR is anchoring the surface; move closer for sharper texture."
        case 3:
            return "Poor LiDAR coverage — move within 10 ft and face the surface directly."
        default:
            return "Not enough viewpoints — arc around this area, stepping \(feet) ft between shots."
        }
    }
}

private struct ScoreBar: View {
    let value: Float

    var body: some View {
        GeometryReader { proxy in
            ZStack(alignment: .leading) {
                Capsule().fill(Color.primary.opacity(0.1))
                Capsule()
                    .fill(value >= 85 ? Color.green
                          : (value >= 60 ? Color.orange : Color.red))
                    .frame(width: proxy.size.width
                           * CGFloat(max(0, min(100, value))) / 100)
            }
        }
        .frame(height: 8)
    }
}
