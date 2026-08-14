import SwiftUI

/// The splat-readiness dashboard: per-region score card with the five
/// sub-score bars and the ranked weak-area list with actionable guidance.
struct ReadinessDashboardView: View {
    let model: CaptureViewModel
    @State private var renameText = ""
    @State private var renameRegionID: UInt32 = 0
    @State private var renaming = false

    private static let axisNames = WeakAreaGuidance.axisNames

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
                            renameRegionID = region.id
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

                    if region.weakAreaCount > 0 {
                        Label(weakSummary(for: region),
                              systemImage: "exclamationmark.triangle.fill")
                            .font(.caption)
                            .foregroundStyle(.orange)
                    } else {
                        Label("No weak spots — ready to capture",
                              systemImage: "checkmark.circle.fill")
                            .font(.caption)
                            .foregroundStyle(.green)
                    }
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
                                Text(WeakAreaGuidance.title(
                                    for: area, viewer: model.viewer,
                                    regionName: regionName(for: area)))
                                    .font(.subheadline.weight(.medium))
                                Spacer()
                                Text("\(Int(area.score))%")
                                    .font(.caption.monospacedDigit())
                                    .foregroundStyle(.secondary)
                            }
                            Text(WeakAreaGuidance.advice(for: area,
                                                         viewer: model.viewer))
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
            Button("Rename") {
                if renameRegionID != 0 {
                    model.renameRegion(id: renameRegionID, name: renameText)
                }
            }
            Button("Cancel", role: .cancel) {}
        }
    }

    private func scoreColor(_ score: Float) -> Color {
        score >= 85 ? .green : (score >= 60 ? .orange : .red)
    }

    private func weakSummary(for region: CoreEngine.Snapshot.Region) -> String {
        let n = region.weakAreaCount
        let spots = "\(n) weak spot\(n == 1 ? "" : "s")"
        if region.worstDeficiency >= 0, region.worstDeficiency < Self.axisNames.count {
            return "\(spots) · worst: \(Self.axisNames[region.worstDeficiency])"
        }
        return spots
    }

    /// Region name shown on weak areas only once there are several rooms.
    private func regionName(for area: CoreEngine.Snapshot.WeakArea) -> String? {
        guard model.snapshot.regions.count > 1 else { return nil }
        return model.snapshot.regions.first { $0.id == area.regionID }?.name
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
