import SwiftUI

/// On-device final reconstruction screen: staged progress, live metrics,
/// thermal banner, cancel, and the export handoff when done.
struct ProcessingView: View {
    @State private var model: ProcessingViewModel
    @State private var exportURL: URL?
    @State private var exportError: String?
    /// Non-nil while a zip is being built. Names the archive so the two
    /// buttons can show progress independently, and disables both — the
    /// exports directory holds one archive at a time.
    @State private var exportingName: String?

    init(sessionURL: URL, preset: String = "quality") {
        _model = State(initialValue: ProcessingViewModel(
            sessionURL: sessionURL, preset: preset))
    }

    var body: some View {
        List {
            if model.thermalPaused {
                Section {
                    Label("Paused — iPhone is cooling down",
                          systemImage: "thermometer.high")
                        .foregroundStyle(.orange)
                }
            }

            Section {
                ProgressView(value: Double(model.totalProgress)) {
                    Text(model.isRunning ? "Reconstructing…" : statusTitle)
                        .font(.headline)
                }
                HStack(spacing: 16) {
                    Label("\(model.registered)/\(model.imagesTotal)",
                          systemImage: "camera")
                    Label("\(model.points)", systemImage: "circle.grid.3x3")
                    if model.rmse > 0 {
                        Label(String(format: "%.2f px", model.rmse),
                              systemImage: "scope")
                    }
                }
                .font(.caption.monospacedDigit())
                .foregroundStyle(.secondary)
            }

            Section("Stages") {
                ForEach(ProcessingViewModel.stages, id: \.id) { stage in
                    HStack {
                        stageIcon(for: stage.id)
                        Text(stage.label)
                            .foregroundStyle(
                                stage.id == model.currentStage ? .primary : .secondary)
                    }
                }
            }

            if case .failed(let message) = model.phase {
                Section {
                    Label(message, systemImage: "xmark.octagon.fill")
                        .foregroundStyle(.red)
                        .font(.footnote)
                }
            }

            if model.phase == .done, let report = model.report {
                qualitySection(report)
            }

            if model.phase == .done {
                Section("Export") {
                    let sessionZip = model.sessionURL.lastPathComponent + ".zip"
                    Button {
                        share(folder: model.colmapURL, name: "colmap_export.zip")
                    } label: {
                        exportLabel("Share COLMAP dataset (zip)",
                                    systemImage: "square.and.arrow.up",
                                    name: "colmap_export.zip")
                    }
                    .disabled(exportingName != nil)
                    Button {
                        share(folder: model.sessionURL, name: sessionZip)
                    } label: {
                        exportLabel("Share full session (zip)",
                                    systemImage: "shippingbox",
                                    name: sessionZip)
                    }
                    .disabled(exportingName != nil)
                    if let exportError {
                        Text(exportError)
                            .font(.caption)
                            .foregroundStyle(.red)
                    }
                }
            }

            if model.isRunning {
                Section {
                    Button(role: .destructive) {
                        model.cancel()
                    } label: {
                        Label("Cancel", systemImage: "stop.circle")
                    }
                }
            }
        }
        .navigationTitle("Reconstruction")
        .navigationBarTitleDisplayMode(.inline)
        .onAppear { model.start() }
        .sheet(item: $exportURL) { url in
            ShareSheet(items: [url])
        }
    }

    /// How the scan came out, in the user's terms rather than the solver's.
    ///
    /// This sits directly above the export buttons on purpose: it is the
    /// moment someone decides whether to train on this or go back and rescan,
    /// and that decision is much cheaper to make here than after an hour of
    /// GPU time. So the wording is about the scan ("94% of your photos were
    /// placed") rather than about the optimizer ("reprojection RMSE 0.45 px"),
    /// which is true but not something anyone can act on.
    @ViewBuilder
    private func qualitySection(_ report: SolveReport) -> some View {
        Section("Result") {
            HStack(spacing: 10) {
                Image(systemName: verdictIcon(report.verdict))
                    .font(.title2)
                    .foregroundStyle(verdictColor(report.verdict))
                VStack(alignment: .leading, spacing: 2) {
                    Text(report.headline).font(.headline)
                    Text(report.summary)
                        .font(.caption)
                        .foregroundStyle(.secondary)
                        .fixedSize(horizontal: false, vertical: true)
                }
            }

            ForEach(Array(report.advice.enumerated()), id: \.offset) { _, line in
                Label(line, systemImage: "lightbulb")
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }

            if let height = report.levelCameraHeightM, height > 0 {
                LabeledContent(
                    "Camera height",
                    value: String(format: "%.2f m%@", height,
                                  report.floorMeasured == true
                                    ? " (measured floor)" : " (inferred floor)"))
                    .font(.caption)
            }

            let flagged = report.flaggedImages
            if !flagged.isEmpty {
                NavigationLink {
                    FlaggedImagesView(images: flagged)
                } label: {
                    Label("\(flagged.count) photos worth a look",
                          systemImage: "photo.badge.exclamationmark")
                        .font(.caption)
                }
            }
        }
    }

    private func verdictIcon(_ verdict: SolveReport.Verdict) -> String {
        switch verdict {
        case .good: return "checkmark.seal.fill"
        case .fair: return "exclamationmark.circle.fill"
        case .poor: return "exclamationmark.triangle.fill"
        case .unknown: return "questionmark.circle"
        }
    }

    private func verdictColor(_ verdict: SolveReport.Verdict) -> Color {
        switch verdict {
        case .good: return .green
        case .fair: return .orange
        case .poor: return .red
        case .unknown: return .secondary
        }
    }

    private var statusTitle: String {
        switch model.phase {
        case .done: return "Reconstruction complete"
        case .failed: return "Reconstruction failed"
        default: return "Preparing…"
        }
    }

    @ViewBuilder
    private func stageIcon(for id: Int32) -> some View {
        if id < model.currentStage || model.phase == .done {
            Image(systemName: "checkmark.circle.fill").foregroundStyle(.green)
        } else if id == model.currentStage && model.isRunning {
            ProgressView().controlSize(.small)
        } else {
            Image(systemName: "circle").foregroundStyle(.tertiary)
        }
    }

    /// Half a gigabyte of JPEGs takes real time to archive, so the label
    /// says so rather than the app appearing to have hung.
    @ViewBuilder
    private func exportLabel(_ title: String, systemImage: String,
                             name: String) -> some View {
        if exportingName == name {
            HStack(spacing: 8) {
                ProgressView().controlSize(.small)
                Text("Preparing \(title.hasPrefix("Share COLMAP") ? "dataset" : "session")…")
            }
        } else {
            Label(title, systemImage: systemImage)
        }
    }

    private func share(folder: URL, name: String) {
        exportingName = name
        exportError = nil
        Task {
            defer { exportingName = nil }
            do {
                exportURL = try await ZipExporter.zipDirectory(at: folder,
                                                               name: name)
            } catch {
                exportError = "Export failed: \(error.localizedDescription)"
            }
        }
    }
}

extension URL: @retroactive Identifiable {
    public var id: String { absoluteString }
}

/// UIActivityViewController wrapper (ShareLink can't hand out lazily-built
/// zips as cleanly).
struct ShareSheet: UIViewControllerRepresentable {
    let items: [Any]

    func makeUIViewController(context: Context) -> UIActivityViewController {
        UIActivityViewController(activityItems: items, applicationActivities: nil)
    }

    func updateUIViewController(_ controller: UIActivityViewController,
                                context: Context) {}
}
