import SwiftUI

/// On-device final reconstruction screen: staged progress, live metrics,
/// thermal banner, cancel, and the export handoff when done.
struct ProcessingView: View {
    @State private var model: ProcessingViewModel
    @State private var exportURL: URL?
    @State private var exportError: String?

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

            if model.phase == .done {
                Section("Export") {
                    Button {
                        share(folder: model.colmapURL, name: "colmap_export.zip")
                    } label: {
                        Label("Share COLMAP dataset (zip)",
                              systemImage: "square.and.arrow.up")
                    }
                    Button {
                        share(folder: model.sessionURL,
                              name: model.sessionURL.lastPathComponent + ".zip")
                    } label: {
                        Label("Share full session (zip)",
                              systemImage: "shippingbox")
                    }
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

    private func share(folder: URL, name: String) {
        do {
            exportURL = try ZipExporter.zipDirectory(at: folder, name: name)
            exportError = nil
        } catch {
            exportError = "Export failed: \(error.localizedDescription)"
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
