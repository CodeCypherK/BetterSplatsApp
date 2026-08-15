import SwiftUI

/// Captured sessions: reconstruct on-device, share exports, delete.
/// Sessions are also visible in the Files app (On My iPhone → BetterSplats)
/// — copy finished work off the phone regularly, especially on a free
/// Apple ID where deleting the app deletes its data.
///
/// Layout note, learned from a device: rows must contain **at most one**
/// navigation target. The first version put two `NavigationLink`s (Quality
/// and Fast) plus two share buttons in one `List` row, which SwiftUI turns
/// into a row-sized tap target for the links — so tapping anywhere on the
/// row, including the share buttons, pushed a reconstruction screen, and
/// going back revealed the *second* push underneath. Four bordered controls
/// crammed into one caption-sized row also left the card looking squashed.
///
/// So: the row is inert, the primary action is one obvious button, and
/// everything else lives in a menu where each item can afford a full label.
struct SessionsView: View {
    struct Entry: Identifiable {
        let id: String
        let url: URL
        let frames: Int
        let megabytes: Double
        let created: Date?
        let hasColmap: Bool
    }

    struct SolveRequest: Hashable {
        let path: String
        let preset: String
    }

    @State private var entries: [Entry] = []
    @State private var shareURL: URL?
    @State private var shareError: String?
    @State private var isSharing = false
    @State private var solveRequest: SolveRequest?
    @State private var pendingDelete: Entry?

    var body: some View {
        List {
            if entries.isEmpty {
                ContentUnavailableView(
                    "No sessions yet",
                    systemImage: "camera.metering.matrix",
                    description: Text("Captured sessions appear here and in the Files app."))
            }
            ForEach(entries) { entry in
                row(entry)
                    .swipeActions(edge: .trailing, allowsFullSwipe: false) {
                        Button(role: .destructive) {
                            pendingDelete = entry
                        } label: {
                            Label("Delete", systemImage: "trash")
                        }
                    }
            }

            if let shareError {
                Text(shareError)
                    .font(.caption)
                    .foregroundStyle(.red)
            }
        }
        .navigationTitle("Sessions")
        .navigationDestination(item: $solveRequest) { request in
            ProcessingView(sessionURL: URL(fileURLWithPath: request.path),
                           preset: request.preset)
        }
        .onAppear(perform: reload)
        .refreshable { reload() }
        .sheet(item: $shareURL) { url in
            ShareSheet(items: [url])
        }
        // A capture is minutes of walking and cannot be recovered, so the
        // delete is confirmed and says exactly what is going away.
        .confirmationDialog(
            "Delete this capture?",
            isPresented: Binding(get: { pendingDelete != nil },
                                 set: { if !$0 { pendingDelete = nil } }),
            titleVisibility: .visible,
            presenting: pendingDelete
        ) { entry in
            Button("Delete \(entry.frames) photos", role: .destructive) {
                delete(entry)
                pendingDelete = nil
            }
            Button("Keep", role: .cancel) { pendingDelete = nil }
        } message: { entry in
            Text("\(entry.id)\n\(String(format: "%.0f MB", entry.megabytes)) "
                 + "will be removed from this iPhone. This cannot be undone.")
        }
    }

    @ViewBuilder
    private func row(_ entry: Entry) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack(spacing: 6) {
                Text(entry.id)
                    .font(.subheadline.weight(.medium))
                    .lineLimit(1)
                    .truncationMode(.middle)
                if entry.hasColmap {
                    Image(systemName: "checkmark.seal.fill")
                        .foregroundStyle(.green)
                        .font(.caption)
                        .accessibilityLabel("Reconstructed")
                }
            }

            Text(subtitle(entry))
                .font(.caption)
                .foregroundStyle(.secondary)

            HStack(spacing: 12) {
                Button {
                    solveRequest = SolveRequest(path: entry.url.path,
                                                preset: "quality")
                } label: {
                    Label(entry.hasColmap ? "Re-solve" : "Reconstruct",
                          systemImage: "gearshape.2")
                }
                .buttonStyle(.borderedProminent)
                .controlSize(.small)

                Spacer(minLength: 0)

                Menu {
                    Button {
                        solveRequest = SolveRequest(path: entry.url.path,
                                                    preset: "fast")
                    } label: {
                        Label("Reconstruct (fast preset)", systemImage: "hare")
                    }
                    if entry.hasColmap {
                        Button {
                            share(folder: entry.url
                                .appendingPathComponent("final/colmap"),
                                name: "colmap_export.zip")
                        } label: {
                            Label("Share COLMAP dataset",
                                  systemImage: "square.and.arrow.up")
                        }
                    }
                    Button {
                        share(folder: entry.url, name: entry.id + ".zip")
                    } label: {
                        Label("Share full session", systemImage: "shippingbox")
                    }
                    // Under a megabyte, and it is what actually diagnoses a
                    // failed reconstruction. The full session is half a
                    // gigabyte of JPEGs and routinely too large to send
                    // anywhere, which is how a real failure came to be
                    // debugged by guesswork.
                    Button {
                        shareDiagnostics(entry)
                    } label: {
                        Label("Share diagnostics (small)",
                              systemImage: "stethoscope")
                    }
                    Divider()
                    Button(role: .destructive) {
                        pendingDelete = entry
                    } label: {
                        Label("Delete capture", systemImage: "trash")
                    }
                } label: {
                    Label("More", systemImage: "ellipsis.circle")
                        .labelStyle(.iconOnly)
                        .font(.title3)
                }
                .disabled(isSharing)
            }

            if isSharing {
                // Archiving a capture is half a gigabyte of work. Saying so
                // beats the app looking hung.
                HStack(spacing: 6) {
                    ProgressView().controlSize(.mini)
                    Text("Preparing export…")
                }
                .font(.caption)
                .foregroundStyle(.secondary)
            }
        }
        .padding(.vertical, 4)
    }

    private func subtitle(_ entry: Entry) -> String {
        var parts = ["\(entry.frames) photos",
                     String(format: "%.0f MB", entry.megabytes)]
        if let created = entry.created {
            parts.append(created.formatted(date: .abbreviated,
                                           time: .shortened))
        }
        return parts.joined(separator: " · ")
    }

    private func share(folder: URL, name: String) {
        isSharing = true
        shareError = nil
        // @MainActor explicitly: `share` is a nonisolated method, so a
        // bare Task has no isolation to inherit and would touch @State
        // off the main actor.
        Task { @MainActor in
            defer { isSharing = false }
            do {
                shareURL = try await ZipExporter.zipDirectory(at: folder,
                                                              name: name)
            } catch {
                shareError = "Export failed: \(error.localizedDescription)"
            }
        }
    }

    /// Scanning sessions means stat-ing every file in every one of them —
    /// ten captures of 500 frames is fifteen thousand syscalls — so it does
    /// not happen on the main thread while the screen is trying to draw.
    /// The no-photographs bundle: calibration, live poses, every frame's
    /// metadata and the solve's own metrics.
    private func shareDiagnostics(_ entry: Entry) {
        isSharing = true
        shareError = nil
        Task { @MainActor in
            defer { isSharing = false }
            do {
                shareURL = try await DiagnosticBundle.build(
                    sessionURL: entry.url)
            } catch {
                shareError = "Could not build diagnostics: "
                           + error.localizedDescription
            }
        }
    }

    private func reload() {
        Task { @MainActor in
            let found = await Task.detached(priority: .userInitiated) {
                Self.scan()
            }.value
            entries = found
        }
    }

    nonisolated private static func scan() -> [Entry] {
        let fm = FileManager.default
        let docs = fm.urls(for: .documentDirectory, in: .userDomainMask)[0]
        let dirs = (try? fm.contentsOfDirectory(
            at: docs, includingPropertiesForKeys: [.creationDateKey])) ?? []
        return dirs
            .filter { $0.lastPathComponent.hasPrefix("session_") }
            .map { url in
                let frames = (try? fm.contentsOfDirectory(
                    at: url.appendingPathComponent("frames"),
                    includingPropertiesForKeys: nil))?.count ?? 0
                let size = Self.directorySize(url)
                let created = try? url.resourceValues(forKeys: [.creationDateKey])
                    .creationDate
                let hasColmap = fm.fileExists(
                    atPath: url.appendingPathComponent(
                        "final/colmap/points3D.txt").path)
                return Entry(
                    id: url.lastPathComponent, url: url, frames: frames,
                    megabytes: Double(size) / 1_048_576.0, created: created,
                    hasColmap: hasColmap)
            }
            .sorted { ($0.created ?? .distantPast) > ($1.created ?? .distantPast) }
    }

    private func delete(_ entry: Entry) {
        try? FileManager.default.removeItem(at: entry.url)
        reload()
    }

    nonisolated private static func directorySize(_ url: URL) -> Int64 {
        let fm = FileManager.default
        guard let enumerator = fm.enumerator(
            at: url, includingPropertiesForKeys: [.fileSizeKey]) else { return 0 }
        var total: Int64 = 0
        for case let file as URL in enumerator {
            total += Int64((try? file.resourceValues(forKeys: [.fileSizeKey])
                .fileSize) ?? 0)
        }
        return total
    }
}
