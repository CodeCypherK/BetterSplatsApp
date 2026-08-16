import SwiftUI

/// Captured sessions: share the RAW capture, delete.
/// Sessions are also visible in the Files app (On My iPhone → BetterSplats)
/// — copy them off the phone regularly; reconstruction runs on the desktop.
///
/// Layout note, learned from a device: rows must contain **at most one**
/// navigation target. The first version put several `NavigationLink`s plus
/// share buttons in one `List` row, which SwiftUI turns into a row-sized
/// tap target — so tapping a share button pushed another screen.
struct SessionsView: View {
    struct Entry: Identifiable {
        let id: String
        let url: URL
        let frames: Int
        let megabytes: Double
        let created: Date?
    }

    @State private var entries: [Entry] = []
    @State private var shareURL: URL?
    @State private var shareError: String?
    @State private var isSharing = false
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
        .onAppear(perform: reload)
        .refreshable { reload() }
        .sheet(item: $shareURL) { url in
            ShareSheet(items: [url])
        }
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
            Text(entry.id)
                .font(.subheadline.weight(.medium))
                .lineLimit(1)
                .truncationMode(.middle)

            Text(subtitle(entry))
                .font(.caption)
                .foregroundStyle(.secondary)

            HStack(spacing: 12) {
                Button {
                    share(folder: entry.url, name: entry.id + ".zip")
                } label: {
                    Label("Share session", systemImage: "square.and.arrow.up")
                }
                .buttonStyle(.borderedProminent)
                .controlSize(.small)
                .disabled(isSharing)

                Spacer(minLength: 0)

                Menu {
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
                return Entry(
                    id: url.lastPathComponent, url: url, frames: frames,
                    megabytes: Double(size) / 1_048_576.0, created: created)
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
