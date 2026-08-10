import SwiftUI

/// Captured sessions: reconstruct on-device, share exports, delete.
/// Sessions are also visible in the Files app (On My iPhone → BetterSplats)
/// — copy finished work off the phone regularly, especially on a free
/// Apple ID where deleting the app deletes its data.
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

    var body: some View {
        List {
            if entries.isEmpty {
                ContentUnavailableView(
                    "No sessions yet",
                    systemImage: "camera.metering.matrix",
                    description: Text("Captured sessions appear here and in the Files app."))
            }
            ForEach(entries) { entry in
                VStack(alignment: .leading, spacing: 6) {
                    HStack {
                        Text(entry.id)
                            .font(.subheadline.weight(.medium))
                            .lineLimit(1)
                        if entry.hasColmap {
                            Image(systemName: "checkmark.seal.fill")
                                .foregroundStyle(.green)
                                .font(.caption)
                        }
                    }
                    HStack {
                        Label("\(entry.frames) frames", systemImage: "photo.stack")
                        Label(String(format: "%.0f MB", entry.megabytes),
                              systemImage: "internaldrive")
                    }
                    .font(.caption)
                    .foregroundStyle(.secondary)

                    HStack(spacing: 10) {
                        NavigationLink(
                            value: SolveRequest(path: entry.url.path,
                                                preset: "quality")) {
                            Label(entry.hasColmap ? "Re-solve" : "Reconstruct",
                                  systemImage: "gearshape.2")
                        }
                        NavigationLink(
                            value: SolveRequest(path: entry.url.path,
                                                preset: "fast")) {
                            Label("Fast", systemImage: "hare")
                        }
                        if entry.hasColmap {
                            Button {
                                share(folder: entry.url
                                    .appendingPathComponent("final/colmap"),
                                    name: "colmap_export.zip")
                            } label: {
                                Label("COLMAP", systemImage: "square.and.arrow.up")
                            }
                        }
                        Button {
                            share(folder: entry.url,
                                  name: entry.id + ".zip")
                        } label: {
                            Label("Session", systemImage: "shippingbox")
                        }
                    }
                    .font(.caption)
                    .buttonStyle(.bordered)
                    .controlSize(.small)
                }
                .padding(.vertical, 2)
            }
            .onDelete(perform: delete)

            if let shareError {
                Text(shareError)
                    .font(.caption)
                    .foregroundStyle(.red)
            }
        }
        .navigationTitle("Sessions")
        .navigationDestination(for: SolveRequest.self) { request in
            ProcessingView(sessionURL: URL(fileURLWithPath: request.path),
                           preset: request.preset)
        }
        .onAppear(perform: reload)
        .refreshable { reload() }
        .sheet(item: $shareURL) { url in
            ShareSheet(items: [url])
        }
    }

    private func share(folder: URL, name: String) {
        do {
            shareURL = try ZipExporter.zipDirectory(at: folder, name: name)
            shareError = nil
        } catch {
            shareError = "Export failed: \(error.localizedDescription)"
        }
    }

    private func reload() {
        let fm = FileManager.default
        let docs = fm.urls(for: .documentDirectory, in: .userDomainMask)[0]
        let dirs = (try? fm.contentsOfDirectory(
            at: docs, includingPropertiesForKeys: [.creationDateKey])) ?? []
        entries = dirs
            .filter { $0.lastPathComponent.hasPrefix("session_") }
            .map { url in
                let frames = (try? fm.contentsOfDirectory(
                    at: url.appendingPathComponent("frames"),
                    includingPropertiesForKeys: nil))?.count ?? 0
                let size = directorySize(url)
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

    private func delete(at offsets: IndexSet) {
        for index in offsets {
            try? FileManager.default.removeItem(at: entries[index].url)
        }
        reload()
    }

    private func directorySize(_ url: URL) -> Int64 {
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
