import SwiftUI

/// Lists captured sessions from Documents. Sessions are also visible in the
/// Files app (On My iPhone → BetterSplats) — that's the recommended way to
/// AirDrop a full session to a computer for replay until in-app export
/// arrives with the final-solve UI.
struct SessionsView: View {
    struct Entry: Identifiable {
        let id: String
        let url: URL
        let frames: Int
        let megabytes: Double
        let created: Date?
    }

    @State private var entries: [Entry] = []

    var body: some View {
        List {
            if entries.isEmpty {
                ContentUnavailableView(
                    "No sessions yet",
                    systemImage: "camera.metering.matrix",
                    description: Text("Captured sessions appear here and in the Files app."))
            }
            ForEach(entries) { entry in
                VStack(alignment: .leading, spacing: 4) {
                    Text(entry.id)
                        .font(.subheadline.weight(.medium))
                        .lineLimit(1)
                    HStack {
                        Label("\(entry.frames) frames", systemImage: "photo.stack")
                        Label(String(format: "%.0f MB", entry.megabytes),
                              systemImage: "internaldrive")
                    }
                    .font(.caption)
                    .foregroundStyle(.secondary)
                }
            }
            .onDelete(perform: delete)
        }
        .navigationTitle("Sessions")
        .onAppear(perform: reload)
        .refreshable { reload() }
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
                return Entry(
                    id: url.lastPathComponent, url: url, frames: frames,
                    megabytes: Double(size) / 1_048_576.0, created: created)
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
