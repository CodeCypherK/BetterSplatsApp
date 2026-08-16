import Foundation

/// Projects: a space captured over several sittings, reopenable.
///
/// There is no project file. A project IS the chain of captures that share a
/// `project_id`, ordered by `parent_session`, and this type reconstructs that
/// grouping by reading the session documents on disk. That is deliberate: a
/// separate index would be a second source of truth that can disagree with
/// the sessions, and the sessions are the thing that survives being zipped up
/// and moved to another machine. Delete a capture in the Files app and the
/// project simply has one fewer capture, rather than a dangling index entry.
struct ProjectStore {
    struct Capture: Identifiable {
        let directory: URL
        let sessionName: String
        let createdUtc: String
        let frameCount: UInt32
        let parentSession: String
        let supersedes: [SessionJSON.SupersedeJSON]

        var id: String { sessionName }
        var date: Date? { ISO8601DateFormatter().date(from: createdUtc) }
    }

    struct Project: Identifiable {
        let id: String
        var name: String
        /// Oldest first — capture order, which is also chain order.
        var captures: [Capture]

        var frameCount: UInt32 { captures.reduce(0) { $0 + $1.frameCount } }
        var latestSessionName: String { captures.last?.sessionName ?? "" }
        var latestDirectory: URL? { captures.last?.directory }
        var date: Date? { captures.first?.date }
        var rescanCount: Int { captures.reduce(0) { $0 + $1.supersedes.count } }

        /// Highest frame id written anywhere in the project, so the next
        /// capture can carry on from it rather than colliding.
        ///
        /// Read from the directory contents, not from `frame_count`: a
        /// capture killed before finalize leaves frame_count at 0 with frames
        /// on disk, and continuing from 0 would duplicate every id it wrote.
        var lastFrameId: UInt32 {
            var highest: UInt32 = 0
            for capture in captures {
                let frames = capture.directory.appendingPathComponent("frames")
                let names = (try? FileManager.default.contentsOfDirectory(
                    atPath: frames.path)) ?? []
                for name in names {
                    if let id = UInt32(name), id > highest { highest = id }
                }
            }
            return highest
        }
    }

    /// Every project under Documents, newest first.
    ///
    /// Captures with no `project_id` — written before projects existed — are
    /// each returned as a single-capture project so nothing becomes
    /// invisible. Losing sight of a scan because the schema moved on would be
    /// far worse than showing it slightly out of context.
    static func load() -> [Project] {
        let fm = FileManager.default
        let root = SessionStore.documentsDirectory()
        let entries = (try? fm.contentsOfDirectory(
            at: root, includingPropertiesForKeys: nil)) ?? []

        let decoder = JSONDecoder()
        var byProject: [String: Project] = [:]
        var parentOf: [String: [String: String]] = [:]  // project -> child: parent

        for entry in entries {
            guard entry.hasDirectoryPath else { continue }
            let jsonURL = entry.appendingPathComponent("session.json")
            guard let data = try? Data(contentsOf: jsonURL),
                  let doc = try? decoder.decode(SessionJSON.self, from: data)
            else { continue }

            let capture = Capture(
                directory: entry,
                sessionName: entry.lastPathComponent,
                createdUtc: doc.createdUtc,
                frameCount: doc.frameCount,
                parentSession: doc.parentSession ?? "",
                supersedes: doc.supersedes ?? [])

            // A capture from before projects existed stands alone under its
            // own name rather than being dropped.
            let projectId = (doc.projectId ?? "").isEmpty
                ? "solo_\(entry.lastPathComponent)" : doc.projectId!
            let projectName = (doc.projectName ?? "").isEmpty
                ? entry.lastPathComponent : doc.projectName!

            byProject[projectId, default: Project(id: projectId,
                                                  name: projectName,
                                                  captures: [])]
                .captures.append(capture)
            parentOf[projectId, default: [:]][capture.sessionName] =
                doc.parentSession ?? ""
        }

        return byProject.values
            .map { project in
                var ordered = project
                ordered.captures = chainOrder(project.captures,
                                              parents: parentOf[project.id] ?? [:])
                return ordered
            }
            .sorted { ($0.date ?? .distantPast) > ($1.date ?? .distantPast) }
    }

    /// Orders captures oldest-first by following `parent_session`.
    ///
    /// Falls back to creation time for anything the links do not reach — a
    /// project whose middle capture was deleted still lists what remains, in
    /// a sensible order, rather than showing an arbitrary one or nothing.
    private static func chainOrder(
        _ captures: [Capture], parents: [String: String]) -> [Capture] {
        let byName = Dictionary(uniqueKeysWithValues:
            captures.map { ($0.sessionName, $0) })
        // The root is the capture whose parent is absent from this project.
        let roots = captures.filter {
            $0.parentSession.isEmpty || byName[$0.parentSession] == nil
        }
        guard let root = roots.min(by: {
            ($0.date ?? .distantPast) < ($1.date ?? .distantPast)
        }) else {
            return captures.sorted { ($0.date ?? .distantPast) < ($1.date ?? .distantPast) }
        }

        let childOf = Dictionary(
            captures.compactMap { c -> (String, Capture)? in
                c.parentSession.isEmpty ? nil : (c.parentSession, c)
            }, uniquingKeysWith: { a, _ in a })

        var ordered: [Capture] = []
        var seen = Set<String>()
        var current: Capture? = root
        while let capture = current, !seen.contains(capture.sessionName) {
            seen.insert(capture.sessionName)
            ordered.append(capture)
            current = childOf[capture.sessionName]
        }
        // Anything the chain missed (a deleted middle, a hand-edited link)
        // still gets listed rather than silently dropped.
        let missed = captures.filter { !seen.contains($0.sessionName) }
            .sorted { ($0.date ?? .distantPast) < ($1.date ?? .distantPast) }
        return ordered + missed
    }

    /// Renames a project by rewriting `project_name` in every capture. There
    /// is no index to update, which is the point of not having one.
    static func rename(_ project: Project, to name: String) {
        let decoder = JSONDecoder()
        let encoder = JSONEncoder()
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        for capture in project.captures {
            let url = capture.directory.appendingPathComponent("session.json")
            guard let data = try? Data(contentsOf: url),
                  var doc = try? decoder.decode(SessionJSON.self, from: data)
            else { continue }
            doc.projectName = name
            if let out = try? encoder.encode(doc) {
                try? out.write(to: url, options: .atomic)
            }
        }
    }
}
