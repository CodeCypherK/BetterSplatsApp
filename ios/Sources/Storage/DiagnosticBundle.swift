import Foundation

/// Everything needed to diagnose a session, without the photographs.
///
/// A capture is half a gigabyte because it is 400-odd JPEGs. Almost none of
/// that is needed to work out WHY a reconstruction went wrong — the answer
/// is usually in the calibration, the live poses, the per-frame metadata or
/// the solve's own metrics, and those together are under a megabyte.
///
/// This exists because the first real scan failed at 2 photos placed out of
/// 429 and the session was too large to send anywhere. The engine is
/// developed on Linux against exactly this data, so a bundle this size is
/// the difference between diagnosing a failure and guessing at it.
///
/// Deliberately includes no image data at all, which also makes it safe to
/// share: a room's photographs are personal in a way its camera intrinsics
/// are not.
enum DiagnosticBundle {
    enum BundleError: LocalizedError {
        case noSession(String)
        var errorDescription: String? {
            switch self {
            case .noSession(let why): return why
            }
        }
    }

    /// Builds the bundle and returns a zip ready to share. Runs off the main
    /// actor — it reads several hundred small files.
    static func build(sessionURL: URL) async throws -> URL {
        try await withCheckedThrowingContinuation { continuation in
            DispatchQueue.global(qos: .userInitiated).async {
                continuation.resume(with: Result { try assemble(sessionURL) })
            }
        }
    }

    private static func assemble(_ sessionURL: URL) throws -> URL {
        let fm = FileManager.default
        guard fm.fileExists(atPath: sessionURL.path) else {
            throw BundleError.noSession("That session is no longer on disk.")
        }
        let name = sessionURL.lastPathComponent
        let staging = fm.temporaryDirectory
            .appendingPathComponent("diagnostics", isDirectory: true)
        try? fm.removeItem(at: staging)
        let root = staging.appendingPathComponent(name, isDirectory: true)
        try fm.createDirectory(at: root, withIntermediateDirectories: true)

        // Whole files, copied as-is. Missing ones are simply absent from the
        // bundle — their absence is itself diagnostic (no calibration.json
        // means the session never stored a frame).
        for relative in ["session.json",
                         "calibration.json",
                         "live/poses.jsonl",
                         "live/poses_scout.jsonl",
                         "final/report.json"] {
            let src = sessionURL.appendingPathComponent(relative)
            guard fm.fileExists(atPath: src.path) else { continue }
            let dst = root.appendingPathComponent(relative)
            try? fm.createDirectory(at: dst.deletingLastPathComponent(),
                                    withIntermediateDirectories: true)
            try? fm.copyItem(at: src, to: dst)
        }

        // Every frame's meta.json, concatenated one per line. Individually
        // they are a few hundred bytes; as 400-odd files in 400-odd
        // directories they are painful to move, and as one JSONL file they
        // are a table anything can read.
        let framesDir = sessionURL.appendingPathComponent("frames")
        let frames = ((try? fm.contentsOfDirectory(atPath: framesDir.path)) ?? [])
            .sorted()
        var meta = Data()
        var withImage = 0
        var withDepth = 0
        for frame in frames {
            let dir = framesDir.appendingPathComponent(frame)
            if fm.fileExists(atPath: dir.appendingPathComponent("image.jpg").path) {
                withImage += 1
            }
            if fm.fileExists(atPath: dir.appendingPathComponent("lidar.depth").path) {
                withDepth += 1
            }
            guard let line = try? Data(
                contentsOf: dir.appendingPathComponent("meta.json")) else { continue }
            // Flatten to one line so the file stays newline-delimited even if
            // the writer ever pretty-prints.
            let flat = String(decoding: line, as: UTF8.self)
                .replacingOccurrences(of: "\n", with: "")
                .replacingOccurrences(of: "\r", with: "")
            meta.append(Data((flat + "\n").utf8))
        }
        try? meta.write(to: root.appendingPathComponent("frames_meta.jsonl"))

        // What is on disk, against what the metadata claims. A frame
        // directory holding an image but no meta.json is an interrupted
        // write; the counts make that visible without shipping the frames.
        let inventory = """
        {
          "session": "\(name)",
          "frame_directories": \(frames.count),
          "with_image": \(withImage),
          "with_depth": \(withDepth),
          "meta_lines": \(meta.isEmpty ? 0 : String(decoding: meta, as: UTF8.self)
                            .split(separator: "\n").count),
          "app_version": "\(Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String ?? "?")",
          "engine": "\(CoreEngine.version)",
          "device": "\(deviceModel())",
          "ios": "\(ProcessInfo.processInfo.operatingSystemVersionString)"
        }
        """
        try? Data(inventory.utf8)
            .write(to: root.appendingPathComponent("inventory.json"))

        var result: Result<URL, Error> =
            .failure(BundleError.noSession("Could not package the bundle."))
        var coordinatorError: NSError?
        NSFileCoordinator().coordinate(
            readingItemAt: root, options: .forUploading,
            error: &coordinatorError) { zipped in
            do {
                let dest = staging
                    .appendingPathComponent("\(name)-diagnostics.zip")
                try? fm.removeItem(at: dest)
                try fm.copyItem(at: zipped, to: dest)
                result = .success(dest)
            } catch {
                result = .failure(error)
            }
        }
        if let coordinatorError {
            throw BundleError.noSession(coordinatorError.localizedDescription)
        }
        return try result.get()
    }

    /// Hardware identifier ("iPhone17,1"), which is what actually determines
    /// the camera; the marketing name does not.
    private static func deviceModel() -> String {
        var info = utsname()
        uname(&info)
        return withUnsafePointer(to: &info.machine) { ptr in
            ptr.withMemoryRebound(to: CChar.self,
                                  capacity: MemoryLayout.size(ofValue: info.machine)) {
                String(cString: $0)
            }
        }
    }
}
