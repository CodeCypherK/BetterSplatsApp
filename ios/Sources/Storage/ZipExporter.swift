import Foundation
import SwiftUI
import UIKit

/// Folder -> zip using the file-coordination conversion (no third-party
/// archiver needed).
///
/// Everything about this runs OFF the main actor, and that is the point. A
/// finished capture is 200-500 JPEGs — comfortably half a gigabyte — and
/// zipping it from a SwiftUI button action freezes the app for as long as
/// it takes.
enum ZipExporter {
    enum ZipError: LocalizedError {
        case coordinationFailed(String)
        case notEnoughSpace(needBytes: Int64, freeBytes: Int64)

        var errorDescription: String? {
            switch self {
            case .coordinationFailed(let why):
                return why
            case .notEnoughSpace(let need, let free):
                let gb = { (b: Int64) in
                    String(format: "%.1f GB", Double(b) / 1_073_741_824)
                }
                return "Not enough free space — this export needs about "
                     + "\(gb(need)) and there is \(gb(free)). Free some "
                     + "space and try again."
            }
        }
    }

    /// Where finished zips live. A directory of our own, so the previous
    /// export can be cleared before the next one.
    private static var exportsDir: URL {
        FileManager.default.temporaryDirectory
            .appendingPathComponent("exports", isDirectory: true)
    }

    static func zipDirectory(at source: URL, name: String) async throws -> URL {
        try await withCheckedThrowingContinuation { continuation in
            DispatchQueue.global(qos: .userInitiated).async {
                continuation.resume(with: Result {
                    try zipSync(at: source, name: name)
                })
            }
        }
    }

    private static func zipSync(at source: URL, name: String) throws -> URL {
        let fm = FileManager.default

        let sourceBytes = directorySize(source)
        let free = (try? URL(fileURLWithPath: NSHomeDirectory())
            .resourceValues(forKeys: [.volumeAvailableCapacityForImportantUsageKey])
            .volumeAvailableCapacityForImportantUsage) ?? nil
        if let free, sourceBytes > 0, free < sourceBytes * 2 {
            throw ZipError.notEnoughSpace(needBytes: sourceBytes * 2,
                                          freeBytes: free)
        }

        try? fm.removeItem(at: exportsDir)
        try fm.createDirectory(at: exportsDir, withIntermediateDirectories: true)

        var result: Result<URL, Error> =
            .failure(ZipError.coordinationFailed("not started"))
        var coordinatorError: NSError?
        let coordinator = NSFileCoordinator()
        coordinator.coordinate(
            readingItemAt: source, options: .forUploading,
            error: &coordinatorError) { zipped in
            do {
                let dest = exportsDir.appendingPathComponent(name)
                try fm.copyItem(at: zipped, to: dest)
                result = .success(dest)
            } catch {
                result = .failure(error)
            }
        }
        if let coordinatorError {
            throw ZipError.coordinationFailed(
                coordinatorError.localizedDescription)
        }
        return try result.get()
    }

    private static func directorySize(_ url: URL) -> Int64 {
        let keys: [URLResourceKey] = [.totalFileAllocatedSizeKey, .fileSizeKey]
        guard let e = FileManager.default.enumerator(
            at: url, includingPropertiesForKeys: keys) else { return 0 }
        var total: Int64 = 0
        for case let file as URL in e {
            let values = try? file.resourceValues(forKeys: Set(keys))
            total += Int64(values?.totalFileAllocatedSize
                           ?? values?.fileSize ?? 0)
        }
        return total
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
