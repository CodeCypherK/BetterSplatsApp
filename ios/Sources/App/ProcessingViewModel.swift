import Foundation
import Observation
import UIKit

/// Drives the on-device final reconstruction: engine worker thread via the
/// C ABI, progress polling, thermal hints, and a background task so the
/// solve survives brief app suspensions.
@Observable
@MainActor
final class ProcessingViewModel {
    enum Phase: Equatable {
        case idle
        case running
        case done
        case failed(String)
    }

    struct StageInfo {
        let id: Int32
        let label: String
    }

    static let stages: [StageInfo] = [
        .init(id: 1, label: "Detecting features"),
        .init(id: 3, label: "Selecting image pairs"),
        .init(id: 4, label: "Matching and verifying"),
        .init(id: 5, label: "Building feature tracks"),
        .init(id: 6, label: "Initializing camera poses"),
        .init(id: 7, label: "Triangulating points"),
        .init(id: 8, label: "Global bundle adjustment"),
        .init(id: 9, label: "Removing floaters"),
        .init(id: 10, label: "Aligning LiDAR (final poses)"),
        .init(id: 11, label: "Exporting COLMAP model"),
    ]

    private(set) var phase: Phase = .idle
    private(set) var currentStage: Int32 = 0
    private(set) var totalProgress: Float = 0
    private(set) var registered: UInt32 = 0
    private(set) var imagesTotal: UInt32 = 0
    private(set) var points: UInt32 = 0
    private(set) var rmse: Float = 0
    private(set) var thermalPaused = false
    /// Read once when the solve finishes. Everything the engine measured
    /// about the reconstruction lives in final/report.json, and this is the
    /// only thing that ever looks at it.
    private(set) var report: SolveReport?

    let sessionURL: URL
    let preset: String

    private var pollTask: Task<Void, Never>?
    private var backgroundTask: UIBackgroundTaskIdentifier = .invalid

    init(sessionURL: URL, preset: String = "quality") {
        self.sessionURL = sessionURL
        self.preset = preset
    }

    var isRunning: Bool { phase == .running }

    func start() {
        guard phase != .running else { return }
        let engine = CoreEngine.shared
        guard engine.finalStart(sessionDir: sessionURL.path, preset: preset)
            == BS_OK else {
            phase = .failed(engine.lastError)
            return
        }
        phase = .running
        UIApplication.shared.isIdleTimerDisabled = true
        backgroundTask = UIApplication.shared.beginBackgroundTask(
            withName: "final-solve") { [weak self] in
            Task { @MainActor in self?.endBackgroundTask() }
        }

        pollTask?.cancel()
        pollTask = Task { [weak self] in
            while !Task.isCancelled {
                try? await Task.sleep(for: .milliseconds(300))
                guard let self else { return }
                self.pushThermalState()
                let progress = CoreEngine.shared.finalPoll()
                self.currentStage = progress.stage
                self.totalProgress = progress.total_progress
                self.registered = progress.images_registered
                self.imagesTotal = progress.images_total
                self.points = progress.points
                self.rmse = progress.reproj_rmse_px
                self.thermalPaused = progress.paused_thermal != 0

                if progress.stage == Int32(BS_STAGE_DONE.rawValue) {
                    self.finish(.done)
                    return
                }
                if progress.stage == Int32(BS_STAGE_FAILED.rawValue) {
                    self.finish(.failed(CoreEngine.shared.lastError))
                    return
                }
                if progress.running == 0,
                   progress.stage == Int32(BS_STAGE_IDLE.rawValue) {
                    self.finish(.idle)  // cancelled
                    return
                }
            }
        }
    }

    func cancel() {
        CoreEngine.shared.finalCancel()
    }

    private func pushThermalState() {
        let level: Int32
        switch ProcessInfo.processInfo.thermalState {
        case .nominal: level = 0
        case .fair: level = 1
        case .serious: level = 2
        case .critical: level = 3
        @unknown default: level = 1
        }
        CoreEngine.shared.thermalHint(level: level)
    }

    private func finish(_ result: Phase) {
        phase = result
        if result == .done { report = SolveReport.read(sessionURL: sessionURL) }
        pollTask?.cancel()
        pollTask = nil
        UIApplication.shared.isIdleTimerDisabled = false
        endBackgroundTask()
    }

    private func endBackgroundTask() {
        if backgroundTask != .invalid {
            UIApplication.shared.endBackgroundTask(backgroundTask)
            backgroundTask = .invalid
        }
    }

    var colmapURL: URL {
        sessionURL.appendingPathComponent("final/colmap")
    }

    var hasColmapExport: Bool {
        FileManager.default.fileExists(
            atPath: colmapURL.appendingPathComponent("points3D.txt").path)
    }
}

/// Folder -> zip using the file-coordination conversion (no third-party
/// archiver needed).
///
/// Everything about this runs OFF the main actor, and that is the point. A
/// finished capture is 200-500 JPEGs — comfortably half a gigabyte — and the
/// old version zipped it synchronously from a SwiftUI button action. The app
/// froze solid for as long as it took, with no progress and no way to
/// cancel, at the one moment the user is being handed the thing they walked
/// the room for. Long enough and the watchdog kills the app outright.
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
                     + "space, or share the COLMAP dataset on its own."
            }
        }
    }

    /// Where finished zips live. A directory of our own, so the previous
    /// export can be cleared before the next one: these are whole copies of
    /// a capture, and iOS empties the temporary directory on its own
    /// schedule rather than ours. Three exports used to mean three copies
    /// sitting on a phone the app had already warned was nearly full.
    private static var exportsDir: URL {
        FileManager.default.temporaryDirectory
            .appendingPathComponent("exports", isDirectory: true)
    }

    /// Builds the zip on a background queue. The URL lives under
    /// `exportsDir` and is superseded by the next export.
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

        // JPEG and LZ4 depth are already compressed, so the archive is about
        // the size of the source — and the coordinator builds its own copy
        // before we take ours, so budget for two.
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
