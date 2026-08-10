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
/// archiver needed). The returned URL lives in the temporary directory.
enum ZipExporter {
    enum ZipError: Error { case coordinationFailed(String) }

    static func zipDirectory(at source: URL, name: String) throws -> URL {
        var result: Result<URL, Error> =
            .failure(ZipError.coordinationFailed("not started"))
        var coordinatorError: NSError?
        let coordinator = NSFileCoordinator()
        coordinator.coordinate(
            readingItemAt: source, options: .forUploading,
            error: &coordinatorError) { zipped in
            do {
                let dest = FileManager.default.temporaryDirectory
                    .appendingPathComponent(name)
                try? FileManager.default.removeItem(at: dest)
                try FileManager.default.copyItem(at: zipped, to: dest)
                result = .success(dest)
            } catch {
                result = .failure(error)
            }
        }
        if let coordinatorError {
            throw ZipError.coordinationFailed(coordinatorError.localizedDescription)
        }
        return try result.get()
    }
}
