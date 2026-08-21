import AVFoundation
import CoreMedia
import Foundation
import Observation
import simd
import SwiftUI
import UIKit

final class FramePipeline: @unchecked Sendable {
    let gate = BestFrameGate()
    let poses = PoseTracker()
    let jpeg = JpegEncoder(quality: 0.9)

    init() {
        gate.encodeJPEG = { [jpeg] buffer in
            jpeg.encodeYUV(buffer, orientation: .up)
        }
    }
}

/// Preview → lock exposure → then save ≤1 JPEG/s from 0.5× ultra-wide.
@MainActor
@Observable
final class ScanViewModel {
    enum Phase: Equatable {
        case ready
        case previewing
        case scanning
        case finishing
        case done
        case failed(String)
    }

    var phase: Phase = .ready
    var photoCount = 0
    var megabytes: Double = 0
    var photos: [CapturedPoseSample] = []
    var statusLine = "Open the camera, set exposure, then start saving"
    var exposureLock: ExposureAxisLock = .none
    var exposureLockLabel = "AE auto"

    var continuingProject: ProjectStore.Project?
    var newProjectName: String?

    private var store: ImageSessionStore?
    private let capture = UltraWideCapture()
    private let pipeline = FramePipeline()
    private var saving = false
    private var lockPoll: Timer?
    private var savingEnabled = false

    var previewLayer: AVCaptureVideoPreviewLayer? { capture.previewLayer }

    var showsCamera: Bool {
        switch phase {
        case .previewing, .scanning: return true
        default: return false
        }
    }

    var isScanning: Bool {
        if case .scanning = phase { return true }
        return false
    }

    var isPreviewing: Bool {
        if case .previewing = phase { return true }
        return false
    }

    func fail(_ message: String) {
        phase = .failed(message)
        statusLine = message
    }

    /// Open the ultra-wide feed so exposure can be locked before any saves.
    func openCamera() {
        guard phase == .ready || phase == .done else { return }
        do {
            try capture.start()
            pipeline.poses.start()
            pipeline.gate.reset()
            savingEnabled = false
            photos = []
            photoCount = 0
            megabytes = 0
            store = nil
            exposureLock = .none
            exposureLockLabel = "AE auto"
            // Preview only — exposure steer runs inside UltraWideCapture.
            // Saving attaches the frame gate in beginSaving().
            capture.onFrame = nil
            phase = .previewing
            let dims = capture.dimensions
            statusLine = String(format: "Preview %d×%d — lock ISO/shutter, then Start saving",
                                dims.width, dims.height)
            startLockPoll()
        } catch {
            capture.stop()
            pipeline.poses.stop()
            fail(error.localizedDescription)
        }
    }

    /// Begin accepting ≤1 photo/s into the room folder.
    private var lastStatusUptime: TimeInterval = 0

    func beginSaving() {
        guard phase == .previewing else { return }
        do {
            let dims = capture.dimensions
            let store = try ImageSessionStore(
                osVersion: UIDevice.current.systemVersion,
                continuing: continuingProject,
                newProjectName: newProjectName,
                videoW: dims.width, videoH: dims.height)
            self.store = store
            pipeline.gate.reset()
            savingEnabled = true
            lastStatusUptime = 0
            capture.onFrame = { [weak self, pipeline] buffer, time in
                let pose = pipeline.poses.currentPose()
                guard let accepted = pipeline.gate.consider(
                    pixelBuffer: buffer, time: time, pose: pose)
                else {
                    let reason = pipeline.gate.lastSkipReason
                    let now = ProcessInfo.processInfo.systemUptime
                    Task { @MainActor [weak self] in
                        guard let self else { return }
                        if now - self.lastStatusUptime >= 0.5 {
                            self.lastStatusUptime = now
                            self.statusLine = reason
                        }
                    }
                    return
                }
                Task { @MainActor [weak self] in
                    await self?.saveAccepted(accepted)
                }
            }
            phase = .scanning
            statusLine = "Saving… move or turn for new views"
        } catch {
            fail(error.localizedDescription)
        }
    }

    func stop() {
        guard showsCamera else { return }
        let wasSaving = isScanning
        phase = .finishing
        lockPoll?.invalidate()
        lockPoll = nil
        savingEnabled = false
        capture.onFrame = nil
        capture.stop()
        pipeline.poses.stop()
        Task {
            if wasSaving {
                do {
                    try await store?.finalize()
                    phase = .done
                    statusLine = "Saved \(photoCount) photos"
                } catch {
                    fail(error.localizedDescription)
                }
            } else {
                phase = .ready
                statusLine = "Open the camera, set exposure, then start saving"
            }
        }
    }

    func lockISO() {
        capture.lockISO()
        refreshExposureLabel()
    }

    func lockShutter() {
        capture.lockShutter()
        refreshExposureLabel()
    }

    func unlockExposure() {
        capture.unlockExposure()
        refreshExposureLabel()
    }

    private func startLockPoll() {
        lockPoll?.invalidate()
        lockPoll = Timer.scheduledTimer(withTimeInterval: 0.35, repeats: true) { [weak self] _ in
            Task { @MainActor in self?.refreshExposureLabel() }
        }
    }

    private func refreshExposureLabel() {
        exposureLock = capture.exposureAxisLock
        switch exposureLock {
        case .none:
            exposureLockLabel = "AE auto"
        case .iso:
            if let iso = capture.lockedISOValue {
                exposureLockLabel = String(format: "ISO locked %.0f", iso)
            } else {
                exposureLockLabel = "ISO locked"
            }
        case .shutter:
            if let s = capture.lockedShutterSeconds, s > 0 {
                exposureLockLabel = String(format: "Shutter locked 1/%.0f", 1.0 / s)
            } else {
                exposureLockLabel = "Shutter locked"
            }
        }
    }

    private func saveAccepted(_ accepted: BestFrameGate.Accepted) async {
        guard isScanning, let store, !saving else { return }
        saving = true
        defer { saving = false }
        let transform = accepted.pose ?? matrix_identity_float4x4
        do {
            let id = try await store.writeJPEG(accepted.jpeg, transform: transform)
            let pos = SIMD3(transform.columns.3.x, transform.columns.3.y,
                            transform.columns.3.z)
            photos.append(CapturedPoseSample(id: id, position: pos,
                                             transform: transform))
            photoCount = photos.count
            megabytes = await store.megabytes
            statusLine = "\(photoCount) · \(String(format: "%.1f MB", megabytes))"
            UIImpactFeedbackGenerator(style: .light).impactOccurred()
        } catch {
            statusLine = "Save failed: \(error.localizedDescription)"
        }
    }

    func currentDirectory() async -> URL? { await store?.directory }

    func packageName() async -> String {
        if let n = newProjectName, !n.isEmpty { return n }
        if let n = continuingProject?.name { return n }
        return await store?.directory.lastPathComponent ?? "capture"
    }
}
