import ARKit
import Foundation
import Observation
import simd
import SwiftUI
import UIKit

/// ARKit mesh coverage capture. Photos are JPEGs for desktop DA3; the mesh
/// is only so you can see what you have and what is still thin.
@MainActor
@Observable
final class ScanViewModel {
    enum Phase: Equatable {
        case ready
        case scanning
        case finishing
        case done
        case failed(String)
    }

    var phase: Phase = .ready
    var guidance = "Start scanning to build the mesh"
    var stats = CoverageStats()
    var thinAnchorIds = Set<UUID>()
    var photos: [CapturedPoseSample] = []
    var autoCapture = true
    var megabytes: Double = 0

    var continuingProject: ProjectStore.Project?
    var newProjectName: String?

    /// Wired from CoverageARView so the shutter can grab the current frame.
    weak var arCoordinator: CoverageARView.Coordinator?

    private var store: ImageSessionStore?
    private let jpeg = JpegEncoder(quality: 0.88)
    private var lastCaptureTransform: simd_float4x4?
    private var lastCaptureTime: TimeInterval = 0
    private var minMoveM: Float = 0.28
    private var minTurnRad: Float = 0.28  // ~16°
    private var minInterval: TimeInterval = 0.55

    var photoCount: Int { photos.count }
    var isScanning: Bool {
        if case .scanning = phase { return true }
        return false
    }

    func sessionStarted() {
        if phase == .ready {
            guidance = "Mesh is live — walk and look; photos fill coverage"
        }
    }

    func fail(_ message: String) {
        phase = .failed(message)
        guidance = message
    }

    func applyCoverage(stats: CoverageStats, thinIds: Set<UUID>) {
        self.stats = stats
        thinAnchorIds = thinIds
        if isScanning {
            guidance = stats.headline
        }
    }

    func noteFrame(transform: simd_float4x4, trackingOK: Bool) {
        guard isScanning, autoCapture, trackingOK else { return }
        maybeAutoCapture(transform: transform)
    }

    func start() {
        guard phase == .ready || phase == .done else { return }
        do {
            let store = try ImageSessionStore(
                osVersion: UIDevice.current.systemVersion,
                continuing: continuingProject,
                newProjectName: newProjectName)
            self.store = store
            photos = []
            stats = CoverageStats()
            thinAnchorIds = []
            lastCaptureTransform = nil
            phase = .scanning
            guidance = "Walk the space — orange means needs photos"
            arCoordinator?.startSessionIfNeeded()
        } catch {
            fail(error.localizedDescription)
        }
    }

    func stop() {
        guard isScanning || phase == .finishing else { return }
        phase = .finishing
        arCoordinator?.pause()
        Task {
            do {
                try await store?.finalize()
                phase = .done
                guidance = "Saved \(photoCount) photos"
            } catch {
                fail(error.localizedDescription)
            }
        }
    }

    func snap() {
        guard isScanning else { return }
        Task { await captureNow(force: true) }
    }

    private func maybeAutoCapture(transform: simd_float4x4) {
        let now = CACurrentMediaTime()
        guard now - lastCaptureTime >= minInterval else { return }
        if let last = lastCaptureTransform {
            let p0 = SIMD3(last.columns.3.x, last.columns.3.y, last.columns.3.z)
            let p1 = SIMD3(transform.columns.3.x, transform.columns.3.y,
                           transform.columns.3.z)
            let moved = simd_distance(p0, p1)
            let f0 = -SIMD3(last.columns.2.x, last.columns.2.y, last.columns.2.z)
            let f1 = -SIMD3(transform.columns.2.x, transform.columns.2.y,
                            transform.columns.2.z)
            let turn = acos(min(1, max(-1, simd_dot(simd_normalize(f0),
                                                    simd_normalize(f1)))))
            // Prefer capturing when looking at thin mesh: always allow if
            // coverage is weak; otherwise require motion.
            let hungry = stats.coveredFraction < 0.7 || stats.photos < 12
            if !hungry, moved < minMoveM, turn < minTurnRad { return }
            if moved < minMoveM * 0.45, turn < minTurnRad * 0.45 { return }
        }
        Task { await captureNow(force: false) }
    }

    private func captureNow(force: Bool) async {
        guard isScanning, let store else { return }
        guard let grabbed = arCoordinator?.captureJPEG(encoder: jpeg) else {
            if force { guidance = "Could not grab a frame — try again" }
            return
        }
        let (jpegData, transform, _) = grabbed
        do {
            let id = try await store.writeJPEG(jpegData, transform: transform)
            let pos = SIMD3(transform.columns.3.x, transform.columns.3.y,
                            transform.columns.3.z)
            photos.append(CapturedPoseSample(id: id, position: pos,
                                             transform: transform))
            lastCaptureTransform = transform
            lastCaptureTime = CACurrentMediaTime()
            megabytes = await store.megabytes
            UIImpactFeedbackGenerator(style: .light).impactOccurred()
        } catch {
            guidance = "Save failed: \(error.localizedDescription)"
        }
    }

    var sessionDirectory: URL? { nil } // resolved async when sending

    func currentDirectory() async -> URL? {
        await store?.directory
    }

    func packageName() async -> String {
        if let n = newProjectName, !n.isEmpty { return n }
        if let n = continuingProject?.name { return n }
        return await store?.directory.lastPathComponent ?? "capture"
    }
}
