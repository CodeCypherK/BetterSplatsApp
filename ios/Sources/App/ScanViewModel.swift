import ARKit
import CoreMedia
import Foundation
import Observation
import simd
import SwiftUI
import UIKit

final class FramePipeline: @unchecked Sendable {
    let gate = BestFrameGate()
}

/// Ultra-wide (when available) continuous capture via ARKit world tracking
/// without mesh: ≤1 accepted JPEG per second.
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
    var photoCount = 0
    var megabytes: Double = 0
    var photos: [CapturedPoseSample] = []
    var statusLine = "Ultra-wide · best frame each second"

    var continuingProject: ProjectStore.Project?
    var newProjectName: String?

    private var store: ImageSessionStore?
    private let capture = UltraWideARCapture()
    private let pipeline = FramePipeline()
    private let jpeg = JpegEncoder(quality: 0.9)
    private var saving = false

    var arSession: ARSession { capture.arSession }

    var isScanning: Bool {
        if case .scanning = phase { return true }
        return false
    }

    func fail(_ message: String) {
        phase = .failed(message)
        statusLine = message
    }

    func start() {
        guard phase == .ready || phase == .done else { return }
        do {
            try capture.start()
            let dims = capture.dimensions
            let store = try ImageSessionStore(
                osVersion: UIDevice.current.systemVersion,
                continuing: continuingProject,
                newProjectName: newProjectName,
                videoW: dims.width, videoH: dims.height)
            self.store = store
            photos = []
            photoCount = 0
            megabytes = 0
            capture.onFrame = { [weak self, pipeline] buffer, time, pose, _ in
                guard let accepted = pipeline.gate.consider(
                    pixelBuffer: buffer, time: time, pose: pose)
                else { return }
                Task { @MainActor [weak self] in
                    await self?.saveAccepted(accepted)
                }
            }
            phase = .scanning
            statusLine = "Scanning — best frame each second"
        } catch {
            fail(error.localizedDescription)
        }
    }

    func stop() {
        guard isScanning else { return }
        phase = .finishing
        capture.onFrame = nil
        capture.stop()
        Task {
            do {
                try await store?.finalize()
                phase = .done
                statusLine = "Saved \(photoCount) photos"
            } catch {
                fail(error.localizedDescription)
            }
        }
    }

    private func saveAccepted(_ accepted: BestFrameGate.Accepted) async {
        guard isScanning, let store, !saving else { return }
        saving = true
        defer { saving = false }
        guard let data = jpeg.encodeYUV(accepted.pixelBuffer,
                                        orientation: .right) else { return }
        let transform = accepted.pose ?? matrix_identity_float4x4
        do {
            let id = try await store.writeJPEG(data, transform: transform)
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
