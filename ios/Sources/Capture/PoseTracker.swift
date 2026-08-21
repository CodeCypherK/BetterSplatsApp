import CoreMotion
import Foundation
import simd

/// Approximate pose from CoreMotion while the ultra-wide camera is owned by
/// AVCapture (ARKit cannot share that camera). Good enough for novelty + the
/// corner map; desktop DA3 re-estimates poses from the images.
final class PoseTracker {
    private let motion = CMMotionManager()
    private let queue = OperationQueue()
    private let lock = NSLock()

    private var rotation = matrix_identity_float3x3
    private var position = SIMD3<Float>.zero
    private var velocity = SIMD3<Float>.zero
    private var lastTime: TimeInterval?

    func start() {
        guard motion.isDeviceMotionAvailable else { return }
        queue.name = "bs.pose"
        queue.maxConcurrentOperationCount = 1
        motion.deviceMotionUpdateInterval = 1.0 / 60.0
        lastTime = nil
        position = .zero
        velocity = .zero
        motion.startDeviceMotionUpdates(using: .xArbitraryZVertical,
                                        to: queue) { [weak self] data, _ in
            guard let self, let data else { return }
            self.ingest(data)
        }
    }

    func stop() {
        motion.stopDeviceMotionUpdates()
    }

    func currentPose() -> simd_float4x4? {
        lock.lock()
        defer { lock.unlock() }
        var t = matrix_identity_float4x4
        t.columns.0 = SIMD4(rotation.columns.0, 0)
        t.columns.1 = SIMD4(rotation.columns.1, 0)
        t.columns.2 = SIMD4(rotation.columns.2, 0)
        t.columns.3 = SIMD4(position, 1)
        return t
    }

    private func ingest(_ data: CMDeviceMotion) {
        let q = data.attitude.quaternion
        let quat = simd_quatf(ix: Float(q.x), iy: Float(q.y),
                              iz: Float(q.z), r: Float(q.w))
        let r = simd_float3x3(quat)

        let now = data.timestamp
        lock.lock()
        defer { lock.unlock() }
        rotation = r
        guard let prev = lastTime else {
            lastTime = now
            return
        }
        let dt = Float(now - prev)
        lastTime = now
        guard dt > 0, dt < 0.25 else { return }

        let a = SIMD3(Float(data.userAcceleration.x),
                      Float(data.userAcceleration.y),
                      Float(data.userAcceleration.z))
        let worldA = r * a * 9.81
        velocity = (velocity + worldA * dt) * 0.92
        if simd_length(velocity) > 0.05 {
            position += velocity * dt
        } else {
            velocity *= 0.5
        }
    }
}
