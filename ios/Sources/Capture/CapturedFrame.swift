import AVFoundation
import CoreVideo

/// One synchronized RGB+depth observation as delivered by the capture
/// pipeline, in sensor-native orientation. The pixel buffer is only valid
/// for the duration of the delegate callback unless retained.
struct CapturedFrame {
    let pixelBuffer: CVPixelBuffer      // 420f bi-planar YUV, 1920x1440
    let depthData: AVDepthData          // converted to DepthFloat16 by the manager
    let tCapture: Double                // seconds, host clock
    let tDepth: Double
    let calibration: AVCameraCalibrationData?
    let exposureDuration: Double
    let iso: Double
    let exposureBias: Double
    /// Integrated raw gyro rotation (rad, device frame) since the previous
    /// frame. Hint-only per the architecture: never a pose source.
    let gyroDelta: (x: Float, y: Float, z: Float)?
}
