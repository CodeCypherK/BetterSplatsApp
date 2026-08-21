import CoreImage
import CoreVideo
import Foundation
import ImageIO

/// GPU-backed JPEG encoding of capture pixel buffers.
final class JpegEncoder {
    private let context: CIContext
    private let fallbackColorSpace = CGColorSpace(name: CGColorSpace.sRGB)!
    private let quality: Double

    init(quality: Double = 0.85) {
        self.quality = quality
        context = CIContext(options: [.cacheIntermediates: false])
    }

    func encode(_ pixelBuffer: CVPixelBuffer) -> Data? {
        encode(CIImage(cvPixelBuffer: pixelBuffer))
    }

    /// AVCapture portrait buffers are already upright (orientation `.up`).
    /// Uses the buffer's attached color space when CIImage exposes one so
    /// video-range YCbCr does not get forced through the wrong transfer.
    func encodeYUV(_ pixelBuffer: CVPixelBuffer,
                   orientation: CGImagePropertyOrientation = .up) -> Data? {
        let image = CIImage(cvPixelBuffer: pixelBuffer).oriented(orientation)
        return encode(image)
    }

    private func encode(_ image: CIImage) -> Data? {
        let options = [
            CIImageRepresentationOption(
                rawValue: kCGImageDestinationLossyCompressionQuality as String): quality
        ]
        let space = image.colorSpace ?? fallbackColorSpace
        return context.jpegRepresentation(
            of: image, colorSpace: space, options: options)
    }
}
