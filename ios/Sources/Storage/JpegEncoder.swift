import CoreImage
import CoreVideo
import Foundation
import ImageIO

/// GPU-backed JPEG encoding of capture pixel buffers. One shared CIContext;
/// encodes are submitted from the storage path only (~3 fps), never for
/// every capture frame.
final class JpegEncoder {
    private let context: CIContext
    private let colorSpace = CGColorSpace(name: CGColorSpace.sRGB)!
    private let quality: Double

    init(quality: Double = 0.85) {
        self.quality = quality
        context = CIContext(options: [.cacheIntermediates: false])
    }

    func encode(_ pixelBuffer: CVPixelBuffer) -> Data? {
        let image = CIImage(cvPixelBuffer: pixelBuffer)
        return encode(image)
    }

    /// ARKit `capturedImage` is YCbCr biplanar; CIImage handles it. Apply
    /// portrait upright orientation so desktop training sees the photo the
    /// way the user held the phone.
    func encodeYUV(_ pixelBuffer: CVPixelBuffer,
                   orientation: CGImagePropertyOrientation = .right) -> Data? {
        let image = CIImage(cvPixelBuffer: pixelBuffer)
            .oriented(orientation)
        return encode(image)
    }

    private func encode(_ image: CIImage) -> Data? {
        let options = [
            CIImageRepresentationOption(
                rawValue: kCGImageDestinationLossyCompressionQuality as String): quality
        ]
        return context.jpegRepresentation(
            of: image, colorSpace: colorSpace, options: options)
    }
}
