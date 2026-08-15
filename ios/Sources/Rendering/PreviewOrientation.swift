import CoreGraphics
import simd

/// Maps the sensor-native video buffer onto the portrait preview quad.
///
/// Separated from the renderer and written as arithmetic rather than as a
/// table of four hand-picked corners, because the hand-picked version was
/// wrong in a way that is very hard to see by reading: it mapped
/// `(u,v) -> (1-v, 1-u)`, which is a REFLECTION about the anti-diagonal, not
/// a rotation. It survived to a device and showed up as an upside-down
/// camera feed.
///
/// Two coordinate systems, and the whole bug lives in the gap between them:
///
///   * **Clip space** is y-UP: the vertex at (-1,-1) is the BOTTOM-left of
///     the screen, and the viewport maps y = +1 to the drawable's top row.
///   * **Metal texture space** is y-DOWN from the top-left, and a texture
///     made from a CVPixelBuffer puts buffer row 0 at v = 0 with no implicit
///     flip.
///
/// So the quad's four vertices, expressed as screen coordinates with the
/// origin at the top-left and v increasing downward, are
/// (0,1), (1,1), (0,0), (1,1) in vertex order — NOT the (0,0)-first ordering
/// the clip-space positions suggest at a glance.
enum PreviewOrientation {
    /// Clockwise rotation applied to the source image to display it upright.
    ///
    /// The rear camera delivers buffers landscape (1920x1440), and the app is
    /// portrait-only (`UISupportedInterfaceOrientations` is Portrait alone),
    /// so there is exactly one of these and it is a constant rather than a
    /// device-orientation lookup. If the app ever supports landscape, this
    /// becomes a function of the interface orientation: portrait 90,
    /// landscapeLeft 180, portraitUpsideDown 270, landscapeRight 0.
    static let rotationCW = 90

    /// Screen UV -> source texture UV for a clockwise rotation of the source.
    ///
    /// Derivation, for `rotationCW == 90`: rotating an image clockwise sends
    /// its bottom-left corner to the top-left. The display's top-left
    /// (u_d, v_d) = (0,0) must therefore sample the source's bottom-left,
    /// (0,1) — which `(v_d, 1 - u_d)` gives. The other three fall out of the
    /// same expression.
    static func sourceUV(screenU u: Float, screenV v: Float,
                         rotationCW degrees: Int) -> SIMD2<Float> {
        switch ((degrees % 360) + 360) % 360 {
        case 90:  return SIMD2(v, 1 - u)
        case 180: return SIMD2(1 - u, 1 - v)
        case 270: return SIMD2(1 - v, u)
        default:  return SIMD2(u, v)
        }
    }

    /// The four texture coordinates, in the vertex order the shader uses,
    /// aspect-FILLED into `drawableSize`.
    ///
    /// Aspect matters here beyond looking tidy. This is a capture app: the
    /// user frames a shot by what the preview shows, and a 4:3 sensor image
    /// stretched to fill a ~9:19.5 portrait view is not what gets stored.
    /// Fill rather than fit, because letterboxing a viewfinder invites the
    /// user to frame into bars that are not part of the picture; the cost is
    /// that the edges of the frame are cropped from view, which is the
    /// conventional trade and the one the camera app makes.
    static func texcoords(sourceWidth: Int, sourceHeight: Int,
                          drawableSize: CGSize,
                          rotationCW degrees: Int = rotationCW)
        -> [SIMD2<Float>] {
        // Screen-space corners in vertex order: bottom-left, bottom-right,
        // top-left, top-right — with v measured DOWNWARD from the top.
        let corners: [(Float, Float)] = [(0, 1), (1, 1), (0, 0), (1, 0)]

        let quarterTurn = (((degrees % 360) + 360) % 360) % 180 == 90
        let shownW = quarterTurn ? sourceHeight : sourceWidth
        let shownH = quarterTurn ? sourceWidth : sourceHeight
        guard shownW > 0, shownH > 0,
              drawableSize.width > 0, drawableSize.height > 0 else {
            return corners.map {
                sourceUV(screenU: $0.0, screenV: $0.1, rotationCW: degrees)
            }
        }

        let sourceAspect = Float(shownW) / Float(shownH)
        let viewAspect = Float(drawableSize.width / drawableSize.height)
        // Fraction of the rotated image that stays visible on each axis.
        var fracU: Float = 1
        var fracV: Float = 1
        if sourceAspect > viewAspect {
            fracU = viewAspect / sourceAspect   // source is wider: crop sides
        } else {
            fracV = sourceAspect / viewAspect   // source is taller: crop top/bottom
        }

        return corners.map { corner in
            // Centred crop, in the ROTATED image's own coordinates...
            let ru = 0.5 + (corner.0 - 0.5) * fracU
            let rv = 0.5 + (corner.1 - 0.5) * fracV
            // ...then back to the source buffer's coordinates.
            return sourceUV(screenU: ru, screenV: rv, rotationCW: degrees)
        }
    }
}
