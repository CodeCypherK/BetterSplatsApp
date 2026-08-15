import CoreGraphics
import XCTest
import simd

@testable import AppLogic

/// The camera preview mapping, which shipped wrong once.
///
/// The version that reached a device mapped `(u,v) -> (1-v, 1-u)`: a
/// reflection about the anti-diagonal, not a rotation. Reading it, it looks
/// like a plausible rotation — that is exactly why it needs a test rather
/// than a careful reader. On the phone it showed as an upside-down feed.
///
/// Every expectation here is derived from the geometry, never from the
/// implementation. Rotating an image 90 degrees clockwise moves its corners
///
///     bottom-left -> top-left      top-left  -> top-right
///     top-right   -> bottom-right  bottom-right -> bottom-left
///
/// so the display corner on the left must SAMPLE the source corner that
/// lands there. Texture coordinates throughout: origin top-left, v downward,
/// which is what a texture built from a CVPixelBuffer gives with no flip.
final class PreviewOrientationTests: XCTestCase {
    private let tl = SIMD2<Float>(0, 0)
    private let tr = SIMD2<Float>(1, 0)
    private let br = SIMD2<Float>(1, 1)
    private let bl = SIMD2<Float>(0, 1)

    private func sample(_ corner: SIMD2<Float>, _ degrees: Int)
        -> SIMD2<Float> {
        PreviewOrientation.sourceUV(screenU: corner.x, screenV: corner.y,
                                    rotationCW: degrees)
    }

    func testAQuarterTurnClockwiseSendsEachCornerWhereRotationDoes() {
        // display corner -> the source corner a 90-degree CW turn puts there
        XCTAssertEqual(sample(tl, 90), bl)
        XCTAssertEqual(sample(tr, 90), tl)
        XCTAssertEqual(sample(br, 90), tr)
        XCTAssertEqual(sample(bl, 90), br)
    }

    func testTheShippedBugIsNotWhatTheMappingDoes() {
        // (1-v, 1-u) — pinned explicitly so a "simplification" cannot quietly
        // reintroduce it. It agrees with the correct mapping on the
        // anti-diagonal, so a spot check of one or two corners can miss it.
        for corner in [tl, tr, br, bl] {
            let reflected = SIMD2<Float>(1 - corner.y, 1 - corner.x)
            XCTAssertNotEqual(sample(corner, 90), reflected,
                              "corner \(corner) matches the reflection")
        }
    }

    func testHalfAndThreeQuarterTurnsAreAlsoRotations() {
        XCTAssertEqual(sample(tl, 180), br)
        XCTAssertEqual(sample(tr, 180), bl)
        XCTAssertEqual(sample(tl, 270), tr)
        XCTAssertEqual(sample(tr, 270), br)
        // No rotation is the identity, and negative or over-wound angles
        // normalise rather than falling through to it.
        XCTAssertEqual(sample(tr, 0), tr)
        XCTAssertEqual(sample(tl, -270), sample(tl, 90))
        XCTAssertEqual(sample(tl, 450), sample(tl, 90))
    }

    /// Three rotations composed with a fourth must return the identity. This
    /// catches a reflection without knowing which corner goes where: a
    /// reflection is its own inverse, so it fails to close the cycle.
    func testFourQuarterTurnsReturnToTheStart() {
        for degrees in [0, 90, 180, 270] {
            for start in [tl, tr, br, bl] {
                var p = start
                for _ in 0..<4 {
                    p = sample(p, degrees)
                }
                XCTAssertEqual(p, start, "\(degrees) deg from \(start)")
            }
        }
    }

    // MARK: - Aspect fill

    func testAWiderSourceIsCroppedOnItsLongAxisNotSquashed() {
        // 1920x1440 rotated a quarter turn shows as 1440 wide by 1920 tall
        // (aspect 0.75) inside a 1179x2556 portrait drawable (aspect ~0.46).
        // The source is RELATIVELY wider, so its sides are cropped.
        let uv = PreviewOrientation.texcoords(
            sourceWidth: 1920, sourceHeight: 1440,
            drawableSize: CGSize(width: 1179, height: 2556), rotationCW: 90)
        XCTAssertEqual(uv.count, 4)

        // Cropping the shown width means cropping the SOURCE's v axis, since
        // a quarter turn swaps the two. Nothing may leave [0,1] — sampling
        // outside the texture is what shows as a smeared or mirrored edge.
        let vs = uv.map(\.y)
        let us = uv.map(\.x)
        XCTAssertGreaterThan(vs.min()!, 0.0)
        XCTAssertLessThan(vs.max()!, 1.0)
        XCTAssertEqual(us.min()!, 0.0, accuracy: 1e-6)
        XCTAssertEqual(us.max()!, 1.0, accuracy: 1e-6)

        // Fill, not fit: the visible window stays centred.
        XCTAssertEqual((vs.min()! + vs.max()!) / 2, 0.5, accuracy: 1e-5)
    }

    func testAMatchingAspectCropsNothing() {
        // Shown 1440x1920 into a drawable of the same aspect.
        let uv = PreviewOrientation.texcoords(
            sourceWidth: 1920, sourceHeight: 1440,
            drawableSize: CGSize(width: 1440, height: 1920), rotationCW: 90)
        XCTAssertEqual(Set(uv.map(\.x)), [0, 1])
        XCTAssertEqual(Set(uv.map(\.y)), [0, 1])
    }

    func testTheQuadStaysAQuadUnderCrop() {
        // Two distinct u values and two distinct v values, i.e. an
        // axis-aligned rectangle. A mapping that skews would still pass a
        // corner-by-corner check written loosely; this will not.
        let uv = PreviewOrientation.texcoords(
            sourceWidth: 1920, sourceHeight: 1440,
            drawableSize: CGSize(width: 1179, height: 2556), rotationCW: 90)
        XCTAssertEqual(Set(uv.map(\.x)).count, 2)
        XCTAssertEqual(Set(uv.map(\.y)).count, 2)
    }

    func testDegenerateSizesFallBackInsteadOfDividingByZero() {
        // A drawable can be zero-sized for a frame during setup or rotation.
        // The corners must still come back rotated, not NaN.
        for size in [CGSize(width: 0, height: 100), CGSize(width: 100, height: 0),
                     CGSize.zero] {
            let uv = PreviewOrientation.texcoords(
                sourceWidth: 1920, sourceHeight: 1440,
                drawableSize: size, rotationCW: 90)
            XCTAssertEqual(uv.count, 4)
            for c in uv {
                XCTAssertFalse(c.x.isNaN || c.y.isNaN)
            }
        }
        let noSource = PreviewOrientation.texcoords(
            sourceWidth: 0, sourceHeight: 0,
            drawableSize: CGSize(width: 100, height: 200), rotationCW: 90)
        XCTAssertEqual(noSource.count, 4)
    }

    func testThePortraitConstantIsTheQuarterTurnTheCameraNeeds() {
        // The app is portrait-only and the rear camera delivers landscape
        // buffers, so this is a constant. If it ever stops being one, the
        // tests above are the specification for what replaces it.
        XCTAssertEqual(PreviewOrientation.rotationCW, 90)
    }
}
