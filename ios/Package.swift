// swift-tools-version:5.9
//
// Host-side tests for the app's pure logic — no simulator, no device, and no
// Mac in the development loop. `swift test` on the CI macOS runner builds
// these for the host and runs them in seconds.
//
// This package exists ALONGSIDE the XcodeGen project; it does not build the
// app. XcodeGen globs `ios/Sources` and is unaffected by a manifest sitting
// beside it, and `ios-tests.yml` is a separate workflow from the one that
// produces the IPA, so nothing here can stop a build the user is waiting on.
//
// Only files with no UIKit / Metal / AVFoundation / engine dependency can go
// in AppLogic. That constraint is the point rather than a limitation: the
// preview-orientation mapping was four hand-picked corner pairs inside a
// renderer that cannot run anywhere but a phone, so nothing could check it,
// and a reflection passed for a rotation all the way to a device. Pulling
// the arithmetic out into a file that imports only CoreGraphics and simd is
// what made it testable at all.
//
// Sources are compiled in place, never copied — a test that reads a
// duplicate of the code proves nothing about the code that ships.
import PackageDescription

let package = Package(
    name: "BetterSplatsAppLogic",
    platforms: [.macOS(.v13)],
    targets: [
        .target(
            name: "AppLogic",
            path: "Sources/Rendering",
            // Everything else in this directory needs Metal or a device.
            exclude: [
                "MapRenderer.swift",
                "MapShaders.metal",
                "PreviewShaders.metal",
                "VideoPreviewRenderer.swift",
            ],
            sources: ["PreviewOrientation.swift"]
        ),
        .testTarget(
            name: "AppLogicTests",
            dependencies: ["AppLogic"],
            path: "Tests/Cases"
        ),
    ]
)
