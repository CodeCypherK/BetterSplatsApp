import AVFoundation
import SwiftUI

/// M0 diagnostics screen. Proves, on a side-loaded device build, that:
/// camera permission works, a LiDAR camera is present, Metal renders, and the
/// C++ engine (Eigen + OpenCV + Ceres) links and computes correctly.
/// The capture UI replaces this as the root in M1.
struct RootView: View {
    @State private var cameraStatus = AVCaptureDevice.authorizationStatus(for: .video)
    @State private var selftestResult: (ok: Bool, report: String)?

    private var hasLiDAR: Bool {
        AVCaptureDevice.default(.builtInLiDARDepthCamera, for: .video, position: .back) != nil
    }

    var body: some View {
        NavigationStack {
            ZStack {
                MetalBackdropView()
                    .ignoresSafeArea()

                List {
                    Section("Engine") {
                        LabeledContent("Version", value: CoreEngine.version)
                        HStack {
                            Text("Dependency selftest")
                            Spacer()
                            if let result = selftestResult {
                                Image(systemName: result.ok
                                      ? "checkmark.circle.fill"
                                      : "xmark.octagon.fill")
                                    .foregroundStyle(result.ok ? .green : .red)
                            } else {
                                ProgressView()
                            }
                        }
                        if let result = selftestResult {
                            Text(result.report)
                                .font(.caption.monospaced())
                                .foregroundStyle(.secondary)
                        }
                    }

                    Section("Sensors") {
                        LabeledContent("LiDAR camera") {
                            Text(hasLiDAR ? "Available" : "Not found")
                                .foregroundStyle(hasLiDAR ? .green : .red)
                        }
                        LabeledContent("Camera access") {
                            Text(cameraStatusLabel)
                        }
                        if cameraStatus == .notDetermined {
                            Button("Request camera access") {
                                AVCaptureDevice.requestAccess(for: .video) { _ in
                                    Task { @MainActor in
                                        cameraStatus = AVCaptureDevice.authorizationStatus(for: .video)
                                    }
                                }
                            }
                        }
                    }

                    Section {
                        Text("Capture UI lands in M1. This build verifies the "
                             + "toolchain: engine link, camera permission, LiDAR "
                             + "presence, and Metal rendering.")
                            .font(.footnote)
                            .foregroundStyle(.secondary)
                    }
                }
                .scrollContentBackground(.hidden)
            }
            .navigationTitle("BetterSplats")
        }
        .task {
            let engine = CoreEngine.shared
            selftestResult = await Task.detached { engine.selftest() }.value
        }
    }

    private var cameraStatusLabel: String {
        switch cameraStatus {
        case .authorized: return "Granted"
        case .denied: return "Denied"
        case .restricted: return "Restricted"
        case .notDetermined: return "Not requested"
        @unknown default: return "Unknown"
        }
    }
}
