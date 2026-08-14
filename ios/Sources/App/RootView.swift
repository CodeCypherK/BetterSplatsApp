import AVFoundation
import SwiftUI

/// Home screen: entry to capture and sessions, plus the device diagnostics
/// that prove the toolchain on a side-loaded build (engine link, LiDAR
/// presence, camera permission, Metal).
struct RootView: View {
    @State private var cameraStatus = AVCaptureDevice.authorizationStatus(for: .video)
    @State private var selftestResult: (ok: Bool, report: String)?

    private var hasLiDAR: Bool { CaptureManager.hasLiDAR }

    var body: some View {
        NavigationStack {
            List {
                Section {
                    NavigationLink {
                        ProjectsView()
                    } label: {
                        Label("Projects", systemImage: "square.stack.3d.up.fill")
                            .font(.headline)
                    }
                    .disabled(!hasLiDAR)

                    NavigationLink {
                        CaptureView()
                    } label: {
                        Label("Quick Capture", systemImage: "camera.viewfinder")
                    }
                    .disabled(!hasLiDAR)

                    NavigationLink {
                        SessionsView()
                    } label: {
                        Label("Sessions", systemImage: "square.stack.3d.up")
                    }
                } footer: {
                    if !hasLiDAR {
                        Text("Capture requires a LiDAR iPhone (12 Pro or later Pro model).")
                    } else {
                        Text("Sessions are saved to the Files app (On My iPhone → BetterSplats) — copy them off the phone regularly.")
                    }
                }

                Section("Diagnostics") {
                    LabeledContent("Engine", value: CoreEngine.version)
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
                    if let result = selftestResult, !result.ok {
                        Text(result.report)
                            .font(.caption.monospaced())
                            .foregroundStyle(.secondary)
                    }
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
                                    cameraStatus =
                                        AVCaptureDevice.authorizationStatus(for: .video)
                                }
                            }
                        }
                    }
                }
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
