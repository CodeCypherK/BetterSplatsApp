import AVFoundation
import SwiftUI

/// Home screen: projects, mesh scan, sessions, desktop PC.
struct RootView: View {
    @State private var cameraStatus = AVCaptureDevice.authorizationStatus(for: .video)
    @ObservedObject private var desktop = DesktopSender.shared

    private var canMesh: Bool {
        UltraWideARCapture.isAvailable
    }

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
                    .disabled(!canMesh)

                    NavigationLink {
                        ScanView()
                    } label: {
                        Label("Scan", systemImage: "cube.transparent")
                    }
                    .disabled(!canMesh)

                    NavigationLink {
                        SessionsView()
                    } label: {
                        Label("Sessions", systemImage: "square.stack.3d.up")
                    }
                } footer: {
                    if !canMesh {
                        Text("Scanning needs ARKit world tracking.")
                    } else {
                        Text("1× ARKit capture keeps the best frame each "
                           + "second. One folder of images per room goes to "
                           + "the desktop.")
                    }
                }

                Section {
                    TextField("PC IP (Tailscale or LAN)", text: $desktop.host)
                        .keyboardType(.numbersAndPunctuation)
                        .textInputAutocapitalization(.never)
                        .autocorrectionDisabled()
                        .onSubmit {
                            desktop.host = desktop.host
                                .trimmingCharacters(in: .whitespacesAndNewlines)
                        }
                    HStack {
                        Text("Port")
                        Spacer()
                        TextField("9999", text: Binding(
                            get: { String(desktop.port) },
                            set: { if let v = UInt16($0), v > 0 { desktop.port = v } }
                        ))
                        .keyboardType(.numberPad)
                        .multilineTextAlignment(.trailing)
                        .frame(width: 80)
                    }
                    if let status = desktop.status {
                        Text(status)
                            .font(.caption)
                            .foregroundStyle(.secondary)
                    }
                } header: {
                    Text("Desktop PC")
                } footer: {
                    Text("Run the capture receiver on the PC, then Send from "
                       + "a finished scan. Images only — depth is estimated "
                       + "on the desktop.")
                }

                Section("Device") {
                    LabeledContent("World tracking") {
                        Text(canMesh ? "Available" : "Not found")
                            .foregroundStyle(canMesh ? .green : .red)
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
