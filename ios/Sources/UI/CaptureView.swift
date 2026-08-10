import MetalKit
import SwiftUI

/// Live capture screen: full-screen camera preview with the guidance pill,
/// capture counters and start/stop control.
struct CaptureView: View {
    @State private var model = CaptureViewModel()
    @Environment(\.dismiss) private var dismiss

    var body: some View {
        ZStack {
            PreviewSurface(renderer: model.previewRenderer)
                .ignoresSafeArea()

            VStack {
                statusPill
                Spacer()
                bottomBar
            }
            .padding()

            if case .failed(let message) = model.state {
                errorOverlay(message)
            }
        }
        .navigationBarBackButtonHidden(model.isCapturing)
        .toolbar(.hidden, for: .tabBar)
        .onAppear { model.start() }
        .onDisappear { if model.isCapturing { model.stop() } }
        .statusBarHidden()
    }

    private var statusPill: some View {
        VStack(spacing: 6) {
            Text(model.guidance)
                .font(.headline)
                .padding(.horizontal, 16)
                .padding(.vertical, 8)
                .background(.ultraThinMaterial, in: Capsule())

            HStack(spacing: 14) {
                Label("\(model.framesSeen)", systemImage: "eye")
                Label("\(model.framesStored)", systemImage: "internaldrive")
                Label(String(format: "%.0f MB", model.megabytesWritten),
                      systemImage: "externaldrive.badge.timemachine")
                if model.readinessOverall > 0 {
                    Label("\(Int(model.readinessOverall))%",
                          systemImage: "checkmark.seal")
                        .foregroundStyle(
                            model.readinessOverall >= 85 ? .green
                            : (model.readinessOverall >= 60 ? .orange : .red))
                }
            }
            .font(.caption.monospacedDigit())
            .padding(.horizontal, 12)
            .padding(.vertical, 5)
            .background(.ultraThinMaterial, in: Capsule())

            if let note = model.storageNote {
                Label(note, systemImage: "exclamationmark.triangle.fill")
                    .font(.caption.weight(.medium))
                    .foregroundStyle(
                        Int(model.framesStored) >= FrameFeedContext.storedFrameCap
                        ? Color.red : Color.orange)
                    .padding(.horizontal, 12)
                    .padding(.vertical, 5)
                    .background(.ultraThinMaterial, in: Capsule())
            }

            HStack(spacing: 10) {
                NavigationLink {
                    ReadinessDashboardView(model: model)
                } label: {
                    Label("Readiness", systemImage: "chart.bar.fill")
                }
                NavigationLink {
                    MapView(model: model)
                } label: {
                    Label("3D Map", systemImage: "cube.transparent")
                }
            }
            .font(.caption)
            .buttonStyle(.bordered)
        }
    }

    private var bottomBar: some View {
        HStack {
            if case .finished(let name) = model.state {
                VStack(spacing: 8) {
                    Text("Saved: \(name)")
                        .font(.footnote)
                        .padding(8)
                        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 8))
                    Button("Done") { dismiss() }
                        .buttonStyle(.borderedProminent)
                }
                .frame(maxWidth: .infinity)
            } else {
                Button {
                    model.isCapturing ? model.stop() : model.start()
                } label: {
                    ZStack {
                        Circle()
                            .strokeBorder(.white, lineWidth: 4)
                            .frame(width: 74, height: 74)
                        if model.isCapturing {
                            RoundedRectangle(cornerRadius: 6)
                                .fill(.red)
                                .frame(width: 30, height: 30)
                        } else {
                            Circle()
                                .fill(.red)
                                .frame(width: 60, height: 60)
                        }
                    }
                }
                .frame(maxWidth: .infinity)
                .disabled(model.state == .starting || model.state == .stopping)
            }
        }
    }

    private func errorOverlay(_ message: String) -> some View {
        VStack(spacing: 12) {
            Image(systemName: "exclamationmark.triangle.fill")
                .font(.largeTitle)
                .foregroundStyle(.yellow)
            Text(message)
                .multilineTextAlignment(.center)
            Button("Back") { dismiss() }
                .buttonStyle(.borderedProminent)
        }
        .padding(24)
        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 16))
        .padding(32)
    }
}

/// MTKView host for the preview renderer.
private struct PreviewSurface: UIViewRepresentable {
    let renderer: VideoPreviewRenderer

    func makeUIView(context: Context) -> MTKView {
        let view = MTKView()
        view.preferredFramesPerSecond = 30
        view.clearColor = MTLClearColor(red: 0.02, green: 0.02, blue: 0.03, alpha: 1)
        renderer.attach(to: view)
        view.delegate = renderer
        return view
    }

    func updateUIView(_ uiView: MTKView, context: Context) {}
}
