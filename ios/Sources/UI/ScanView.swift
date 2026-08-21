import SwiftUI

/// Minimal scan UI: live preview, image count, pose pins, start/stop.
struct ScanView: View {
    @State private var model = ScanViewModel()
    @Environment(\.dismiss) private var dismiss
    @State private var sendFolders: [URL] = []
    @State private var sendName = "capture"

    init(project: ProjectStore.Project? = nil, newProjectName: String? = nil) {
        let m = ScanViewModel()
        m.continuingProject = project
        m.newProjectName = newProjectName
        _model = State(initialValue: m)
    }

    var body: some View {
        ZStack {
            ScanPreview(session: model.isScanning ? model.arSession : nil,
                        poses: model.photos)
                .ignoresSafeArea()
                .background(Color.black)

            VStack {
                HStack {
                    VStack(alignment: .leading, spacing: 2) {
                        Text("\(model.photoCount)")
                            .font(.system(size: 48, weight: .bold, design: .rounded))
                            .foregroundStyle(.white)
                            .shadow(radius: 3)
                        Text(model.statusLine)
                            .font(.caption)
                            .foregroundStyle(.white.opacity(0.9))
                    }
                    Spacer()
                }
                .padding(16)

                Spacer()

                if model.isScanning {
                    Button {
                        model.stop()
                    } label: {
                        Text("Stop")
                            .font(.headline)
                            .frame(maxWidth: .infinity)
                            .padding(.vertical, 16)
                    }
                    .buttonStyle(.borderedProminent)
                    .tint(.red)
                    .padding(.horizontal, 24)
                    .padding(.bottom, 28)
                }
            }

            if model.phase == .ready { startCard }
            if model.phase == .done { doneCard }
            if case .failed(let msg) = model.phase { errorCard(msg) }
        }
        .navigationBarBackButtonHidden(model.isScanning)
        .toolbar(.hidden, for: .tabBar)
        .statusBarHidden()
        .onDisappear {
            if model.isScanning { model.stop() }
        }
    }

    private var startCard: some View {
        VStack {
            Spacer()
            VStack(spacing: 14) {
                Text("Ultra-wide capture")
                    .font(.title3.weight(.semibold))
                Text("Keeps at most one sharp, well-exposed frame per second. "
                   + "Yellow marks show where photos were taken and which "
                   + "way the camera faced. No photo limit.")
                    .font(.subheadline)
                    .foregroundStyle(.secondary)
                    .multilineTextAlignment(.center)
                Button {
                    model.start()
                } label: {
                    Text("Start")
                        .frame(maxWidth: .infinity)
                }
                .buttonStyle(.borderedProminent)
                .controlSize(.large)
            }
            .padding(20)
            .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 20))
            .padding(24)
        }
    }

    private var doneCard: some View {
        VStack {
            Spacer()
            VStack(alignment: .leading, spacing: 12) {
                Text("Saved \(model.photoCount) photos")
                    .font(.headline)
                Text(String(format: "%.1f MB", model.megabytes))
                    .foregroundStyle(.secondary)
                if !sendFolders.isEmpty {
                    SendToDesktopButton(folders: sendFolders,
                                        packageName: sendName,
                                        nestUnderPackage: false,
                                        layout: .imagesOnly)
                } else {
                    ProgressView()
                }
                Button("Done") { dismiss() }
                    .frame(maxWidth: .infinity)
            }
            .padding(20)
            .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 20))
            .padding(24)
            .task { await prepareSend() }
        }
    }

    private func errorCard(_ msg: String) -> some View {
        VStack {
            Spacer()
            VStack(spacing: 12) {
                Text("Scan failed").font(.headline)
                Text(msg).font(.subheadline).foregroundStyle(.secondary)
                Button("Close") { dismiss() }
            }
            .padding(20)
            .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 20))
            .padding(24)
        }
    }

    private func prepareSend() async {
        if let dir = await model.currentDirectory() {
            sendFolders = [dir]
            sendName = await model.packageName()
        }
    }
}
