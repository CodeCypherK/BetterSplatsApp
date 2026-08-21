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
            ScanPreview(previewLayer: model.isScanning ? model.previewLayer : nil)
                .ignoresSafeArea()
                .background(Color.black)

            VStack {
                HStack(alignment: .top) {
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
                    PoseTrailView(poses: model.photos)
                        .frame(width: 120, height: 120)
                        .clipShape(RoundedRectangle(cornerRadius: 12))
                        .overlay(
                            RoundedRectangle(cornerRadius: 12)
                                .stroke(.white.opacity(0.2), lineWidth: 1))
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
                Text("0.5× ultra-wide")
                    .font(.title3.weight(.semibold))
                Text("Uses the ultra-wide camera. Keeps at most one photo per "
                   + "second when the frame is sharp, well exposed, and a new "
                   + "viewpoint. Images land in one folder for this room.")
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
