import SwiftUI

/// Capture screen built around the AR mesh: see what is covered, snap what
/// is thin, send JPEGs to the desktop.
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
            CoverageARView(model: model)
                .ignoresSafeArea()

            VStack(spacing: 0) {
                topHUD
                Spacer()
                bottomBar
            }
            .padding(.horizontal, 16)
            .padding(.top, 8)
            .padding(.bottom, 20)

            if case .failed(let msg) = model.phase {
                errorCard(msg)
            }
            if model.phase == .done {
                doneCard
                    .task { await prepareSend() }
            }
            if model.phase == .ready {
                startCard
            }
        }
        .navigationBarBackButtonHidden(model.isScanning)
        .toolbar(.hidden, for: .tabBar)
        .statusBarHidden()
        .onDisappear {
            model.arCoordinator?.pause()
            if model.isScanning { model.stop() }
        }
    }

    private var topHUD: some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack(spacing: 10) {
                coverageRing
                VStack(alignment: .leading, spacing: 2) {
                    Text(model.guidance)
                        .font(.subheadline.weight(.semibold))
                        .foregroundStyle(.white)
                        .shadow(radius: 2)
                        .fixedSize(horizontal: false, vertical: true)
                    Text(statusLine)
                        .font(.caption.monospacedDigit())
                        .foregroundStyle(.white.opacity(0.85))
                }
                Spacer(minLength: 0)
            }
            .padding(12)
            .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 14))

            legend
        }
    }

    private var coverageRing: some View {
        let pct = model.stats.coveredPercent
        return ZStack {
            Circle()
                .stroke(Color.white.opacity(0.2), lineWidth: 6)
            Circle()
                .trim(from: 0, to: CGFloat(model.stats.coveredFraction))
                .stroke(
                    AngularGradient(
                        colors: [.orange, .yellow, .cyan],
                        center: .center),
                    style: StrokeStyle(lineWidth: 6, lineCap: .round))
                .rotationEffect(.degrees(-90))
            Text("\(pct)%")
                .font(.caption.weight(.bold).monospacedDigit())
                .foregroundStyle(.white)
        }
        .frame(width: 52, height: 52)
    }

    private var statusLine: String {
        "\(model.photoCount) photos · "
            + "\(model.stats.meshAnchorCount) mesh tiles · "
            + String(format: "%.0f MB", model.megabytes)
    }

    private var legend: some View {
        HStack(spacing: 14) {
            Label("Covered", systemImage: "square.grid.3x3.fill")
                .foregroundStyle(.cyan)
            Label("Needs photos", systemImage: "square.grid.3x3.fill")
                .foregroundStyle(.orange)
            Label("Photo", systemImage: "circle.fill")
                .foregroundStyle(.yellow)
        }
        .font(.caption2.weight(.medium))
        .padding(.horizontal, 10)
        .padding(.vertical, 6)
        .background(.black.opacity(0.35), in: Capsule())
    }

    private var bottomBar: some View {
        HStack(spacing: 18) {
            Button {
                model.autoCapture.toggle()
            } label: {
                Image(systemName: model.autoCapture
                      ? "figure.walk.motion" : "figure.walk")
                    .font(.title3)
                    .frame(width: 48, height: 48)
                    .background(.ultraThinMaterial, in: Circle())
            }
            .foregroundStyle(model.autoCapture ? Color.cyan : Color.white)
            .disabled(!model.isScanning)

            Button {
                model.snap()
            } label: {
                ZStack {
                    Circle()
                        .fill(.white)
                        .frame(width: 74, height: 74)
                    Circle()
                        .stroke(.white.opacity(0.5), lineWidth: 3)
                        .frame(width: 84, height: 84)
                }
            }
            .disabled(!model.isScanning)
            .accessibilityLabel("Snap photo")

            Button {
                model.stop()
            } label: {
                Image(systemName: "stop.fill")
                    .font(.title3)
                    .frame(width: 48, height: 48)
                    .background(.ultraThinMaterial, in: Circle())
            }
            .foregroundStyle(.red)
            .disabled(!model.isScanning)
        }
        .frame(maxWidth: .infinity)
    }

    private var startCard: some View {
        VStack(spacing: 16) {
            Spacer()
            VStack(spacing: 14) {
                Image(systemName: "cube.transparent")
                    .font(.system(size: 36, weight: .semibold))
                    .foregroundStyle(.cyan)
                Text("Scan with coverage")
                    .font(.title3.weight(.semibold))
                Text("ARKit builds a mesh of what you see. Cyan is covered "
                   + "by photos; orange still needs shots. Only images go "
                   + "to the desktop.")
                    .font(.subheadline)
                    .foregroundStyle(.secondary)
                    .multilineTextAlignment(.center)
                Button {
                    model.start()
                } label: {
                    Text("Start scan")
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
                Text("Coverage \(model.stats.coveredPercent)% · "
                   + String(format: "%.1f MB", model.megabytes))
                    .font(.subheadline)
                    .foregroundStyle(.secondary)
                if !sendFolders.isEmpty {
                    SendToDesktopButton(folders: sendFolders,
                                        packageName: sendName,
                                        nestUnderPackage: false,
                                        layout: .imagesOnly)
                } else {
                    Button("Prepare send") {
                        Task { await prepareSend() }
                    }
                }
                Button("Done") { dismiss() }
                    .frame(maxWidth: .infinity)
            }
            .padding(20)
            .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 20))
            .padding(24)
        }
    }

    private func errorCard(_ msg: String) -> some View {
        VStack {
            Spacer()
            VStack(spacing: 12) {
                Text("Scan failed")
                    .font(.headline)
                Text(msg)
                    .font(.subheadline)
                    .foregroundStyle(.secondary)
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
