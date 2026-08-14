import MetalKit
import SwiftUI

/// Live capture screen: full-screen camera preview with the guidance pill,
/// capture counters and start/stop control.
struct CaptureView: View {
    @State private var model: CaptureViewModel
    @Environment(\.dismiss) private var dismiss

    /// `project` continues an existing one; `newProjectName` starts a new one.
    /// Both nil is a standalone capture, which is what the old entry point
    /// did and still works.
    init(project: ProjectStore.Project? = nil, newProjectName: String? = nil,
         rescanLabel: String? = nil) {
        let model = CaptureViewModel()
        model.continuingProject = project
        model.newProjectName = newProjectName
        model.rescanLabel = rescanLabel
        _model = State(initialValue: model)
    }

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

            if let recovery = model.recovery {
                recoveryArrow(recovery)
            }

            if let phase = model.floorPhase {
                floorCalibrationPrompt(phase)
            }

            if model.state == .idle {
                // A rescan is always one room by definition, so asking "one
                // room or several" would be a question with a known answer.
                if model.isRescan { rescanIntro } else { planChooser }
            }

            if case .failed(let message) = model.state {
                errorOverlay(message)
            }
        }
        .navigationBarBackButtonHidden(model.isRunning)
        .toolbar(.hidden, for: .tabBar)
        .onDisappear { if model.isRunning { model.stop() } }
        .statusBarHidden()
    }

    /// Asked once, before the camera starts, because the answer changes what
    /// the first minute is for and cannot be changed later without throwing
    /// the session away.
    ///
    /// The question is about the space, not about the algorithm. "Do you want
    /// a localization scaffold" is unanswerable by the person holding the
    /// phone; "how many rooms" is the same decision in terms they can see.
    private var planChooser: some View {
        // The scrim is not decoration: without it the shutter button in the
        // bottom bar sits under this card and stays tappable, so a tap
        // meant for "several rooms" that misses the card silently starts a
        // single-room session instead.
        ZStack {
            Color.black.opacity(0.45)
                .ignoresSafeArea()
                .contentShape(Rectangle())
                .onTapGesture {}
            planChooserCard
        }
    }

    /// Redoing one room. Says plainly what will happen to the old frames,
    /// because "replace" is the kind of word people reasonably read as
    /// "delete" — and here it does not mean that.
    private var rescanIntro: some View {
        ZStack {
            Color.black.opacity(0.45)
                .ignoresSafeArea()
                .contentShape(Rectangle())
                .onTapGesture {}
            VStack(spacing: 16) {
                Image(systemName: "arrow.clockwise")
                    .font(.system(size: 34, weight: .semibold))
                    .foregroundStyle(.tint)
                Text("Redo \(model.rescanLabel ?? "this room")")
                    .font(.title3.weight(.semibold))
                Text("Walk this room again. Wherever you go, the new frames "
                   + "take over from the old ones.")
                    .font(.subheadline)
                    .multilineTextAlignment(.center)
                Text("The old frames stay on your phone — the reconstruction "
                   + "just stops using them, so this is reversible.")
                    .font(.caption)
                    .foregroundStyle(.secondary)
                    .multilineTextAlignment(.center)
                Button {
                    model.start(plan: .captureOnly)
                } label: {
                    Label("Start", systemImage: "camera.viewfinder")
                        .frame(maxWidth: .infinity)
                }
                .buttonStyle(.borderedProminent)
                .controlSize(.large)
            }
            .padding(24)
            .frame(maxWidth: 380)
            .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 20))
            .padding(24)
        }
    }

    private var planChooserCard: some View {
        VStack(spacing: 18) {
            Image(systemName: "map")
                .font(.system(size: 34, weight: .semibold))
                .foregroundStyle(.tint)
            Text("What are you scanning?")
                .font(.title3.weight(.semibold))

            VStack(spacing: 12) {
                Button {
                    model.start(plan: .captureOnly)
                } label: {
                    planOption(
                        title: "One room",
                        detail: "Start scanning straight away.",
                        icon: "square.dashed")
                }
                Button {
                    model.start(plan: .scoutThenCapture)
                } label: {
                    planOption(
                        title: "Several rooms",
                        detail: "Walk the whole space once first — a lap of "
                              + "every room, back to the walls, camera facing "
                              + "in. It takes a minute and gives the detailed "
                              + "scan something to hold position against.",
                        icon: "figure.walk.motion")
                }
            }
            .buttonStyle(.plain)
        }
        .padding(24)
        .frame(maxWidth: 380)
        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 20))
        .padding(24)
    }

    private func planOption(title: String, detail: String,
                            icon: String) -> some View {
        HStack(alignment: .top, spacing: 14) {
            Image(systemName: icon)
                .font(.system(size: 22))
                .foregroundStyle(.tint)
                .frame(width: 30)
            VStack(alignment: .leading, spacing: 4) {
                Text(title).font(.headline)
                Text(detail)
                    .font(.caption)
                    .foregroundStyle(.secondary)
                    .fixedSize(horizontal: false, vertical: true)
            }
            Spacer(minLength: 0)
        }
        .padding(14)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 12))
    }

    /// The opening step: measure the floor so the finished scan comes out
    /// level and square instead of on a slope at an arbitrary bearing.
    ///
    /// The instruction asks for a step forward, not just an aim, and that is
    /// load-bearing rather than friction: a calibration frame captured while
    /// standing still is pure rotation, nothing triangulates around it, and
    /// the solve drops the frame the calibration is attached to. Measured on
    /// the harness, adding the movement took registration from 71/84 to
    /// 84/84.
    @ViewBuilder
    private func floorCalibrationPrompt(_ phase: FloorCalibrator.Phase) -> some View {
        VStack {
            Spacer()
            VStack(spacing: 12) {
                switch phase {
                case .aiming(let advice, let heightM):
                    Image(systemName: "arrow.down.to.line")
                        .font(.system(size: 30, weight: .semibold))
                        .foregroundStyle(.tint)
                    Text("Point at the floor and take a step")
                        .font(.headline)
                    Text(advice)
                        .font(.subheadline)
                        .foregroundStyle(.secondary)
                        .multilineTextAlignment(.center)
                    if let heightM {
                        Text(String(format: "%.2f m above the floor", heightM))
                            .font(.caption.monospacedDigit())
                            .foregroundStyle(.secondary)
                    }
                    Button("Skip") { model.skipFloorCalibration() }
                        .font(.subheadline)
                        .buttonStyle(.bordered)

                case .measured(let heightM):
                    Image(systemName: "checkmark.circle.fill")
                        .font(.system(size: 30, weight: .semibold))
                        .foregroundStyle(.green)
                    Text("Floor measured")
                        .font(.headline)
                    Text(String(format: "%.2f m below the camera — carry on scanning",
                                heightM))
                        .font(.subheadline)
                        .foregroundStyle(.secondary)
                        .multilineTextAlignment(.center)

                case .skipped:
                    Image(systemName: "arrow.turn.down.right")
                        .font(.system(size: 26, weight: .semibold))
                        .foregroundStyle(.secondary)
                    Text("Carrying on without it")
                        .font(.headline)
                    Text("The floor will be worked out from the scan instead.")
                        .font(.subheadline)
                        .foregroundStyle(.secondary)
                        .multilineTextAlignment(.center)
                }
            }
            .padding(20)
            .frame(maxWidth: 340)
            .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 16))
            .padding(.bottom, 60)
        }
        .transition(.opacity)
        .animation(.easeInOut(duration: 0.2), value: model.floorPhase)
    }

    /// A big arrow pointing at the nearest mapped place, centred over the
    /// preview.
    ///
    /// This is deliberately the loudest thing on the screen. Lost tracking is
    /// the one state where the user cannot make progress by carrying on, and
    /// every second spent lost is frames that do not reach the
    /// reconstruction — so it is worth interrupting for, unlike the advisory
    /// guidance in the pill.
    @ViewBuilder
    private func recoveryArrow(_ recovery: RecoveryHint) -> some View {
        VStack(spacing: 14) {
            // Behind the user, an on-screen arrow is the wrong instrument —
            // pointing at the bottom edge of the phone does not read as "turn
            // round". Say it instead.
            if recovery.isBehind {
                Image(systemName: "arrow.uturn.backward")
                    .font(.system(size: 64, weight: .bold))
            } else {
                Image(systemName: "arrow.up")
                    .font(.system(size: 72, weight: .bold))
                    .rotationEffect(.radians(recovery.arrowAngle))
            }
            Text(String(format: "%.1f m", recovery.distanceM))
                .font(.title3.weight(.semibold).monospacedDigit())
        }
        .foregroundStyle(.orange)
        .shadow(radius: 6)
        .padding(28)
        .background(.ultraThinMaterial, in: Circle())
        .transition(.scale.combined(with: .opacity))
        .animation(.easeInOut(duration: 0.2), value: recovery)
        .allowsHitTesting(false)
    }

    /// What just happened, and what to do next — shown while the user is
    /// still standing in the room they captured.
    ///
    /// That timing is the whole value. A thin capture looks completely fine
    /// at the time and only shows up as holes in the trained splat hours
    /// later, by which point fixing it means a return trip. One extra minute
    /// here costs almost nothing.
    @ViewBuilder
    private var finishedCard: some View {
        let summary = model.summary
        VStack(spacing: 12) {
            if let summary {
                Image(systemName: summaryIcon(summary.verdict))
                    .font(.system(size: 30, weight: .semibold))
                    .foregroundStyle(summaryColor(summary.verdict))
                Text(summary.headline).font(.headline)
                Text(summary.detail)
                    .font(.caption)
                    .foregroundStyle(.secondary)
                    .multilineTextAlignment(.center)
                    .fixedSize(horizontal: false, vertical: true)
            } else {
                Text("Capture saved").font(.headline)
            }

            // A thin room is the one case where the primary action should be
            // "fix it", not "move on".
            if summary?.verdict == .thin {
                Button {
                    model.restartForMoreCoverage()
                } label: {
                    Label("Walk it again", systemImage: "arrow.clockwise")
                        .frame(maxWidth: .infinity)
                }
                .buttonStyle(.borderedProminent)
                .controlSize(.large)
                Button("Keep it anyway") { dismiss() }
                    .font(.subheadline)
            } else {
                Button("Done") { dismiss() }
                    .buttonStyle(.borderedProminent)
                    .controlSize(.large)
            }
        }
        .padding(20)
        .frame(maxWidth: 360)
        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 16))
    }

    private func summaryIcon(_ v: CaptureViewModel.CaptureSummary.Verdict) -> String {
        switch v {
        case .good: return "checkmark.circle.fill"
        case .thin: return "exclamationmark.triangle.fill"
        case .full: return "tray.full.fill"
        }
    }

    private func summaryColor(_ v: CaptureViewModel.CaptureSummary.Verdict) -> Color {
        switch v {
        case .good: return .green
        case .thin: return .orange
        case .full: return .blue
        }
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

            // Too thin and too full are opposite problems and must not look
            // alike: one says keep going, the other says stop. Below target
            // is ordinary progress, not a warning — nobody should feel they
            // are doing something wrong thirty seconds into a room.
            if let note = model.storageNote {
                let stored = Int(model.framesStored)
                let underTarget = stored < FrameFeedContext.storedFrameTarget
                Label(note, systemImage: underTarget
                      ? "circle.dotted" : "exclamationmark.triangle.fill")
                    .font(.caption.weight(.medium))
                    .foregroundStyle(
                        underTarget ? Color.secondary
                        : (stored >= FrameFeedContext.storedFrameCap
                           ? Color.red : Color.orange))
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

            // Said plainly during the circuit, because the frame counter is
            // climbing and someone watching it would otherwise reasonably
            // believe the scan has begun.
            if model.isScouting {
                Text("These frames map the route — the scan itself comes next")
                    .font(.caption2)
                    .foregroundStyle(.secondary)
                    .padding(.horizontal, 12)
                    .padding(.vertical, 4)
                    .background(.ultraThinMaterial, in: Capsule())
            } else if model.scoutFramesStored > 0 {
                Text("\(model.scoutFramesStored) route frames + "
                     + "\(max(0, Int(model.framesStored) - Int(model.scoutFramesStored))) scan frames")
                    .font(.caption2.monospacedDigit())
                    .foregroundStyle(.secondary)
                    .padding(.horizontal, 12)
                    .padding(.vertical, 4)
                    .background(.ultraThinMaterial, in: Capsule())
            }
        }
    }

    /// During the circuit the primary action is not "stop" — it is "I have
    /// walked the whole space, now let me scan it". Presenting a shutter
    /// button here would make the natural next tap end the session.
    private var scoutBar: some View {
        VStack(spacing: 10) {
            Text("Walking the space")
                .font(.subheadline.weight(.semibold))
            Text(model.scaffoldKeyframes == 0
                 ? "Mapping…"
                 : "\(model.scaffoldKeyframes) waypoints mapped")
                .font(.caption.monospacedDigit())
                .foregroundStyle(.secondary)
            Text("Finish where you started, then scan in detail.")
                .font(.caption)
                .foregroundStyle(.secondary)
                .multilineTextAlignment(.center)
            Button {
                model.finishScout()
            } label: {
                Label("Start detailed scan", systemImage: "camera.viewfinder")
                    .frame(maxWidth: .infinity)
            }
            .buttonStyle(.borderedProminent)
            .controlSize(.large)
            Button("Cancel session") { model.stop() }
                .font(.caption)
                .buttonStyle(.plain)
                .foregroundStyle(.secondary)
        }
        .padding(16)
        .frame(maxWidth: 340)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 16))
    }

    private var bottomBar: some View {
        HStack {
            if model.isScouting {
                scoutBar.frame(maxWidth: .infinity)
            } else if case .finished = model.state {
                finishedCard.frame(maxWidth: .infinity)
            } else {
                Button {
                    if model.isCapturing { model.stop() } else { model.start() }
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
