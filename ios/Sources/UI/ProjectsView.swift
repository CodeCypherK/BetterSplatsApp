import SwiftUI

/// The project board: every space you have captured, reopenable.
///
/// A project is the unit that matches how the work actually happens — a
/// house is not one capture, it is ten sittings over a space, and between
/// them the useful questions are "which room is thin", "can I re-export
/// this", "I want to redo the kitchen". A flat list of dated session folders
/// answers none of those.
struct ProjectsView: View {
    @State private var projects: [ProjectStore.Project] = []
    @State private var creating = false
    @State private var newName = ""
    @State private var renaming: ProjectStore.Project?
    @State private var renameText = ""

    var body: some View {
        List {
            if projects.isEmpty {
                ContentUnavailableView(
                    "No projects yet",
                    systemImage: "square.stack.3d.up",
                    description: Text("Create a project, then capture a room "
                                    + "at a time. Captures in one project "
                                    + "share a coordinate frame, so they line "
                                    + "up without any work later."))
            }

            ForEach(projects) { project in
                NavigationLink {
                    ProjectDetailView(project: project)
                } label: {
                    projectRow(project)
                }
                .swipeActions {
                    Button {
                        renaming = project
                        renameText = project.name
                    } label: {
                        Label("Rename", systemImage: "pencil")
                    }
                    .tint(.blue)
                }
            }
        }
        .navigationTitle("Projects")
        .toolbar {
            Button {
                newName = ""
                creating = true
            } label: {
                Label("New project", systemImage: "plus")
            }
        }
        .onAppear(perform: reload)
        .refreshable { reload() }
        .alert("New project", isPresented: $creating) {
            TextField("Name (e.g. Oak Street house)", text: $newName)
            Button("Cancel", role: .cancel) {}
            Button("Create") { startNewProject() }
        } message: {
            Text("You will capture a room at a time. Every capture in the "
               + "project shares one coordinate frame.")
        }
        .alert("Rename project", isPresented: Binding(
            get: { renaming != nil },
            set: { if !$0 { renaming = nil } })) {
            TextField("Name", text: $renameText)
            Button("Cancel", role: .cancel) { renaming = nil }
            Button("Rename") {
                if let project = renaming, !renameText.isEmpty {
                    ProjectStore.rename(project, to: renameText)
                }
                renaming = nil
                reload()
            }
        }
        .navigationDestination(item: $startingProject) { request in
            CaptureView(project: nil, newProjectName: request.name)
        }
    }

    /// Pushes straight into capture once a new project has been named. A
    /// project has no existence of its own until its first capture writes a
    /// session — there is no project file to create — so naming it and
    /// starting it are one action.
    @State private var startingProject: NewProject?

    struct NewProject: Identifiable, Hashable {
        let name: String
        var id: String { name }
    }

    private func projectRow(_ project: ProjectStore.Project) -> some View {
        VStack(alignment: .leading, spacing: 4) {
            Text(project.name).font(.headline)
            HStack(spacing: 12) {
                Label("\(project.captures.count)", systemImage: "camera")
                Label("\(project.frameCount)", systemImage: "photo.stack")
                if project.rescanCount > 0 {
                    Label("\(project.rescanCount)", systemImage: "arrow.clockwise")
                }
                if project.hasExport {
                    Label("exported", systemImage: "checkmark.seal")
                        .foregroundStyle(.green)
                }
            }
            .font(.caption.monospacedDigit())
            .foregroundStyle(.secondary)
        }
        .padding(.vertical, 2)
    }

    private func startNewProject() {
        startingProject = NewProject(
            name: newName.isEmpty ? "Untitled" : newName)
    }

    private func reload() { projects = ProjectStore.load() }
}

/// One project: its captures, and the three things you come back to do —
/// capture more, re-export, or redo a room.
struct ProjectDetailView: View {
    let project: ProjectStore.Project

    @State private var reloaded: ProjectStore.Project?

    private var current: ProjectStore.Project { reloaded ?? project }

    var body: some View {
        List {
            Section {
                LabeledContent("Captures", value: "\(current.captures.count)")
                LabeledContent("Frames", value: "\(current.frameCount)")
                if current.rescanCount > 0 {
                    LabeledContent("Rooms rescanned",
                                   value: "\(current.rescanCount)")
                }
            } footer: {
                Text("Every capture here shares one coordinate frame, so they "
                   + "line up without alignment work later.")
            }

            Section {
                NavigationLink {
                    CaptureView(project: current, newProjectName: nil)
                } label: {
                    Label("Capture another room", systemImage: "camera.badge.ellipsis")
                }
                Button {
                    rescanName = ""
                    namingRescan = true
                } label: {
                    Label("Redo a room", systemImage: "arrow.clockwise")
                }
                if let latest = current.latestDirectory {
                    NavigationLink {
                        ProcessingView(sessionURL: latest)
                    } label: {
                        Label(current.hasExport
                              ? "Reconstruct and re-export"
                              : "Reconstruct and export",
                              systemImage: "cube.transparent")
                    }
                }
            } header: {
                Text("Continue")
            } footer: {
                Text("Redoing a room replaces the old frames wherever you "
                   + "walk. They stay on the phone — the reconstruction just "
                   + "stops using them.")
            }

            Section("Captures") {
                ForEach(current.captures) { capture in
                    captureRow(capture)
                }
            }
        }
        .navigationTitle(current.name)
        .navigationBarTitleDisplayMode(.inline)
        .onAppear {
            reloaded = ProjectStore.load().first { $0.id == project.id }
            // Readiness lives in the newest capture's report, since that is
            // the solve that covered the whole chain.
            if let latest = current.latestDirectory {
                report = SolveReport.read(sessionURL: latest)
            }
        }
        .sheet(isPresented: $namingRescan) { roomPicker }
        .navigationDestination(item: $startingRescan) { rescan in
            CaptureView(project: current, newProjectName: nil,
                        rescanLabel: rescan.label)
        }
    }

    /// Rooms from the last solve, worst first — the whole point is to pick
    /// the one that came out badly, so ranking by score is the answer to the
    /// question rather than an alphabetical list to read through.
    ///
    /// Falls back to naming it by hand when there is no report yet: a
    /// project that has never been reconstructed still has rooms, they just
    /// have no scores, and refusing to rescan until you have solved once
    /// would be a strange thing to insist on.
    private var roomPicker: some View {
        NavigationStack {
            List {
                let rooms = (report?.readiness?.regions ?? [])
                    .sorted { $0.score < $1.score }
                if rooms.isEmpty {
                    Section {
                        TextField("Room name (e.g. Kitchen)", text: $rescanName)
                        Button("Start") {
                            namingRescan = false
                            startingRescan = Rescan(
                                label: rescanName.isEmpty ? "this room"
                                                          : rescanName)
                        }
                    } footer: {
                        Text("No reconstruction yet, so there are no room "
                           + "scores to choose from. Name it and walk it.")
                    }
                } else {
                    Section {
                        ForEach(rooms) { room in
                            Button {
                                namingRescan = false
                                startingRescan = Rescan(label: room.name)
                            } label: {
                                roomRow(room)
                            }
                        }
                    } footer: {
                        Text("Scores are from the last reconstruction. Pick a "
                           + "room and walk it again — the new frames take "
                           + "over wherever you go.")
                    }
                }
            }
            .navigationTitle("Redo which room?")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                Button("Cancel") { namingRescan = false }
            }
        }
        .presentationDetents([.medium, .large])
    }

    private func roomRow(_ room: SolveReport.Readiness.Region) -> some View {
        HStack {
            VStack(alignment: .leading, spacing: 2) {
                Text(room.name).font(.headline)
                if let worst = room.worstAxisName {
                    Text("weakest: \(worst)")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
                Text(String(format: "%.1f m² · %d weak spot%@", room.areaM2,
                            Int(room.weakAreas), room.weakAreas == 1 ? "" : "s"))
                    .font(.caption2)
                    .foregroundStyle(.secondary)
            }
            Spacer()
            Text("\(Int(room.score))%")
                .font(.title3.weight(.semibold).monospacedDigit())
                .foregroundStyle(room.score >= 85 ? .green
                                 : (room.score >= 60 ? .orange : .red))
        }
        .padding(.vertical, 2)
    }

    @State private var namingRescan = false
    @State private var rescanName = ""
    @State private var startingRescan: Rescan?
    @State private var report: SolveReport?

    struct Rescan: Identifiable, Hashable {
        let label: String
        var id: String { label }
    }

    private func captureRow(_ capture: ProjectStore.Capture) -> some View {
        VStack(alignment: .leading, spacing: 3) {
            HStack {
                Text(capture.date.map {
                    $0.formatted(date: .abbreviated, time: .shortened)
                } ?? capture.sessionName)
                    .font(.subheadline)
                Spacer()
                Text("\(capture.frameCount)")
                    .font(.caption.monospacedDigit())
                    .foregroundStyle(coverageColor(capture.frameCount))
            }
            if !capture.supersedes.isEmpty {
                Label(capture.supersedes.map(\.label)
                        .filter { !$0.isEmpty }
                        .joined(separator: ", "),
                      systemImage: "arrow.clockwise")
                    .font(.caption2)
                    .foregroundStyle(.secondary)
            }
            // A thin capture is worth flagging here as well as during
            // capture: this is where someone decides what to redo, and
            // "which of these is weak" is exactly the question.
            if capture.frameCount < FrameFeedContext.storedFrameTarget
                && capture.frameCount > 0 {
                Text("Thin — under \(FrameFeedContext.storedFrameTarget) frames")
                    .font(.caption2)
                    .foregroundStyle(.orange)
            }
        }
        .padding(.vertical, 2)
    }

    private func coverageColor(_ frames: UInt32) -> Color {
        frames >= UInt32(FrameFeedContext.storedFrameTarget)
            ? .secondary : .orange
    }
}
