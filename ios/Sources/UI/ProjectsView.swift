import SwiftUI

/// The project board: every space you have captured, reopenable.
///
/// A project is the unit that matches how the work actually happens — a
/// house is not one capture, it is ten sittings over a space, and between
/// them the useful questions are "which room is thin" and "I want to redo
/// the kitchen". A flat list of dated session folders answers none of those.
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
            Text("You will scan with a live mesh so you can see coverage. "
               + "Photos go to the desktop for reconstruction.")
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
            ScanView(project: nil, newProjectName: request.name)
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

/// One project: its captures, and the two things you come back to do —
/// capture more, or redo a room.
struct ProjectDetailView: View {
    let project: ProjectStore.Project

    @State private var reloaded: ProjectStore.Project?
    @State private var rooms: [ProjectRoomRow] = []
    @State private var renaming: ProjectRoomRow?
    @State private var renameText = ""

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

            if !rooms.isEmpty {
                Section {
                    ForEach(rooms) { room in
                        Button {
                            renameText = room.name
                            renaming = room
                        } label: {
                            HStack {
                                Text(room.name)
                                    .foregroundStyle(.primary)
                                Spacer()
                                Text("\(room.captureCount) extras")
                                    .font(.caption.monospacedDigit())
                                    .foregroundStyle(.secondary)
                                Image(systemName: "pencil")
                                    .font(.caption)
                                    .foregroundStyle(.secondary)
                            }
                        }
                    }
                } header: {
                    Text("Rooms")
                } footer: {
                    Text("Tap a room to rename it. Those names become folder "
                       + "names when you send one folder per room.")
                }
            }

            Section {
                NavigationLink {
                    ScanView(project: current, newProjectName: nil)
                } label: {
                    Label("Scan more", systemImage: "cube.transparent")
                }
                ProjectSendToDesktopButton(
                    folders: current.captures.map(\.directory),
                    projectName: current.name)
            } header: {
                Text("Continue")
            } footer: {
                Text("Each scan adds photos to this project. Send packages "
                   + "images for desktop Depth Anything.")
            }

            Section("Captures") {
                ForEach(current.captures) { capture in
                    captureRow(capture)
                }
            }
        }
        .navigationTitle(current.name)
        .navigationBarTitleDisplayMode(.inline)
        .onAppear { reload() }
        .alert("Rename room", isPresented: Binding(
            get: { renaming != nil },
            set: { if !$0 { renaming = nil } })) {
            TextField("Name", text: $renameText)
            Button("Cancel", role: .cancel) { renaming = nil }
            Button("Save") {
                if let room = renaming {
                    RoomsDocument.rename(roomId: room.roomId,
                                         to: renameText,
                                         in: room.sessionDirectory)
                    reloadRooms()
                }
                renaming = nil
            }
        }
    }

    private func reload() {
        reloaded = ProjectStore.load().first { $0.id == project.id }
        reloadRooms()
    }

    private func reloadRooms() {
        var rows: [ProjectRoomRow] = []
        for capture in current.captures {
            let file = RoomsDocument.load(from: capture.directory)
            for room in file?.rooms ?? [] {
                rows.append(ProjectRoomRow(
                    sessionDirectory: capture.directory,
                    roomId: room.id,
                    name: room.name,
                    captureCount: room.captureCount))
            }
        }
        rooms = rows
    }

    struct ProjectRoomRow: Identifiable {
        let sessionDirectory: URL
        let roomId: UInt32
        var name: String
        var captureCount: UInt32
        var id: String {
            "\(sessionDirectory.lastPathComponent)-\(roomId)"
        }
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
                    .foregroundStyle(.secondary)
            }
        }
        .padding(.vertical, 2)
    }
}
