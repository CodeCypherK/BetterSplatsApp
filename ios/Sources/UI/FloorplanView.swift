import SwiftUI
import simd

/// Plan after the scout circuit: LiDAR occupancy of the space, plus rooms
/// the user draws on top. Rooms are never invented automatically.
struct FloorplanView: View {
    @Bindable var model: CaptureViewModel
    @State private var renaming: Floorplan.Room?
    @State private var renameText = ""
    @State private var pendingDelete: Floorplan.Room?
    @State private var namingNewRoom = false
    @State private var newRoomName = ""

    var body: some View {
        VStack(spacing: 0) {
            header
            planCanvas
                .frame(maxWidth: .infinity, maxHeight: .infinity)
            footer
        }
        .background(.black.opacity(0.92))
        .alert("Name this room", isPresented: $namingNewRoom) {
            TextField("Kitchen, living room…", text: $newRoomName)
            Button("Cancel", role: .cancel) { namingNewRoom = false }
            Button("Save") {
                model.closeDrawnRoom(named: newRoomName)
                namingNewRoom = false
            }
        } message: {
            Text("This name is what the folder will be called when you send "
               + "rooms to the desktop.")
        }
        .alert("Rename room", isPresented: Binding(
            get: { renaming != nil },
            set: { if !$0 { renaming = nil } })) {
            TextField("Name", text: $renameText)
            Button("Cancel", role: .cancel) { renaming = nil }
            Button("Save") {
                if let room = renaming, !renameText.isEmpty {
                    model.renameRoom(room.id, to: renameText)
                }
                renaming = nil
            }
        }
        .alert(deleteTitle, isPresented: Binding(
            get: { pendingDelete != nil },
            set: { if !$0 { pendingDelete = nil } })) {
            Button("Cancel", role: .cancel) { pendingDelete = nil }
            Button(deleteActionTitle, role: .destructive) {
                if let room = pendingDelete {
                    model.deleteRoom(room.id)
                }
                pendingDelete = nil
            }
        } message: {
            Text(deleteMessage)
        }
    }

    private var deleteTitle: String {
        guard let room = pendingDelete else { return "Remove room" }
        return room.scanned ? "Delete \(room.name)?" : "Remove \(room.name)?"
    }
    private var deleteActionTitle: String {
        (pendingDelete?.scanned ?? false) ? "Delete room and photos" : "Remove outline"
    }
    private var deleteMessage: String {
        guard let room = pendingDelete else { return "" }
        if room.scanned {
            return "Removes the outline and deletes \(room.captureCount) extra "
                 + "photos for this room. Route photos stay."
        }
        return "Removes the outline. Nothing has been scanned here yet."
    }

    private var header: some View {
        VStack(alignment: .leading, spacing: 4) {
            Text("The space")
                .font(.headline)
            Text("Gray is what LiDAR saw. Draw each room on top. "
               + "\(model.scoutFramesStored) route photos stay off the "
               + "room budget — each room gets up to "
               + "\(FrameFeedContext.roomCaptureCap) extra photos.")
                .font(.caption)
                .foregroundStyle(.secondary)
                .fixedSize(horizontal: false, vertical: true)
        }
        .padding(.horizontal, 16)
        .padding(.top, 12)
        .padding(.bottom, 8)
        .frame(maxWidth: .infinity, alignment: .leading)
    }

    private var planCanvas: some View {
        GeometryReader { geo in
            let plan = model.floorplan
            let bounds = plan?.bounds ?? (min: .zero, max: SIMD2(1, 1))
            Canvas { ctx, size in
                guard let plan else {
                    ctx.draw(Text("Walk a loop first.").foregroundColor(.white),
                             at: CGPoint(x: size.width / 2, y: size.height / 2))
                    return
                }
                let map = PlanMap(bounds: bounds, size: size)
                for ring in plan.footprints {
                    var path = Path()
                    let pts = ring.map { map.point($0) }
                    guard let first = pts.first, pts.count >= 3 else { continue }
                    path.move(to: first)
                    for p in pts.dropFirst() { path.addLine(to: p) }
                    path.closeSubpath()
                    ctx.fill(path, with: .color(.white.opacity(0.10)))
                    ctx.stroke(path, with: .color(.white.opacity(0.55)),
                               lineWidth: 1.6)
                }
                for room in plan.rooms {
                    var path = Path()
                    let pts = room.polygon.map { map.point($0) }
                    guard let first = pts.first, pts.count >= 3 else { continue }
                    path.move(to: first)
                    for p in pts.dropFirst() { path.addLine(to: p) }
                    path.closeSubpath()
                    let selected = room.id == model.selectedRoomId
                    ctx.fill(path, with: .color(roomColor(room.id)
                        .opacity(selected ? 0.45 : 0.22)))
                    ctx.stroke(path, with: .color(roomColor(room.id)),
                               lineWidth: selected ? 3 : 1.5)
                }
                if model.drawingVertices.count >= 1 {
                    var path = Path()
                    let pts = model.drawingVertices.map { map.point($0) }
                    path.move(to: pts[0])
                    for p in pts.dropFirst() { path.addLine(to: p) }
                    ctx.stroke(path, with: .color(.white),
                               style: StrokeStyle(lineWidth: 2, dash: [6, 4]))
                    for p in pts {
                        ctx.fill(Path(ellipseIn: CGRect(x: p.x - 4, y: p.y - 4,
                                                        width: 8, height: 8)),
                                 with: .color(.white))
                    }
                }
                // Camera path is not the plan — occupancy is. Keep a faint
                // trail so the walk is still visible if LiDAR was thin.
            }
            .contentShape(Rectangle())
            .onTapGesture { loc in
                let map = PlanMap(bounds: bounds, size: geo.size)
                let world = map.plan(loc)
                if model.isDrawingRoom {
                    model.addDrawPoint(world)
                } else if let plan,
                          let hit = plan.rooms.last(where: {
                              FloorplanBuilder.pointInPolygon(world, $0.polygon)
                          }) {
                    model.selectRoom(hit.id)
                }
            }
        }
        .background(Color.white.opacity(0.06))
        .clipShape(RoundedRectangle(cornerRadius: 12))
        .padding(.horizontal, 12)
    }

    private var footer: some View {
        VStack(spacing: 10) {
            if model.isDrawingRoom {
                Text(model.drawingVertices.count < 3
                     ? "Tap corners of the room on the plan"
                     : "\(model.drawingVertices.count) corners — close when the shape is right")
                    .font(.caption)
                    .foregroundStyle(.secondary)
                HStack {
                    Button("Cancel") { model.cancelDrawing() }
                    Spacer()
                    Button("Close room") {
                        let next = (model.floorplan?.rooms.map(\.id).max() ?? 0) + 1
                        newRoomName = "Room \(next)"
                        namingNewRoom = true
                    }
                        .disabled(model.drawingVertices.count < 3)
                        .buttonStyle(.borderedProminent)
                }
            } else {
                roomList
                Button {
                    model.beginDrawingRoom()
                } label: {
                    Label("Add a room", systemImage: "plus.viewfinder")
                        .frame(maxWidth: .infinity)
                }
                .buttonStyle(.bordered)

                if let id = model.selectedRoomId,
                   let room = model.floorplan?.rooms.first(where: { $0.id == id }) {
                    Button {
                        model.startRoomScan(id)
                    } label: {
                        Label("Scan \(room.name)", systemImage: "camera.viewfinder")
                            .frame(maxWidth: .infinity)
                    }
                    .buttonStyle(.borderedProminent)
                    .controlSize(.large)
                    Button {
                        renameText = room.name
                        renaming = room
                    } label: {
                        Label("Rename \(room.name)", systemImage: "pencil")
                            .frame(maxWidth: .infinity)
                    }
                    .buttonStyle(.bordered)
                    Button(role: .destructive) {
                        pendingDelete = room
                    } label: {
                        Label(room.scanned ? "Delete room and photos"
                                           : "Remove this room",
                              systemImage: "trash")
                    }
                    .font(.caption)
                }

                Button("End session") { model.stop() }
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
        }
        .padding(16)
    }

    @ViewBuilder
    private var roomList: some View {
        if let rooms = model.floorplan?.rooms, !rooms.isEmpty {
            VStack(alignment: .leading, spacing: 6) {
                ForEach(rooms) { room in
                    Button {
                        model.selectRoom(room.id)
                    } label: {
                        HStack {
                            Circle()
                                .fill(roomColor(room.id))
                                .frame(width: 10, height: 10)
                            Text(room.name)
                                .font(.subheadline.weight(
                                    room.id == model.selectedRoomId ? .semibold : .regular))
                            Spacer()
                            Text("\(room.scoutFrameIds.count) route · "
                               + "\(room.captureCount)/\(FrameFeedContext.roomCaptureCap)")
                                .font(.caption.monospacedDigit())
                                .foregroundStyle(.secondary)
                            if room.scanned {
                                Image(systemName: "checkmark.circle.fill")
                                    .foregroundStyle(.green)
                                    .font(.caption)
                            }
                        }
                    }
                    .buttonStyle(.plain)
                    .contextMenu {
                        Button("Rename") {
                            renameText = room.name
                            renaming = room
                        }
                        Button(role: .destructive) {
                            pendingDelete = room
                        } label: {
                            Label(room.scanned ? "Delete room and photos"
                                               : "Remove outline",
                                  systemImage: "trash")
                        }
                    }
                }
            }
        } else {
            Text("No rooms yet — tap Add a room and outline it on the plan.")
                .font(.caption)
                .foregroundStyle(.secondary)
        }
    }

    private func roomColor(_ id: UInt32) -> Color {
        let palette: [Color] = [.blue, .orange, .green, .purple, .pink, .teal, .yellow]
        return palette[Int(id) % palette.count]
    }
}

private struct PlanMap {
    let bounds: (min: SIMD2<Double>, max: SIMD2<Double>)
    let size: CGSize

    func point(_ p: SIMD2<Double>) -> CGPoint {
        let dx = max(0.5, bounds.max.x - bounds.min.x)
        let dy = max(0.5, bounds.max.y - bounds.min.y)
        let scale = min(Double(size.width) / dx, Double(size.height) / dy) * 0.9
        let cx = (bounds.min.x + bounds.max.x) / 2
        let cy = (bounds.min.y + bounds.max.y) / 2
        return CGPoint(
            x: size.width / 2 + (p.x - cx) * scale,
            y: size.height / 2 - (p.y - cy) * scale)
    }

    func plan(_ p: CGPoint) -> SIMD2<Double> {
        let dx = max(0.5, bounds.max.x - bounds.min.x)
        let dy = max(0.5, bounds.max.y - bounds.min.y)
        let scale = min(Double(size.width) / dx, Double(size.height) / dy) * 0.9
        let cx = (bounds.min.x + bounds.max.x) / 2
        let cy = (bounds.min.y + bounds.max.y) / 2
        return SIMD2(
            cx + (Double(p.x) - size.width / 2) / scale,
            cy - (Double(p.y) - size.height / 2) / scale)
    }
}
