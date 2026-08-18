import Foundation
import Network
import SwiftUI
import UIKit

/// Zips a session or project, then sends that one archive to the desktop
/// receiver over the same TCP protocol PhoneStreamer uses:
///
///   [u32 big-endian length][u8 type][payload]
///   type 20 file — [u16 pathLen][utf8 relPath][u32 size][bytes]
///   type 21 done — [u16 nameLen][utf8 packageName]
///
/// The payload is a single `.zip`, streamed in 1 MB chunks, so a 400-photo
/// room is one file on the wire instead of a thousand. The receiver writes
/// `incoming/<name>.zip` and unpacks it. PhoneStreamer's capture-server.cmd
/// speaks the same bytes.
@MainActor
final class DesktopSender: ObservableObject {
    static let shared = DesktopSender()

    /// Empty until the user types one. Never a baked-in address — PhoneStreamer's
    /// default Tailscale IP is not ours to assume.
    @Published var host: String {
        didSet { Self.saveHost(host) }
    }
    @Published var port: UInt16 {
        didSet { UserDefaults.standard.set(Int(port), forKey: Self.portKey) }
    }
    @Published var status: String?
    @Published var isSending = false

    private static let hostKey = "desktop_host"
    private static let portKey = "desktop_port"
    static let defaultPort: UInt16 = 9999

    private init() {
        host = Self.savedHost()
        let saved = UserDefaults.standard.integer(forKey: Self.portKey)
        port = saved == 0 ? Self.defaultPort : UInt16(saved)
    }

    private static func savedHost() -> String {
        UserDefaults.standard.string(forKey: hostKey)?
            .trimmingCharacters(in: .whitespacesAndNewlines) ?? ""
    }

    private static func saveHost(_ value: String) {
        let trimmed = value.trimmingCharacters(in: .whitespacesAndNewlines)
        if trimmed.isEmpty {
            UserDefaults.standard.removeObject(forKey: hostKey)
        } else {
            UserDefaults.standard.set(trimmed, forKey: hostKey)
        }
    }

    /// `nestUnderPackage` is true for a project (paths become
    /// `<project>/<session>/...`) and false for a single session (paths
    /// stay `<session>/...`).
    func send(folders: [URL], packageName: String,
              nestUnderPackage: Bool) {
        let name = Self.sanitize(packageName)
        guard !isSending else { return }
        guard !host.trimmingCharacters(in: .whitespaces).isEmpty else {
            status = "Set the PC IP on the home screen first"
            return
        }
        guard !folders.isEmpty else {
            status = "Nothing to send"
            return
        }

        isSending = true
        status = "Zipping…"
        UIApplication.shared.isIdleTimerDisabled = true

        let host = self.host.trimmingCharacters(in: .whitespaces)
        let port = self.port
        Task.detached(priority: .userInitiated) {
            let result = await Self.upload(
                folders: folders, packageName: name,
                nestUnderPackage: nestUnderPackage,
                host: host, port: port) { message in
                Task { @MainActor in
                    DesktopSender.shared.status = message
                }
            }
            await MainActor.run {
                DesktopSender.shared.isSending = false
                DesktopSender.shared.status = result
                UIApplication.shared.isIdleTimerDisabled = false
            }
        }
    }

    /// PhoneStreamer's path hygiene: keep letters, digits, `_`, `-`, `.`.
    static func sanitize(_ name: String) -> String {
        let trimmed = name.trimmingCharacters(in: .whitespacesAndNewlines)
        let mapped = trimmed.map { ch -> Character in
            if ch.isLetter || ch.isNumber || ch == "_" || ch == "-" || ch == "." {
                return ch
            }
            return "_"
        }
        let out = String(mapped)
        return out.isEmpty ? "capture" : out
    }

    // MARK: - Zip, then one type-20 file (off the main actor)

    nonisolated private static func upload(
        folders: [URL], packageName: String, nestUnderPackage: Bool,
        host: String, port: UInt16,
        progress: @escaping @Sendable (String) -> Void
    ) async -> String {
        progress("Zipping…")
        let zipURL: URL
        do {
            zipURL = try await makeZip(folders: folders, packageName: packageName,
                                       nestUnderPackage: nestUnderPackage)
        } catch {
            return "Zip failed: \(error.localizedDescription)"
        }
        defer { try? FileManager.default.removeItem(at: zipURL) }

        let size = (try? zipURL.resourceValues(forKeys: [.fileSizeKey]).fileSize) ?? 0
        guard size > 0 else { return "Zip is empty" }
        if size > Int(UInt32.max) {
            return "Zip is larger than 4 GB — the protocol cannot send it"
        }

        progress("Connecting…")
        let conn = NWConnection(
            host: NWEndpoint.Host(host),
            port: NWEndpoint.Port(rawValue: port) ?? 9999,
            using: .tcp)
        let sendQueue = DispatchQueue(label: "bs.desktop.send")
        let ready = DispatchSemaphore(value: 0)
        var connectError: String?
        conn.stateUpdateHandler = { state in
            switch state {
            case .ready:
                connectError = nil
                ready.signal()
            case .failed(let err):
                connectError = err.localizedDescription
                ready.signal()
            case .waiting(let err):
                connectError = err.localizedDescription
            default:
                break
            }
        }
        conn.start(queue: sendQueue)
        // PhoneStreamer waits ~0.8s for TCP to come up before the first file.
        let wait = ready.wait(timeout: .now() + 8)
        if wait == .timedOut {
            conn.cancel()
            return "Connect timed out — is capture_receiver.py running on \(host)?"
        }
        if let connectError {
            conn.cancel()
            return "Connect failed: \(connectError)"
        }
        // Extra beat so the first packet is not dropped on a slow handshake.
        Thread.sleep(forTimeInterval: 0.8)

        let sem = DispatchSemaphore(value: 0)
        var failed = false
        func sendChunk(_ chunk: Data) -> Bool {
            conn.send(content: chunk, completion: .contentProcessed { err in
                if err != nil { failed = true }
                sem.signal()
            })
            sem.wait()
            return !failed
        }

        let rel = packageName + ".zip"
        let relData = rel.data(using: .utf8) ?? Data()
        var header = Data()
        header.appendUInt32(UInt32(1 + 2 + relData.count + 4 + size))
        header.append(20)
        header.appendUInt16(UInt16(relData.count))
        header.append(relData)
        header.appendUInt32(UInt32(size))
        guard let fh = try? FileHandle(forReadingFrom: zipURL),
              sendChunk(header) else {
            conn.cancel()
            return "Upload failed mid-transfer"
        }
        var sent = 0
        var lastPct = -1
        while sent < size {
            guard let piece = try? fh.read(upToCount: 1 << 20),
                  !piece.isEmpty else { break }
            sent += piece.count
            guard sendChunk(piece) else { break }
            let pct = Int(Double(sent) / Double(size) * 100)
            if pct != lastPct {
                lastPct = pct
                progress("Sending \(pct)%")
            }
        }
        try? fh.close()
        if failed || sent != size {
            conn.cancel()
            return "Upload failed mid-transfer"
        }

        var done = Data([21])
        let nameData = packageName.data(using: .utf8) ?? Data()
        done.appendUInt16(UInt16(nameData.count))
        done.append(nameData)
        var packet = Data()
        packet.appendUInt32(UInt32(done.count))
        packet.append(done)
        conn.send(content: packet, completion: .contentProcessed { _ in sem.signal() })
        sem.wait()
        Thread.sleep(forTimeInterval: 1.0)
        conn.cancel()
        return "Sent — on the PC: incoming/\(packageName)"
    }

    /// One session folder zips as-is. A project is copied under a staging
    /// directory named after the project, then zipped, so the archive holds
    /// every capture.
    nonisolated private static func makeZip(
        folders: [URL], packageName: String, nestUnderPackage: Bool
    ) async throws -> URL {
        if folders.count == 1, !nestUnderPackage {
            return try await ZipExporter.zipDirectory(
                at: folders[0], name: packageName + ".zip")
        }
        let fm = FileManager.default
        let stageRoot = fm.temporaryDirectory
            .appendingPathComponent("desktop-stage-\(UUID().uuidString)",
                                    isDirectory: true)
        let stage = stageRoot.appendingPathComponent(packageName, isDirectory: true)
        try fm.createDirectory(at: stage, withIntermediateDirectories: true)
        defer { try? fm.removeItem(at: stageRoot) }
        for folder in folders {
            try fm.copyItem(
                at: folder,
                to: stage.appendingPathComponent(folder.lastPathComponent,
                                                 isDirectory: true))
        }
        return try await ZipExporter.zipDirectory(
            at: stage, name: packageName + ".zip")
    }
}

private extension Data {
    mutating func appendUInt16(_ value: UInt16) {
        var v = value.bigEndian
        Swift.withUnsafeBytes(of: &v) { append(contentsOf: $0) }
    }

    mutating func appendUInt32(_ value: UInt32) {
        var v = value.bigEndian
        Swift.withUnsafeBytes(of: &v) { append(contentsOf: $0) }
    }
}

/// Button + live status, used on both the session row and the project page.
struct SendToDesktopButton: View {
    @ObservedObject private var sender = DesktopSender.shared
    let folders: [URL]
    let packageName: String
    var nestUnderPackage: Bool = false
    var title: String = "Send to desktop"
    var showStatus: Bool = true

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            Button {
                sender.send(folders: folders, packageName: packageName,
                            nestUnderPackage: nestUnderPackage)
            } label: {
                Label(sender.isSending ? "Sending…" : title,
                      systemImage: "desktopcomputer")
            }
            .disabled(sender.isSending || folders.isEmpty)
            if showStatus, let status = sender.status {
                Text(status)
                    .font(.caption)
                    .foregroundStyle(.secondary)
                    .fixedSize(horizontal: false, vertical: true)
            }
        }
    }
}
