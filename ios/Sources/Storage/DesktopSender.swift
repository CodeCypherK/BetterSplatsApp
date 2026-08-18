import Foundation
import Network
import SwiftUI
import UIKit

/// Sends a session or project folder to the desktop receiver over the same
/// TCP protocol PhoneStreamer uses:
///
///   [u32 big-endian length][u8 type][payload]
///   type 20 file — [u16 pathLen][utf8 relPath][u32 size][bytes]
///   type 21 done — [u16 nameLen][utf8 packageName]
///
/// Files are streamed in 1 MB chunks. Relative paths keep the folder tree,
/// so a session lands as `incoming/<session>/...` and a project as
/// `incoming/<project>/<session>/...`. The desktop side is
/// `tools/desktop/capture_receiver.py` (or PhoneStreamer's capture-server.cmd
/// — same bytes on the wire).
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
        status = "Connecting…"
        UIApplication.shared.isIdleTimerDisabled = true

        let host = self.host.trimmingCharacters(in: .whitespaces)
        let port = self.port
        Task.detached(priority: .userInitiated) {
            let result = Self.upload(
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

    // MARK: - Wire protocol (off the main actor, identical to PhoneStreamer)

    private struct Item {
        let url: URL
        let relative: String
        let size: Int
    }

    nonisolated private static func upload(
        folders: [URL], packageName: String, nestUnderPackage: Bool,
        host: String, port: UInt16,
        progress: @escaping @Sendable (String) -> Void
    ) -> String {
        var items: [Item] = []
        let fm = FileManager.default
        for folder in folders {
            let resolved = folder.resolvingSymlinksInPath()
            guard let enumerator = fm.enumerator(
                at: resolved,
                includingPropertiesForKeys: [.isRegularFileKey, .fileSizeKey]
            ) else { continue }
            for case let file as URL in enumerator {
                guard (try? file.resourceValues(forKeys: [.isRegularFileKey])
                    .isRegularFile) == true else { continue }
                let size = (try? file.resourceValues(forKeys: [.fileSizeKey])
                    .fileSize) ?? 0
                guard size > 0 else { continue }
                if size > Int(UInt32.max) {
                    return "A file is larger than 4 GB — the protocol cannot send it"
                }
                var rel = file.resolvingSymlinksInPath().path
                let base = resolved.deletingLastPathComponent()
                    .resolvingSymlinksInPath().path
                if rel.hasPrefix(base + "/") {
                    rel = String(rel.dropFirst(base.count + 1))
                } else {
                    rel = resolved.lastPathComponent + "/" + file.lastPathComponent
                }
                // A project send prefixes every session path with the package
                // name so the receiver writes incoming/<project>/<session>/…
                if nestUnderPackage {
                    rel = packageName + "/" + rel
                }
                items.append(Item(url: file, relative: rel, size: size))
            }
        }
        guard !items.isEmpty else { return "Folder is empty" }

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

        for (i, item) in items.enumerated() {
            guard let fh = try? FileHandle(forReadingFrom: item.url) else { continue }
            let relData = item.relative.data(using: .utf8) ?? Data()
            var header = Data()
            header.appendUInt32(UInt32(1 + 2 + relData.count + 4 + item.size))
            header.append(20)
            header.appendUInt16(UInt16(relData.count))
            header.append(relData)
            header.appendUInt32(UInt32(item.size))
            guard sendChunk(header) else {
                try? fh.close()
                conn.cancel()
                return "Upload failed mid-transfer"
            }
            var sent = 0
            while sent < item.size {
                guard let piece = try? fh.read(upToCount: 1 << 20),
                      !piece.isEmpty else { break }
                sent += piece.count
                guard sendChunk(piece) else { break }
            }
            try? fh.close()
            if failed || sent != item.size {
                conn.cancel()
                return "Upload failed mid-transfer"
            }
            let pct = Int(Double(i + 1) / Double(items.count) * 100)
            progress("Sending \(pct)% (\(i + 1)/\(items.count))")
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
