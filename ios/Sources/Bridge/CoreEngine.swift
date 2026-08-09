import Foundation

/// Thin Swift facade over the C engine ABI (`bs_api.h`). One instance per
/// process; all heavy work happens on engine-owned threads behind the ABI.
final class CoreEngine: @unchecked Sendable {
    static let shared = CoreEngine()

    private let handle: OpaquePointer?

    private init() {
        handle = bs_create("{}")
    }

    deinit {
        bs_destroy(handle)
    }

    var isAvailable: Bool { handle != nil }

    static var version: String {
        String(cString: bs_version())
    }

    var lastError: String {
        String(cString: bs_last_error(handle))
    }

    /// Runs tiny known-answer problems through Eigen, OpenCV and Ceres on
    /// this device. Surfaced on the diagnostics screen so a side-loaded
    /// build can prove its dependency chain in the field.
    func selftest() -> (ok: Bool, report: String) {
        var buf = [CChar](repeating: 0, count: 512)
        let result = bs_selftest(&buf, buf.count)
        return (result == BS_OK, String(cString: buf))
    }
}
