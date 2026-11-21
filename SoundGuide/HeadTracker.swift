import CoreMotion

/// AirPods head yaw. Runs for the app's lifetime so the setup screen can show whether they're connected.
/// Yaw is relative to wherever the head pointed when updates started,
/// so ARController aligns it to the room once per guiding session.
final class HeadTracker {
    private let manager = CMHeadphoneMotionManager()
    private(set) var yaw: Double?
    private var lastUpdate = Date.distantPast

    /// False when no supported AirPods are connected (updates simply never arrive).
    var isLive: Bool { yaw != nil && Date().timeIntervalSince(lastUpdate) < 0.5 }

    func start() {
        guard manager.isDeviceMotionAvailable, !manager.isDeviceMotionActive else { return }
        manager.startDeviceMotionUpdates(to: .main) { [weak self] motion, _ in
            guard let self, let motion else { return }
            yaw = motion.attitude.yaw
            lastUpdate = Date()
        }
    }

    var accessDenied: Bool { CMHeadphoneMotionManager.authorizationStatus() == .denied }
}
