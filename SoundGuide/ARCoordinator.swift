import ARKit
import RealityKit
import SwiftUI
import UIKit

enum ObjectKind { case waypoint, obstacle }

struct Placed {
    let kind: ObjectKind
    let position: SIMD3<Float>
    let entities: [AnchorEntity]   // marker, plus the route line for waypoints
}

final class ARController: NSObject, ObservableObject, ARSessionDelegate {
    let arView = ARView(frame: .zero)
    var tuning = Tuning()

    @Published private(set) var placed: [Placed] = []
    @Published private(set) var guiding = false
    @Published private(set) var status = ""
    @Published private(set) var trackingNormal = false
    @Published private(set) var usingHeadTracking = false
    @Published private(set) var headphonesReady = false
    @Published private(set) var motionAccessDenied = false

    private let audio = SpatialAudio()
    private let head = HeadTracker()
    // Phone heading and AirPods yaw captured together; assumes you face along the phone at that moment.
    private var headRef: (phoneYaw: Float, headYaw: Double)?
    private var nav = Navigator()
    private var trackingLost = false
    private var lastHaptic: TimeInterval = 0
    private let tap = UIImpactFeedbackGenerator(style: .medium)
    private let light = UIImpactFeedbackGenerator(style: .light)
    private let warn = UIImpactFeedbackGenerator(style: .heavy)
    private let success = UINotificationFeedbackGenerator()

    var waypoints: [SIMD3<Float>] { placed.filter { $0.kind == .waypoint }.map(\.position) }
    var obstacles: [SIMD3<Float>] { placed.filter { $0.kind == .obstacle }.map(\.position) }

    override init() {
        super.init()
        arView.automaticallyConfigureSession = false
        arView.session.delegate = self
        let config = ARWorldTrackingConfiguration()
        config.planeDetection = [.horizontal, .vertical]
        arView.session.run(config)

        let coaching = ARCoachingOverlayView()
        coaching.session = arView.session
        coaching.goal = .anyPlane
        coaching.autoresizingMask = [.flexibleWidth, .flexibleHeight]
        arView.addSubview(coaching)
        if tuning.useHeadTracking { head.start() }
    }

    // MARK: Setup

    func place(_ kind: ObjectKind) {
        let center = CGPoint(x: arView.bounds.midX, y: arView.bounds.midY)
        guard let hit = arView.raycast(from: center, allowing: .estimatedPlane, alignment: .any).first else {
            status = "No surface under the crosshair. Move the phone slowly."
            return
        }
        let c = hit.worldTransform.columns.3
        let pos = SIMD3(c.x, c.y, c.z)

        let marker = AnchorEntity(world: pos)
        let mesh: MeshResource = kind == .waypoint ? .generateSphere(radius: 0.06) : .generateBox(size: 0.15)
        let color: UIColor = kind == .waypoint ? .systemGreen : .systemRed
        marker.addChild(ModelEntity(mesh: mesh, materials: [SimpleMaterial(color: color, isMetallic: false)]))
        arView.scene.addAnchor(marker)
        var entities = [marker]

        if kind == .waypoint, let prev = waypoints.last {
            let line = Self.line(from: prev, to: pos)
            arView.scene.addAnchor(line)
            entities.append(line)
        }
        placed.append(Placed(kind: kind, position: pos, entities: entities))
        status = kind == .waypoint ? "Waypoint \(waypoints.count)" : "Obstacle \(obstacles.count)"
        tap.impactOccurred()
    }

    func undo() {
        guard let last = placed.popLast() else { return }
        last.entities.forEach { arView.scene.removeAnchor($0) }
        status = ""
    }

    func clear() {
        placed.flatMap(\.entities).forEach { arView.scene.removeAnchor($0) }
        placed = []
        status = ""
    }

    private static func line(from a: SIMD3<Float>, to b: SIMD3<Float>) -> AnchorEntity {
        let anchor = AnchorEntity(world: .zero)
        let bar = ModelEntity(mesh: .generateBox(size: [0.01, 0.01, simd_distance(a, b)]),
                              materials: [SimpleMaterial(color: .systemGreen.withAlphaComponent(0.6), isMetallic: false)])
        bar.look(at: b, from: (a + b) / 2, relativeTo: nil)
        anchor.addChild(bar)
        return anchor
    }

    // MARK: Guide

    func startGuiding() {
        guard !waypoints.isEmpty else { status = "Place a waypoint first."; return }
        nav = Navigator()
        trackingLost = false
        lastHaptic = 0
        guiding = true
        headRef = nil
        UIApplication.shared.isIdleTimerDisabled = true
        audio.start()
        audio.say("Guiding to waypoint 1 of \(waypoints.count)")
        status = "Guiding to 1 of \(waypoints.count)"
    }

    func stopGuiding(announce: String? = nil) {
        guiding = false
        usingHeadTracking = false
        UIApplication.shared.isIdleTimerDisabled = false
        if let announce { audio.say(announce) } else { audio.stop() }
        status = announce ?? "Stopped"
    }

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        let normal: Bool
        if case .normal = frame.camera.trackingState { normal = true } else { normal = false }
        if normal != trackingNormal { trackingNormal = normal }
        let ready = head.isLive, denied = head.accessDenied
        if ready != headphonesReady { headphonesReady = ready }
        if denied != motionAccessDenied { motionAccessDenied = denied }
        guard guiding else { return }

        // Never guide from a stale pose.
        guard normal else {
            if !trackingLost { trackingLost = true; audio.say("Hold still"); status = "Tracking lost" }
            vibrate(.stop, now: frame.timestamp)
            return
        }
        trackingLost = false

        let m = frame.camera.transform
        var ear = SIMD3(m.columns.3.x, m.columns.3.y, m.columns.3.z)
        ear.y += tuning.earHeightAbovePhone
        let forward = listenerForward(phone: -SIMD3(m.columns.2.x, m.columns.2.y, m.columns.2.z))

        let wps = waypoints
        let out = nav.tick(listener: ear, forward: forward, now: frame.timestamp,
                           waypoints: wps, obstacles: obstacles, tuning: tuning)
        let index = out.index

        if out.finished {
            audio.playChime()
            success.notificationOccurred(.success)
            stopGuiding(announce: "Arrived")
            return
        }
        if out.arrived {
            audio.playChime()
            success.notificationOccurred(.success)
            audio.say("Reached \(index) of \(wps.count). Next.")
            status = "Guiding to \(index + 1) of \(wps.count)"
        }
        vibrate(out.haptic, now: frame.timestamp)

        audio.update(listener: ear, forward: forward, output: out, now: frame.timestamp)
    }
}

extension ARController {
    /// AirPods head direction when available, otherwise the phone's direction.
    fileprivate func listenerForward(phone: SIMD3<Float>) -> SIMD3<Float> {
        guard tuning.useHeadTracking, head.isLive, let headYaw = head.yaw else {
            headRef = nil   // re-align when AirPods come back
            setHeadTracking(false)
            return phone
        }
        if headRef == nil {
            // Can't read a heading off a phone pointing at the floor; wait for a usable frame.
            guard simd_length(SIMD2(phone.x, phone.z)) > 0.5 else { return phone }
            headRef = (Guidance.yaw(of: phone), headYaw)
        }
        setHeadTracking(true)
        let r = headRef!
        return Guidance.forward(yaw: r.phoneYaw + tuning.headYawSign * Float(headYaw - r.headYaw))
    }

    /// Throttled off the frame clock, like the audio pings.
    fileprivate func vibrate(_ haptic: Haptic, now: TimeInterval) {
        switch haptic {
        case .none: return
        case .slowDown(let interval):
            guard now - lastHaptic >= interval else { return }
            light.impactOccurred(intensity: 0.7)
        case .stop:
            guard now - lastHaptic >= tuning.stopPulseInterval else { return }
            warn.impactOccurred(intensity: 1)
        }
        lastHaptic = now
    }

    private func setHeadTracking(_ on: Bool) {
        guard on != usingHeadTracking else { return }
        usingHeadTracking = on
        if guiding { audio.say(on ? "Head tracking on" : "Using phone direction", interrupt: false) }
    }
}

struct ARViewContainer: UIViewRepresentable {
    let controller: ARController
    func makeUIView(context: Context) -> ARView { controller.arView }
    func updateUIView(_ uiView: ARView, context: Context) {}
}
