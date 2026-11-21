import ARKit
import RealityKit
import SwiftUI
import UIKit
import CoreML
import Vision
import os

/// Construct and use exclusively on the serial Vision queue.
final class YOLO26Detector {
    private let request: VNCoreMLRequest

    enum InferenceError: Error { case unexpectedResults }

    init(configuration: MLModelConfiguration = MLModelConfiguration()) throws {
        let model = try VNCoreMLModel(for: YOLO26n(configuration: configuration).model)
        request = VNCoreMLRequest(model: model)
        // The model's portrait input approximates the camera aspect ratio. Preserve the
        // whole image so normalized Vision coordinates map directly back to the sensor.
        request.imageCropAndScaleOption = .scaleFill
    }

    func detect(in image: CVPixelBuffer, orientation: CGImagePropertyOrientation) throws -> [VNRecognizedObjectObservation] {
        try VNImageRequestHandler(cvPixelBuffer: image, orientation: orientation).perform([request])
        guard let objects = request.results as? [VNRecognizedObjectObservation] else {
            throw InferenceError.unexpectedResults
        }
        return objects
    }
}

enum ObjectKind { case waypoint, obstacle }

struct Placed {
    let kind: ObjectKind
    let position: SIMD3<Float>
    var obstacle: Obstacle?
    let entities: [AnchorEntity]   // marker, plus the route line for waypoints
}

struct Detected: Equatable {
    enum Source: Equatable { case plane(UUID), points, object }
    let obstacle: Obstacle
    let source: Source
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
    @Published private(set) var map: GuideMap?
    // Beacon pings and spoken directions only play while this is on. Safety sounds always play.
    @Published private(set) var guideAudioOn = false
    private var lastMapUpdate: TimeInterval = 0
    @Published var autoDetect = true {
        didSet {
            guard autoDetect != oldValue else { return }
            detectorGeneration &+= 1
            if !autoDetect {
                navigationObstacles = []
                setDetected([])
            }
        }
    }
    @Published private(set) var detectedCount = 0
    private var detected: [Detected] = []

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
    // Obstacle being dragged out: first corner, phone heading at press, live preview.
    private var drag: (corner: SIMD3<Float>, yaw: Float, last: SIMD3<Float>, preview: AnchorEntity)?
    private var pointMap = PointMap()
    private var ignoredPlanes = Set<UUID>()
    private var lastBuild: TimeInterval = 0
    private var floorY: Float?
    // Object detector: runs off the main thread, one image at a time.
    private static let detectorLog = Logger(subsystem: Bundle.main.bundleIdentifier ?? "SoundGuide", category: "YOLO26")
    private let visionQueue = DispatchQueue(label: "detector", qos: .userInitiated)
    private var detector: YOLO26Detector?
    private var detectorGeneration = 0
    private var detectorBusy = false
    private var lastDetectorRun = -TimeInterval.infinity
    private var tracks = ObjectTracks()
    private var navigationObstacles: [Obstacle] = []
    private var geometryHitboxes: [Hitbox] = []
    private var modelHitboxes: [Hitbox] = []
    private lazy var hitboxMesh = MeshResource.generateBox(size: 1)

    var waypoints: [SIMD3<Float>] { placed.filter { $0.kind == .waypoint }.map(\.position) }
    var obstacles: [Obstacle] { placed.compactMap(\.obstacle) + navigationObstacles }

    override init() {
        super.init()
        arView.automaticallyConfigureSession = false
        arView.session.delegate = self
        let config = ARWorldTrackingConfiguration()
        config.planeDetection = [.horizontal, .vertical]
        arView.session.run(config)

        // Show the live camera and setup UI while world tracking initializes.
        if tuning.useHeadTracking { head.start() }
        visionQueue.async { [weak self] in
            do {
                let detector = try YOLO26Detector()
                DispatchQueue.main.async { self?.detector = detector }
            } catch {
                Self.detectorLog.error("Could not load YOLO26: \(error.localizedDescription)")
            }
        }
    }

    // MARK: Setup

    private func crosshairHit() -> SIMD3<Float>? {
        let center = CGPoint(x: arView.bounds.midX, y: arView.bounds.midY)
        guard let c = arView.raycast(from: center, allowing: .estimatedPlane, alignment: .any).first?.worldTransform.columns.3
        else { return nil }
        return SIMD3(c.x, c.y, c.z)
    }

    private func noSurface() { status = "No surface under the crosshair. Move the phone slowly." }

    func placeWaypoint() {
        guard let pos = crosshairHit() else { return noSurface() }
        let marker = AnchorEntity(world: pos)
        marker.addChild(ModelEntity(mesh: .generateSphere(radius: 0.06),
                                    materials: [SimpleMaterial(color: .systemGreen, isMetallic: false)]))
        arView.scene.addAnchor(marker)
        var entities = [marker]
        if let prev = waypoints.last {
            let line = Self.line(from: prev, to: pos)
            arView.scene.addAnchor(line)
            entities.append(line)
        }
        placed.append(Placed(kind: .waypoint, position: pos, entities: entities))
        status = "Waypoint \(waypoints.count)"
        tap.impactOccurred()
    }

    /// Press on the Obstacle button: this corner. Release: the opposite corner. A quick tap gives a small box.
    func beginObstacle() {
        guard drag == nil else { return }
        guard let a = crosshairHit() else { return noSurface() }
        var yaw: Float = 0   // ponytail: phone pointing straight down has no heading; falls back to world axes
        if let m = arView.session.currentFrame?.camera.transform {
            let f = -SIMD3(m.columns.2.x, 0, m.columns.2.z)
            if simd_length(f) > 0.3 { yaw = Guidance.yaw(of: f) }
        }
        let preview = Self.box(Obstacle.fromCorners(a, a, yaw: yaw, tuning: tuning), color: .systemRed.withAlphaComponent(0.4))
        arView.scene.addAnchor(preview)
        drag = (a, yaw, a, preview)
        status = "Move to the opposite corner, then let go"
        tap.impactOccurred()
    }

    func endObstacle() {
        guard let d = drag else { return }
        drag = nil
        arView.scene.removeAnchor(d.preview)
        let b = crosshairHit() ?? d.last
        let o = Obstacle.fromCorners(d.corner, b, yaw: d.yaw, tuning: tuning)
        let marker = Self.box(o, color: .systemRed.withAlphaComponent(0.8))
        arView.scene.addAnchor(marker)
        placed.append(Placed(kind: .obstacle, position: o.center, obstacle: o, entities: [marker]))
        status = String(format: "Obstacle %d: %.1f × %.1f m", placed.compactMap(\.obstacle).count, o.halfSize.x * 2, o.halfSize.y * 2)
        tap.impactOccurred()
    }

    private func updateDragPreview() {
        guard let d = drag, let b = crosshairHit() else { return }
        drag?.last = b
        let o = Obstacle.fromCorners(d.corner, b, yaw: d.yaw, tuning: tuning)
        d.preview.transform = Self.boxTransform(o)
    }

    /// Unit cube scaled to the obstacle: a 10 cm tall slab so it doesn't hide what's behind it.
    private static func box(_ o: Obstacle, color: UIColor) -> AnchorEntity {
        let anchor = AnchorEntity(world: .zero)
        let cube = ModelEntity(mesh: .generateBox(size: 1), materials: [SimpleMaterial(color: color, isMetallic: false)])
        anchor.addChild(cube)
        anchor.transform = boxTransform(o)
        return anchor
    }

    private static func boxTransform(_ o: Obstacle) -> Transform {
        // Rotating about +y by yaw turns -z into Guidance.forward(yaw), so the box lines up with the obstacle's axes.
        Transform(scale: SIMD3(o.halfSize.x * 2, 0.1, o.halfSize.y * 2),
                  rotation: simd_quatf(angle: o.yaw, axis: [0, 1, 0]),
                  translation: o.center + [0, 0.05, 0])
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

    // MARK: Automatic obstacles

    private func detect(_ frame: ARFrame) {
        let m = frame.camera.transform
        if let pc = frame.rawFeaturePoints {
            pointMap.add(pc.points, ids: pc.identifiers, camera: SIMD3(m.columns.3.x, m.columns.3.y, m.columns.3.z),
                         now: frame.timestamp, tuning: tuning)
        }
        runDetector(frame)
        guard frame.timestamp - lastBuild >= tuning.detectInterval else { return }
        lastBuild = frame.timestamp

        let planes = frame.anchors.compactMap { $0 as? ARPlaneAnchor }
        let ground = planes.filter { a in
            if case .ceiling = a.classification { return false }
            return a.alignment == .horizontal
        }
        let floors = ground.filter { if case .floor = $0.classification { return true }; return false }
        guard let floorY = (floors.isEmpty ? ground : floors).map({ Self.corners($0)[0].y }).min() else { return }
        self.floorY = floorY

        let planeBoxes = planes.compactMap { a -> Detected? in
            guard !ignoredPlanes.contains(a.identifier) else { return nil }
            return Detection.planeObstacle(corners: Self.corners(a), vertical: a.alignment == .vertical,
                                           floorY: floorY, tuning: tuning).map { Detected(obstacle: $0, source: .plane(a.identifier)) }
        }
        // Points on a table top would only duplicate the table.
        let pointBoxes = pointMap.build(floorY: floorY, now: frame.timestamp, tuning: tuning)
            .filter { b in !planeBoxes.contains { $0.obstacle.distance(to: xz(b.center)) == 0 } }
            .map { Detected(obstacle: $0, source: .points) }
        let boxes = planeBoxes + pointBoxes + trackedObjects
        // Keep route planning on its original cadence; render YOLO results as they arrive.
        navigationObstacles = boxes.map { $0.obstacle.navigationFootprint }
        setDetected(boxes)
    }

    private var trackedObjects: [Detected] {
        tracks.obstacles(tuning: tuning).map { Detected(obstacle: $0, source: .object) }
    }

    /// YOLO boxes → floor footprints, using the pose the image was taken at. Labels are ignored, except people:
    /// ponytail: people move, so they never enter the map; time-to-collision on live boxes if walkers need them.
    private func runDetector(_ frame: ARFrame) {
        guard let model = detector, let floorY, !detectorBusy else { return }
        let thermal = ProcessInfo.processInfo.thermalState
        if thermal == .serious || thermal == .critical {
            guard frame.timestamp - lastDetectorRun >= tuning.hotObjectInterval else { return }
        }
        detectorBusy = true
        lastDetectorRun = frame.timestamp
        let cam = frame.camera, image = frame.capturedImage, t = tuning
        let generation = detectorGeneration, timestamp = frame.timestamp
        let features = frame.rawFeaturePoints?.points ?? []
        let shot = CameraShot(transform: cam.transform, intrinsics: cam.intrinsics,
                              resolution: SIMD2(Float(cam.imageResolution.width), Float(cam.imageResolution.height)))
        visionQueue.async { [weak self] in
            let found: [Obstacle]?
            do {
                found = try model.detect(in: image, orientation: .right).compactMap { o in
                    guard let label = o.labels.first, label.confidence >= t.objectConfidence,
                          label.identifier != "person" else { return nil }
                    return Detection.objectFootprint(minX: Float(o.boundingBox.minX), maxX: Float(o.boundingBox.maxX),
                                                     minY: Float(o.boundingBox.minY), maxY: Float(o.boundingBox.maxY),
                                                     shot, floorY: floorY, tuning: t, features: features)
                }
            } catch {
                Self.detectorLog.error("YOLO26 inference failed: \(error.localizedDescription)")
                found = nil   // A failed request is not evidence that the room is empty.
            }
            DispatchQueue.main.async {
                guard let self else { return }
                self.detectorBusy = false
                guard let found, self.autoDetect, self.trackingNormal, self.detectorGeneration == generation else { return }
                self.tracks.update(found, shot: shot, now: timestamp, tuning: t)
                self.setDetected(self.detected.filter { $0.source != .object } + self.trackedObjects)
            }
        }
    }

    /// Reuse the outline's anchor, mesh and edges; inference only moves or resizes them.
    private final class Hitbox {
        let anchor = AnchorEntity(world: .zero)
        private let frame = Entity()
        private let edges: [ModelEntity]
        private var obstacle: Obstacle?

        init(mesh: MeshResource, color: UIColor) {
            let material = UnlitMaterial(color: color)
            edges = (0..<12).map { _ in ModelEntity(mesh: mesh, materials: [material]) }
            edges.forEach { frame.addChild($0) }
            anchor.addChild(frame)
        }

        func update(_ o: Obstacle) {
            guard obstacle != o else { return }
            obstacle = o
            let y = o.displayY ?? o.center.y...(o.center.y + 0.1)
            let half = SIMD3(o.halfSize.x, (y.upperBound - y.lowerBound) / 2, o.halfSize.y)
            frame.position = SIMD3(o.center.x, y.lowerBound + half.y, o.center.z)
            frame.orientation = simd_quatf(angle: o.yaw, axis: [0, 1, 0])
            let thickness: Float = 0.015
            for axis in 0..<3 {
                var size = SIMD3<Float>(repeating: thickness)
                size[axis] = half[axis] * 2 + thickness
                let a = (axis + 1) % 3, b = (axis + 2) % 3
                for i in 0..<4 {
                    let edge = edges[axis * 4 + i]
                    var position = SIMD3<Float>.zero
                    position[a] = i < 2 ? -half[a] : half[a]
                    position[b] = i.isMultiple(of: 2) ? -half[b] : half[b]
                    edge.scale = size
                    edge.position = position
                }
            }
        }
    }

    private static func updateHitboxes(_ hitboxes: inout [Hitbox], with obstacles: [Obstacle],
                                      mesh: MeshResource, color: UIColor, scene: RealityKit.Scene) {
        while hitboxes.count > obstacles.count { scene.removeAnchor(hitboxes.removeLast().anchor) }
        for (i, obstacle) in obstacles.enumerated() {
            if i == hitboxes.count {
                let box = Hitbox(mesh: mesh, color: color)
                hitboxes.append(box)
                scene.addAnchor(box.anchor)
            }
            hitboxes[i].update(obstacle)
        }
    }

    private func setDetected(_ d: [Detected]) {
        guard d != detected else { return }
        detected = d
        if detectedCount != d.count { detectedCount = d.count }
        let mesh = hitboxMesh, scene = arView.scene
        Self.updateHitboxes(&geometryHitboxes, with: d.filter { $0.source != .object }.map(\.obstacle),
                            mesh: mesh, color: .systemOrange, scene: scene)
        Self.updateHitboxes(&modelHitboxes, with: d.filter { $0.source == .object }.map(\.obstacle),
                            mesh: mesh, color: .systemYellow, scene: scene)
    }

    /// Deletes the detected obstacle under the crosshair, and keeps it from coming back.
    /// ponytail: a removed plane is ignored by anchor ID; if ARKit later merges it into another plane, Remove again.
    func removeDetected() {
        guard let hit = crosshairHit() else { return noSurface() }
        let p = xz(hit)
        guard let d = detected.min(by: { $0.obstacle.distance(to: p) < $1.obstacle.distance(to: p) }),
              d.obstacle.distance(to: p) < tuning.removeRadius
        else { status = "No detected obstacle under the crosshair"; return }
        let r = d.obstacle.boundingRadius + tuning.detectCell
        switch d.source {
        case .plane(let id): ignoredPlanes.insert(id)
        case .points: pointMap.ignore(around: xz(d.obstacle.center), radius: r)
        case .object: tracks.ignore(around: xz(d.obstacle.center), radius: r)
        }
        let remaining = detected.filter { $0 != d }
        navigationObstacles = remaining.map { $0.obstacle.navigationFootprint }
        setDetected(remaining)
        status = "Removed detected obstacle"
        tap.impactOccurred()
    }

    /// World corners of the plane's extent, in order around it.
    private static func corners(_ a: ARPlaneAnchor) -> [SIMD3<Float>] {
        let e = a.planeExtent, r = simd_quatf(angle: e.rotationOnYAxis, axis: [0, 1, 0])
        let signs: [SIMD2<Float>] = [[-1, -1], [1, -1], [1, 1], [-1, 1]]
        return signs.map { s in
            let w = a.transform * SIMD4(a.center + r.act(SIMD3(s.x * e.width / 2, 0, s.y * e.height / 2)), 1)
            return SIMD3(w.x, w.y, w.z)
        }
    }

    // MARK: Guide

    func startGuiding() {
        guard !waypoints.isEmpty else { status = "Place a waypoint first."; return }
        nav = Navigator()
        map = nil
        guideAudioOn = false
        trackingLost = false
        lastHaptic = 0
        guiding = true
        headRef = nil
        UIApplication.shared.isIdleTimerDisabled = true
        audio.start()
        audio.say("Guiding to waypoint 1 of \(waypoints.count). Press Play to hear the guide.")
        status = "Guiding to 1 of \(waypoints.count). Guide paused"
    }

    func toggleGuideAudio() {
        guideAudioOn.toggle()
        if guideAudioOn { nav.requestCallout() } else { audio.say("Guide paused") }
        tap.impactOccurred()
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
        if normal != trackingNormal {
            if !normal { detectorGeneration &+= 1 }
            trackingNormal = normal
        }
        let ready = head.isLive, denied = head.accessDenied
        if ready != headphonesReady { headphonesReady = ready }
        if denied != motionAccessDenied { motionAccessDenied = denied }
        if drag != nil { updateDragPreview() }
        if autoDetect && normal { detect(frame) }
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

        if let i = out.hitObstacle {
            success.notificationOccurred(.error)
            audio.say("Bump")
            status = "Hit obstacle \(i + 1). \(nav.hits) hit\(nav.hits == 1 ? "" : "s") so far"
        }
        if out.finished {
            audio.playChime()
            success.notificationOccurred(.success)
            let hits = nav.hits
            stopGuiding(announce: hits == 0 ? "Arrived, no obstacles hit" : "Arrived, \(hits) obstacle hit\(hits == 1 ? "" : "s")")
            return
        }
        if out.arrived {
            audio.playChime()
            success.notificationOccurred(.success)
            audio.say("Reached \(index) of \(wps.count). Next.")
            status = "Guiding to \(index + 1) of \(wps.count)"
        }
        if guideAudioOn, let callout = out.callout {
            audio.say(callout, interrupt: false)
            status = callout
        }
        vibrate(out.haptic, now: frame.timestamp)

        var heard = out
        if !guideAudioOn { heard.beaconPosition = nil }   // no pings; obstacle buzz still plays
        audio.update(listener: ear, forward: forward, output: heard, now: frame.timestamp)

        if frame.timestamp - lastMapUpdate > 0.1 || out.hitObstacle != nil {   // ~10 Hz is plenty for a map
            lastMapUpdate = frame.timestamp
            map = GuideMap(me: xz(ear), heading: Guidance.yaw(of: forward), waypoints: wps.map(xz), index: index,
                           obstacles: obstacles, path: nav.path ?? [], touching: nav.touching,
                           everHit: nav.everHit, hits: nav.hits)
        }
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
