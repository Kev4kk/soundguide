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

struct Detected: Equatable {
    enum Source: Equatable { case plane(UUID), points, object }
    let obstacle: Obstacle
    let source: Source
    var trackID: Int? = nil
}

final class ARController: NSObject, ObservableObject, ARSessionDelegate {
    let arView = ARView(frame: .zero)
    var tuning = Tuning()

    @Published private(set) var status = ""
    @Published private(set) var trackingNormal = false
    @Published private(set) var cameraStatus = "Starting object detection…"
    private var beepTiming = ProximityBeepTiming()
    @Published private(set) var proximityStatus = "Distance not established"
    private var liveRange = LiveRange()
    private var lastRangeSample = -Double.infinity
    private var lastRangeHit = -Double.infinity
    private var rangeDistance: Float = 0
    private var lastHead: (ear: SIMD3<Float>, forward: SIMD3<Float>)?   // last frame's listener while AirPods steer it
    private var classifiedFloor: Float?
    @Published private(set) var usingHeadTracking = false
    @Published private(set) var motionAccessDenied = false
    @Published private(set) var map: GuideMap?
    private var lastMapUpdate: TimeInterval = 0
    @Published var autoDetect = true {
        didSet {
            guard autoDetect != oldValue else { return }
            detectorGeneration &+= 1
            resetRange()
            cameraStatus = autoDetect ? "Starting object detection…" : "Object detection off"
            if !autoDetect {
                navigationObstacles = []
                setDetected([])
            }
        }
    }
    @Published private(set) var detectedCount = 0
    private var detected: [Detected] = []

    private let audio = SpatialAudio()
    private let web = WebViewer()
    private var lastWebUpdate: TimeInterval = 0
    var webURLs: [String] { WebViewer.addresses.map { "http://\($0):\(WebViewer.port)" } }
    lazy var asker = Asker { [weak self] in self?.audio.say($0, interrupt: false) }
    private let head = HeadTracker()
    // Phone heading and AirPods yaw captured together; assumes you face along the phone at that moment.
    private var headRef: (phoneYaw: Float, headYaw: Double)?
    private var motion = VelocityTracker()
    private var lastHaptic: TimeInterval = 0
    private var lastHapticWasStop = false
    private var rangeHaptic = Haptic.none
    private let tap = UIImpactFeedbackGenerator(style: .medium)
    private let stopBuzz = UINotificationFeedbackGenerator()
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


    override init() {
        super.init()
        arView.automaticallyConfigureSession = false
        arView.session.delegate = self
        let config = ARWorldTrackingConfiguration()
        config.planeDetection = [.horizontal, .vertical]
        arView.session.run(config)

        if tuning.useHeadTracking { head.start() }
        web.start()
        UIApplication.shared.isIdleTimerDisabled = true   // the walker never touches the screen to keep it awake
        audio.start()
        asker.prepare()
        visionQueue.async { [weak self] in
            do {
                let detector = try YOLO26Detector()
                DispatchQueue.main.async { self?.detector = detector }
            } catch {
                Self.detectorLog.error("Could not load YOLO26: \(error.localizedDescription)")
                DispatchQueue.main.async {
                    self?.cameraStatus = "Object detection unavailable"
                }
            }
        }
    }

    // MARK: Automatic obstacles

    private func detect(_ frame: ARFrame) {
        let m = frame.camera.transform
        if let pc = frame.rawFeaturePoints {
            pointMap.add(pc.points, ids: pc.identifiers, camera: SIMD3(m.columns.3.x, m.columns.3.y, m.columns.3.z),
                         now: frame.timestamp, tuning: tuning)
        }
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
        classifiedFloor = floors.map { Self.corners($0)[0].y }.min()

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
        tracks.confirmed(tuning: tuning).map { Detected(obstacle: $0.obstacle, source: .object, trackID: $0.id) }
    }

    /// Match same-class footprints using the image's own pose. People mask visibility but never enter the persistent map.
    private func runDetector(_ frame: ARFrame) {
        guard let model = detector, !detectorBusy else { return }
        let mappingFloor = trackingNormal ? floorY : nil
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
        let previousTracks = tracks
        visionQueue.async { [weak self] in
            let found: [ObjectObservation]?
            var masks: [CGRect] = []
            let cameraMessage: String?
            do {
                let boxes = try model.detect(in: image, orientation: .right)
                let confident = boxes.filter { ($0.labels.first?.confidence ?? 0) >= t.objectConfidence }
                let objects = confident.compactMap { o in o.labels.first.map { CameraObject(label: $0.identifier, bounds: o.boundingBox) } }
                cameraMessage = CameraObject.primary(objects)?.displayText
                masks = confident.map(\.boundingBox)
                found = confident.compactMap { o in
                    guard let floorY = mappingFloor else { return nil }
                    guard let label = o.labels.first, label.confidence >= t.objectConfidence,
                          label.identifier != "person" else { return nil }
                    guard let fit = Detection.fitObject(minX: Float(o.boundingBox.minX), maxX: Float(o.boundingBox.maxX),
                                                     minY: Float(o.boundingBox.minY), maxY: Float(o.boundingBox.maxY),
                                                     shot, floorY: floorY, tuning: t, features: features) else { return nil }
                    return ObjectObservation(obstacle: fit.obstacle, label: label.identifier, imageBox: o.boundingBox, measured: fit.measured)
                }
            } catch {
                Self.detectorLog.error("YOLO26 inference failed: \(error.localizedDescription)")
                cameraMessage = nil
                found = nil   // A failed request is not evidence that the room is empty.
            }
            let updated = mappingFloor == nil ? nil : found.map { observations in
                var next = previousTracks
                next.update(observations, shot: shot, now: timestamp, tuning: t, features: features, masks: masks)
                return next
            }
            DispatchQueue.main.async {
                guard let self else { return }
                self.detectorBusy = false
                guard self.autoDetect, self.detectorGeneration == generation else { return }
                let liveStatus = found == nil ? "Object detection unavailable" : cameraMessage ?? "Scanning for objects"
                if self.cameraStatus != liveStatus { self.cameraStatus = liveStatus }
                guard let updated, self.trackingNormal else { return }
                self.tracks = updated
                self.setDetected(self.detected.filter { $0.source != .object } + self.trackedObjects)
            }
        }
    }

    private func publishWorld(_ frame: ARFrame) {
        guard web.watched, frame.timestamp - lastWebUpdate >= 0.1 else { return }
        if !trackingNormal { lastHead = nil }   // skipped below then, so it would go stale
        lastWebUpdate = frame.timestamp
        let m = frame.camera.transform
        let p = SIMD3(m.columns.3.x, m.columns.3.y, m.columns.3.z)
        let live = autoDetect && trackingNormal   // otherwise the tracks and points are stale
        web.publish(WorldSnapshot(camera: p, forward: -SIMD3(m.columns.2.x, m.columns.2.y, m.columns.2.z), floorY: floorY,
                                  objects: live ? tracks.confirmed(tuning: tuning) : [],
                                  walls: live ? detected.filter { $0.source != .object }.map(\.obstacle) : [],
                                  range: live && frame.timestamp - lastRangeHit < 0.4 ? rangeDistance : nil,
                                  head: lastHead,
                                  ask: asker.state == .unsupported ? nil
                                      : WorldSnapshot.Ask(state: asker.stateName, q: asker.heard, a: asker.answer,
                                                          said: asker.spoken, saidCount: asker.spokenCount)))
    }

    /// One shared cube mesh drawn in wireframe, not twelve render entities per box.
    private final class Hitbox {
        let anchor = AnchorEntity(world: .zero)
        private let frame: ModelEntity
        private var obstacle: Obstacle?

        init(mesh: MeshResource, color: UIColor) {
            var material = UnlitMaterial(color: color)
            if #available(iOS 18.0, *) {
                material.triangleFillMode = .lines
            } else {
                material.blending = .transparent(opacity: 0.12)
            }
            frame = ModelEntity(mesh: mesh, materials: [material])
            anchor.addChild(frame)
        }

        func update(_ o: Obstacle) {
            guard obstacle != o else { return }
            obstacle = o
            let y = o.displayY ?? o.center.y...(o.center.y + 0.1)
            let half = SIMD3(o.halfSize.x, (y.upperBound - y.lowerBound) / 2, o.halfSize.y)
            frame.position = SIMD3(o.center.x, y.lowerBound + half.y, o.center.z)
            frame.orientation = simd_quatf(angle: o.yaw, axis: [0, 1, 0])
            frame.scale = half * 2
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

    static func nearestHitboxes(_ boxes: [Detected], to camera: SIMD2<Float>) -> [Detected] {
        let limit = 32
        guard boxes.count > limit else { return boxes }
        let ranked = boxes.indices.map { ($0, boxes[$0].obstacle.distance(to: camera)) }
            .sorted { ($0.1, $0.0) < ($1.1, $1.0) }
        return ranked.prefix(limit).map { boxes[$0.0] }
    }

    private func setDetected(_ d: [Detected]) {
        detected = d
        if detectedCount != d.count { detectedCount = d.count }
        let mesh = hitboxMesh, scene = arView.scene
        let p = arView.session.currentFrame?.camera.transform.columns.3 ?? .zero
        let visible = Self.nearestHitboxes(d, to: SIMD2(p.x, p.z))
        Self.updateHitboxes(&geometryHitboxes, with: visible.filter { $0.source != .object }.map(\.obstacle),
                            mesh: mesh, color: .systemOrange, scene: scene)
        Self.updateHitboxes(&modelHitboxes, with: visible.filter { $0.source == .object }.map(\.obstacle),
                            mesh: mesh, color: .systemYellow, scene: scene)
    }

    /// The detected obstacle a map tap at `p` points at, if any.
    func detected(near p: SIMD2<Float>) -> Detected? {
        detected.filter { $0.obstacle.distance(to: p) < tuning.removeRadius }
            .min { $0.obstacle.distance(to: p) < $1.obstacle.distance(to: p) }
    }

    /// Deletes the detected obstacle nearest `p`, and keeps it from coming back.
    /// ponytail: a removed plane is ignored by anchor ID; if ARKit later merges it into another plane, remove it again.
    func removeDetected(near p: SIMD2<Float>) {
        guard let d = detected(near: p) else { status = "That obstacle is gone already"; return }
        let r = d.obstacle.boundingRadius + tuning.detectCell
        switch d.source {
        case .plane(let id): ignoredPlanes.insert(id)
        case .points: pointMap.ignore(around: xz(d.obstacle.center), radius: r)
        case .object:
            detectorGeneration &+= 1 // don't resurrect the removed track from an in-flight snapshot
            tracks.ignore(around: xz(d.obstacle.center), radius: r)
        }
        let remaining = detected.filter { $0 != d }
        navigationObstacles = remaining.map { $0.obstacle.navigationFootprint }
        setDetected(remaining)
        status = "Removed obstacle"
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

    // MARK: Walk

    func beginAsk() {
        audio.stopSpeaking()
        tap.impactOccurred()
        asker.begin(camera: arView.session.currentFrame?.capturedImage)
    }

    func endAsk() { asker.end() }

    private func resetRange() {
        liveRange = LiveRange()
        beepTiming = ProximityBeepTiming()
        lastRangeHit = -.infinity
        rangeHaptic = .none
        proximityStatus = "Distance not established"
    }

    /// Independent of model inference and mapped hitboxes.
    private func checkClearance(_ frame: ARFrame) {
        let now = frame.timestamp
        guard now - lastRangeSample >= 0.1 else { return }
        lastRangeSample = now
        let cam = frame.camera, m = cam.transform
        let shot = CameraShot(transform: m, intrinsics: cam.intrinsics,
                              resolution: [Float(cam.imageResolution.width), Float(cam.imageResolution.height)])
        let flat = SIMD3(-m.columns.2.x, 0, -m.columns.2.z)
        var wall: Float?
        if simd_length(flat) > 0.5 {
            // Only hit the observed polygon. An estimated/infinite wall could span a doorway.
            let forward = simd_normalize(flat), right = simd_cross(forward, SIMD3(0, 1, 0))
            for offset: Float in [-0.30, 0, 0.30] {
                let origin = shot.position + right * offset
                let query = ARRaycastQuery(origin: origin, direction: forward, allowing: .existingPlaneGeometry, alignment: .vertical)
                if let hit = arView.session.raycast(query).first {
                    let p = hit.worldTransform.columns.3
                    let d = simd_dot(SIMD3(p.x, p.y, p.z) - origin, forward)
                    if d > 0 { wall = min(wall ?? .infinity, d) }
                }
            }
        }
        let cloud = frame.rawFeaturePoints
        guard let d = liveRange.clearance(points: cloud?.points ?? [], ids: cloud?.identifiers ?? [], shot: shot,
                                           floor: classifiedFloor, plane: wall, now: now) else {
            if now - lastRangeHit > 0.4 {
                rangeHaptic = .none
                if proximityStatus != "Distance not established" { proximityStatus = "Distance not established" }
            }
            return
        }
        lastRangeHit = now
        rangeDistance = d
        let distance = max(0.25, (d * 4).rounded() / 4)
        let message = String(format: "Obstacle ahead, about %g %@", Double(distance), distance == 1 ? "meter" : "meters")
        if proximityStatus != message { proximityStatus = message }
        rangeHaptic = Guidance.rangeHaptic(distance: d, speed: simd_length(motion.velocity), tuning: tuning)
        if beepTiming.shouldBeep(distance: d, now: now) { audio.playProximityBeep() }
    }

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        let normal: Bool
        if case .normal = frame.camera.trackingState { normal = true } else { normal = false }
        if normal != trackingNormal {
            if !normal {
                detectorGeneration &+= 1
                resetRange()
                audio.say("Hold still")   // obstacle warnings are off until tracking comes back
            }
            status = normal ? "" : "Tracking lost"
            trackingNormal = normal
        }
        let denied = head.accessDenied
        if denied != motionAccessDenied { motionAccessDenied = denied }
        if autoDetect {
            if normal { checkClearance(frame); detect(frame) } // Metric mapping still requires a trustworthy pose.
            runDetector(frame)         // Camera recognition needs neither a floor nor world tracking.
        }
        publishWorld(frame)

        guard normal else { return }   // never warn from a stale pose

        let m = frame.camera.transform
        var ear = SIMD3(m.columns.3.x, m.columns.3.y, m.columns.3.z)
        ear.y += tuning.earHeightAbovePhone
        let forward = listenerForward(phone: -SIMD3(m.columns.2.x, m.columns.2.y, m.columns.2.z))

        motion.add(xz(ear), now: frame.timestamp, window: tuning.velocityWindow)
        let w = Guidance.warning(listener: ear, velocity: motion.velocity, obstacles: navigationObstacles, tuning: tuning)
        lastHead = usingHeadTracking ? (ear, forward) : nil
        vibrate(max(w.haptic, rangeHaptic), now: frame.timestamp)   // one source at a time: the more urgent
        audio.update(listener: ear, forward: forward, warning: w, now: frame.timestamp)

        if frame.timestamp - lastMapUpdate > 0.1 {   // ~10 Hz is plenty for a map
            lastMapUpdate = frame.timestamp
            map = GuideMap(me: xz(ear), heading: Guidance.yaw(of: forward), obstacles: detected,
                           warned: w.obstacle, beepDistance: frame.timestamp - lastRangeHit < 0.4 ? rangeDistance : nil)
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
        case .none: lastHapticWasStop = false; return
        case .slowDown(let interval):
            guard now - lastHaptic >= interval else { return }
            tap.impactOccurred(intensity: 0.8)
            lastHapticWasStop = false
        case .stop:
            // Entering stop fires at once; don't wait out a slow tap's interval.
            guard !lastHapticWasStop || now - lastHaptic >= tuning.stopPulseInterval else { return }
            stopBuzz.notificationOccurred(.error)
            lastHapticWasStop = true
        }
        lastHaptic = now
    }

    private func setHeadTracking(_ on: Bool) {
        guard on != usingHeadTracking else { return }
        usingHeadTracking = on
        audio.say(on ? "Head tracking on" : "Using phone direction", interrupt: false)
    }
}

struct ARViewContainer: UIViewRepresentable {
    let controller: ARController
    func makeUIView(context: Context) -> ARView { controller.arView }
    func updateUIView(_ uiView: ARView, context: Context) {}
}
