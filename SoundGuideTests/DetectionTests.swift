import XCTest
import CoreML
import CoreVideo
import simd
@testable import SoundGuide

private extension ObjectTracks {
    mutating func update(_ boxes: [Obstacle], shot: CameraShot, now: Double, tuning t: Tuning,
                         rays: [SIMD3<Float>] = [], masks: [CGRect] = []) {
        update(boxes.map { ObjectObservation(obstacle: $0, label: "chair", imageBox: .null) },
               shot: shot, now: now, tuning: t, features: rays, masks: masks)
    }
}

final class DetectionTests: XCTestCase {
    let t = Tuning()
    let cam = SIMD3<Float>(0, 1.4, 0)

    func feed(_ m: inout PointMap, _ pts: [SIMD3<Float>], firstID: UInt64 = 0, frames: Int = 3,
              camera: SIMD3<Float>? = nil) {
        let ids = (0..<UInt64(pts.count)).map { $0 + firstID }
        for f in 0..<frames { m.add(pts, ids: ids, camera: camera ?? cam, now: Double(f) * t.carveInterval * 1.2, tuning: t) }
    }

    /// Points filling a w×d footprint, standing between the heights given.
    func blob(_ x: Float, _ z: Float, w: Float = 0.3, d: Float = 0.3, heights: [Float] = [0.3, 0.6]) -> [SIMD3<Float>] {
        stride(from: -w / 2, through: w / 2, by: 0.05).flatMap { dx in
            stride(from: -d / 2, through: d / 2, by: 0.05).flatMap { dz in heights.map { SIMD3(x + dx, $0, z + dz) } }
        }
    }

    // MARK: Planes

    func testDiagonalWallIsThinRotatedBox() {
        let dir = SIMD3<Float>(cos(0.5), 0, -sin(0.5)), up = SIMD3<Float>(0, 2.5, 0), a = SIMD3<Float>(1, 0, -2)
        let o = Detection.planeObstacle(corners: [a, a + dir * 4, a + dir * 4 + up, a + up], vertical: true, floorY: 0, tuning: t)!
        XCTAssertEqual(o.halfSize.x, 2, accuracy: 1e-3)
        XCTAssertEqual(o.halfSize.y, t.minDetectedHalfSize, accuracy: 1e-3)
        XCTAssertEqual(o.distance(to: xz(a + dir * 2)), 0)
        let side = SIMD2<Float>(-dir.z, dir.x) * 0.3
        XCTAssertEqual(o.distance(to: xz(a + dir * 2) + side), 0.25, accuracy: 1e-3)
    }

    func testHighCabinetIsNotAWall() {
        let up = SIMD3<Float>(0, 0.6, 0), a = SIMD3<Float>(0, 1.9, -2), b = SIMD3<Float>(1, 1.9, -2)
        XCTAssertNil(Detection.planeObstacle(corners: [a, b, b + up, a + up], vertical: true, floorY: 0, tuning: t))
    }

    func testHorizontalPlanesCountOnlyAtTableHeight() {
        func rect(y: Float) -> [SIMD3<Float>] {
            let r = SIMD3<Float>(cos(0.35), 0, -sin(0.35)), f = SIMD3<Float>(-sin(0.35), 0, -cos(0.35)), c = SIMD3<Float>(1, y, -2)
            return [c - r * 0.5 - f * 0.3, c + r * 0.5 - f * 0.3, c + r * 0.5 + f * 0.3, c - r * 0.5 + f * 0.3]
        }
        let table = Detection.planeObstacle(corners: rect(y: 0.75), vertical: false, floorY: 0, tuning: t)!
        XCTAssertEqual(4 * table.halfSize.x * table.halfSize.y, 0.6, accuracy: 1e-3)   // tight, not an axis-aligned bound
        XCTAssertEqual(table.center.y, 0.75, accuracy: 1e-5)
        XCTAssertEqual(table.displayY, 0...0.75)
        XCTAssertNil(Detection.planeObstacle(corners: rect(y: 0.02), vertical: false, floorY: 0, tuning: t))  // floor
        XCTAssertNil(Detection.planeObstacle(corners: rect(y: 1.5), vertical: false, floorY: 0, tuning: t))   // shelf
        XCTAssertNotNil(Detection.planeObstacle(corners: rect(y: 1.25), vertical: false, floorY: 0.5, tuning: t))
    }

    // MARK: Point map

    func testStablePointsBecomeOneBox() {
        var m = PointMap()
        feed(&m, blob(1, -2))
        let obs = m.build(floorY: 0, now: 0.1, tuning: t)
        XCTAssertEqual(obs.count, 1)
        XCTAssertEqual(obs[0].distance(to: [1, -2]), 0)
        XCTAssertEqual(obs[0].distance(to: [1, -2.5]), 0.3, accuracy: 0.1)
        XCTAssertEqual(obs[0].displayY?.lowerBound, 0)
        XCTAssertGreaterThan(obs[0].displayY?.upperBound ?? 0, 0.1)
    }

    func testOneSightingFloorClutterAndLonePointsDontCount() {
        var once = PointMap()
        feed(&once, blob(1, -2), frames: 1)
        XCTAssertTrue(once.build(floorY: 0, now: 0.1, tuning: t).isEmpty)

        var rug = PointMap()
        feed(&rug, blob(1, -2, heights: [0.03]))
        XCTAssertTrue(rug.build(floorY: 0, now: 0.1, tuning: t).isEmpty)

        var sparse = PointMap()
        feed(&sparse, [[1, 0.5, -2], [1, 0.8, -2], [2, 0.5, -1]])
        XCTAssertTrue(sparse.build(floorY: 0, now: 0.1, tuning: t).isEmpty)
    }

    func testFloorHeightIsRelative() {
        var m = PointMap()
        feed(&m, blob(1, -2, heights: [-0.97]))   // 3 cm above a floor at -1 m
        XCTAssertTrue(m.build(floorY: -1, now: 0.1, tuning: t).isEmpty)
    }

    func testFarPointsAreIgnored() {
        var m = PointMap()
        feed(&m, blob(0, -4))
        XCTAssertTrue(m.build(floorY: 0, now: 0.1, tuning: t).isEmpty)
    }

    func testTwoObjectsTwoBoxes() {
        var m = PointMap()
        feed(&m, blob(1, -2) + blob(-1, -2))
        XCTAssertEqual(m.build(floorY: 0, now: 0.1, tuning: t).count, 2)
    }

    func testDiagonalLineStaysThin() {
        var m = PointMap()
        let line = stride(from: Float(0), through: 2.8, by: 0.02).flatMap { s in
            [0.3, 0.6, 0.9].map { SIMD3<Float>(s * 0.7071, $0, -1 - s * 0.7071) }
        }
        feed(&m, line, camera: [1.5, 1.4, -0.5])
        let obs = m.build(floorY: 0, now: 0.1, tuning: t)
        XCTAssertEqual(obs.count, 1)
        XCTAssertGreaterThan(max(obs[0].halfSize.x, obs[0].halfSize.y), 1.3)
        XCTAssertLessThanOrEqual(min(obs[0].halfSize.x, obs[0].halfSize.y), 0.15)
    }

    func testLShapeLeavesInsideCornerFree() {
        var m = PointMap()
        let armX = stride(from: Float(0), through: 1.5, by: 0.02).flatMap { s in [0.3, 0.6].map { SIMD3<Float>(s, $0, -2) } }
        let armZ = stride(from: Float(0), through: 1.5, by: 0.02).flatMap { s in [0.3, 0.6].map { SIMD3<Float>(0, $0, -2 + s) } }
        feed(&m, armX + armZ, camera: [0.5, 1.4, -1])
        let obs = m.build(floorY: 0, now: 0.1, tuning: t)
        XCTAssertGreaterThan(obs.count, 1)
        XCTAssertTrue(obs.allSatisfy { $0.distance(to: [0.75, -1.25]) > 0.3 })
        for end: SIMD2<Float> in [[1.5, -2], [0, -0.5], [0, -2]] {
            XCTAssertTrue(obs.contains { $0.distance(to: end) < 0.01 })
        }
    }

    func testRefinedPointMoves() {
        var m = PointMap()
        feed(&m, blob(1, -2.6), frames: 1)
        feed(&m, blob(1, -2))   // same IDs, better estimate
        let obs = m.build(floorY: 0, now: 0.1, tuning: t)
        XCTAssertEqual(obs.count, 1)
        XCTAssertEqual(obs[0].distance(to: [1, -2]), 0)
        XCTAssertGreaterThan(obs[0].distance(to: [1, -2.6]), 0.2)
    }

    func testRemovedAreaStaysRemoved() {
        var m = PointMap()
        feed(&m, blob(1, -2) + blob(-1, -2))
        let o = m.build(floorY: 0, now: 0.1, tuning: t).first { $0.distance(to: [1, -2]) == 0 }!
        m.ignore(around: xz(o.center), radius: o.boundingRadius + t.detectCell)
        XCTAssertEqual(m.build(floorY: 0, now: 0.1, tuning: t).count, 1)
        feed(&m, blob(1, -2), firstID: 5000)   // new features on the same spot
        XCTAssertEqual(m.build(floorY: 0, now: 0.2, tuning: t).count, 1)
    }

    // MARK: Forgetting once seen empty

    /// A wall 2.8 m away, and an "object" sitting on the lines of sight to it at 1.5 m.
    let wall = stride(from: Float(-0.15), through: 0.15, by: 0.05).flatMap { x in
        stride(from: Float(0.1), through: 0.5, by: 0.1).map { SIMD3<Float>(x, $0, -2.8) }
    }
    func onSightLines(drop: Float = 0) -> [SIMD3<Float>] { wall.map { cam + ($0 - cam) * (1.5 / 2.8) - [0, drop, 0] } }

    func seeWall(_ m: inout PointMap, alsoSeeing obj: [SIMD3<Float>] = []) {
        feed(&m, wall + obj, firstID: 1000, frames: t.minPointSightings + t.forgetMisses)
    }

    func testSeeingThroughForgetsObject() {
        var m = PointMap()
        feed(&m, onSightLines())
        XCTAssertTrue(m.build(floorY: 0, now: 0.1, tuning: t).contains { $0.distance(to: [0, -1.5]) < 0.01 })
        seeWall(&m)
        let obs = m.build(floorY: 0, now: 0.5, tuning: t)
        XCTAssertFalse(obs.contains { $0.distance(to: [0, -1.5]) < 0.01 })
        XCTAssertTrue(obs.contains { $0.distance(to: [0, -2.8]) < 0.01 })   // the wall itself stays
    }

    func testStillSightedOrBelowTheSightLineIsKept() {
        var sighted = PointMap()
        feed(&sighted, onSightLines())
        _ = sighted.build(floorY: 0, now: 0.1, tuning: t)
        // Same IDs 0..<n re-reported every frame: still there (the ray grazes it).
        for f in 0..<(t.minPointSightings + t.forgetMisses) {
            let pts = wall + onSightLines()
            sighted.add(pts, ids: (0..<UInt64(wall.count)).map { $0 + 1000 } + (0..<UInt64(wall.count)).map { $0 },
                        camera: cam, now: 1 + Double(f) * t.carveInterval * 1.2, tuning: t)
        }
        XCTAssertTrue(sighted.build(floorY: 0, now: 1, tuning: t).contains { $0.distance(to: [0, -1.5]) < 0.01 })

        var below = PointMap()
        feed(&below, onSightLines(drop: 0.2))
        _ = below.build(floorY: 0, now: 0.1, tuning: t)
        seeWall(&below)
        XCTAssertTrue(below.build(floorY: 0, now: 0.5, tuning: t).contains { $0.distance(to: [0, -1.5]) < 0.01 })
    }

    // MARK: Model integration

    func testYOLO26SupportsEmptyVisionResults() throws {
        let config = MLModelConfiguration()
        config.computeUnits = .cpuOnly
        let detector = try YOLO26Detector(configuration: config)
        var buffer: CVPixelBuffer?
        XCTAssertEqual(CVPixelBufferCreate(nil, 640, 480, kCVPixelFormatType_32BGRA, nil, &buffer), kCVReturnSuccess)
        let image = try XCTUnwrap(buffer)
        CVPixelBufferLockBaseAddress(image, [])
        memset(CVPixelBufferGetBaseAddress(image), 0, CVPixelBufferGetDataSize(image))
        CVPixelBufferUnlockBaseAddress(image, [])
        XCTAssertTrue(try detector.detect(in: image, orientation: .right).isEmpty)
    }

    // MARK: Object detector geometry

    /// Portrait phone at 1.4 m facing -z, pitched down by `pitch` radians; sensor 1920×1440.
    func shot(pitch: Float = -.pi / 4, yaw: Float = 0) -> CameraShot {
        let q = simd_quatf(angle: yaw, axis: [0, 1, 0]) * simd_quatf(angle: pitch, axis: [1, 0, 0])
        // Held portrait, the sensor's x points down and its y to the right.
        let x = q.act([0, -1, 0]), y = q.act([1, 0, 0]), z = q.act([0, 0, 1])
        return CameraShot(transform: simd_float4x4(SIMD4(x, 0), SIMD4(y, 0), SIMD4(z, 0), [0, 1.4, 0, 1]),
                          intrinsics: simd_float3x3([1500, 0, 0], [0, 1500, 0], [960, 720, 1]),
                          resolution: [1920, 1440])
    }

    func hit(_ n: SIMD2<Float>, _ s: CameraShot) -> SIMD3<Float>? {
        Detection.floorHit(pixel: Detection.sensorPixel(portrait: n, s), s, floorY: 0)
    }

    func testFloorHitFromPortraitImagePoints() {
        let s = shot()
        let c = hit([0.5, 0.5], s)!
        XCTAssertEqual(c.x, 0, accuracy: 1e-4); XCTAssertEqual(c.y, 0, accuracy: 1e-4); XCTAssertEqual(c.z, -1.4, accuracy: 1e-3)
        let low = hit([0.5, 0.3], s)!                      // lower on screen = nearer floor
        XCTAssertEqual(low.x, 0, accuracy: 1e-4); XCTAssertGreaterThan(low.z, -1.4)
        XCTAssertGreaterThan(hit([0.7, 0.5], s)!.x, 0.1)   // right of screen = world right
        XCTAssertNil(hit([0.5, 0.6], shot(pitch: 0)))      // level camera: upper half never meets the floor
    }

    func testProjectInvertsFloorHit() {
        let s = shot(), n = SIMD2<Float>(0.3, 0.2)
        let px = Detection.project(hit(n, s)!, s)!
        let back = Detection.portraitPoint(sensor: px, s)
        XCTAssertEqual(back.x, n.x, accuracy: 1e-4); XCTAssertEqual(back.y, n.y, accuracy: 1e-4)
        XCTAssertNil(Detection.project([0, 0, 2], s))       // behind the camera
    }

    func testObjectFootprintStandsOnBoxBottom() {
        let s = shot()
        let o = Detection.objectFootprint(minX: 0.4, maxX: 0.6, minY: 0.5, maxY: 0.85, s, floorY: 0, tuning: t)!
        XCTAssertEqual(o.center.x, 0, accuracy: 1e-4)
        XCTAssertEqual(o.distance(to: [0, -1.4]), 0, accuracy: 1e-3)    // bottom-centre is its near edge
        XCTAssertLessThan(o.center.z, -1.4)                              // and it extends away from the camera
        XCTAssertGreaterThan(o.halfSize.x, t.minObjectSize / 2 - 1e-5)
        XCTAssertNil(Detection.objectFootprint(minX: 0.4, maxX: 0.6, minY: 0.01, maxY: 0.5, s, floorY: 0, tuning: t))  // bottom cut off
        XCTAssertNil(Detection.objectFootprint(minX: 0.4, maxX: 0.6, minY: 0.98, maxY: 1, shot(pitch: -0.15), floorY: 0, tuning: t))  // near the horizon: too far or never meets the floor
    }

    func testHitboxHeightUsesTopRayWithoutChangingFootprint() throws {
        let s = shot()
        let top = try XCTUnwrap(Detection.project([0, 0.8, -1.4], s))
        let maxY = Detection.portraitPoint(sensor: top, s).y
        let tall = try XCTUnwrap(Detection.objectFootprint(minX: 0.4, maxX: 0.6, minY: 0.5, maxY: maxY, s, floorY: 0, tuning: t))
        let short = try XCTUnwrap(Detection.objectFootprint(minX: 0.4, maxX: 0.6, minY: 0.5, maxY: 0.6, s, floorY: 0, tuning: t))
        let y = try XCTUnwrap(tall.displayY)
        XCTAssertEqual(y.lowerBound, 0)
        XCTAssertEqual(y.upperBound, 0.8, accuracy: 1e-4)
        XCTAssertLessThan(try XCTUnwrap(short.displayY).upperBound, y.upperBound)
        XCTAssertEqual(tall.navigationFootprint, short.navigationFootprint)
    }

    func testElevatedObjectUsesVisibleSurfaceInsteadOfFloor() throws {
        let s = shot(pitch: 0)
        func image(_ p: SIMD3<Float>) throws -> SIMD2<Float> {
            Detection.portraitPoint(sensor: try XCTUnwrap(Detection.project(p, s)), s)
        }
        let lo = try image([-0.3, 0.95, -2]), hi = try image([0.3, 1.65, -2])
        let features: [SIMD3<Float>] = [[-0.1, 1.2, -2], [0.1, 1.3, -2], [0, 1.5, -2], [0, 1.4, -2], [0, 1.4, -3.5]]
        let o = try XCTUnwrap(Detection.objectFootprint(minX: lo.x, maxX: hi.x, minY: lo.y, maxY: hi.y, s, floorY: 0, tuning: t, features: features))
        XCTAssertEqual(o.halfSize.x * 2, 0.6, accuracy: 0.01)
        XCTAssertLessThan(o.halfSize.y, o.halfSize.x)
        XCTAssertEqual(try XCTUnwrap(o.displayY).lowerBound, 0.95, accuracy: 0.01)
        XCTAssertEqual(try XCTUnwrap(o.displayY).upperBound, 1.65, accuracy: 0.01)
    }

    func testCrowdedSceneBoundsOnlyTheOverlay() {
        let all = (0..<2_000).map { i in
            Detected(obstacle: Obstacle(center: [Float(i), 0, 0], halfSize: [0.1, 0.1]), source: .points)
        }
        let visible = ARController.nearestHitboxes(all, to: [1999, 0])
        XCTAssertEqual(visible.count, 32)
        XCTAssertEqual(visible.first, all.last)
        XCTAssertEqual(visible.last, all[1968])
    }

    func testLiveClearanceWithoutFloorIgnoresIsolatedAndSidePoints() throws {
        let s = shot(pitch: 0), points: [SIMD3<Float>] = [[0, 1.4, -0.3], [-0.1, 1.4, -1], [0, 1.3, -1.02], [0.1, 1.5, -1.05], [0.6, 1.4, -0.4]]
        let ids = points.indices.map(UInt64.init)
        var range = LiveRange()
        XCTAssertNil(range.clearance(points: points, ids: ids, shot: s, floor: nil, plane: nil, now: 0))
        XCTAssertEqual(try XCTUnwrap(range.clearance(points: points, ids: ids, shot: s, floor: nil, plane: nil, now: 0.1)), 1.02, accuracy: 0.001)
        XCTAssertEqual(range.clearance(points: points, ids: ids, shot: s, floor: nil, plane: 0.7, now: 0.2), 0.7)
        XCTAssertNil(range.clearance(points: points, ids: ids, shot: s, floor: nil, plane: nil, now: 1))
        XCTAssertNil(range.clearance(points: [], ids: [], shot: s, floor: nil, plane: nil, now: 1.1))
    }

    func testLiveClearanceRejectsFloorAndBehindCameraEvidence() {
        let points: [SIMD3<Float>] = [[0, 0.05, -1], [-0.1, 0.02, -1], [0.1, 0.04, -1], [0, 1.2, 1], [-0.1, 1.2, 1], [0.1, 1.2, 1]]
        var range = LiveRange()
        for now in [0.0, 0.1, 0.2] {
            XCTAssertNil(range.clearance(points: points, ids: points.indices.map(UInt64.init), shot: shot(), floor: 0, plane: nil, now: now))
        }
    }

    func testCameraCuesNeedNoFloorAndIncludePeople() {
        let person = CameraObject(label: "person", bounds: CGRect(x: 0.05, y: 0.1, width: 0.4, height: 0.7))
        let chair = CameraObject(label: "chair", bounds: CGRect(x: 0.7, y: 0.2, width: 0.1, height: 0.1))
        XCTAssertEqual(CameraObject.primary([person, chair])?.displayText, "Person, left")
        XCTAssertEqual(CameraObject.primary([chair])?.displayText, "Chair, right")
        XCTAssertNil(CameraObject.primary([])) // No all-clear conclusion from a blank image.
    }

    func testProximityBeepCadence() {
        var timing = ProximityBeepTiming()
        XCTAssertFalse(timing.shouldBeep(distance: .nan, now: 0))
        XCTAssertFalse(timing.shouldBeep(distance: 2.1, now: 0))
        XCTAssertTrue(timing.shouldBeep(distance: 2, now: 0))
        XCTAssertFalse(timing.shouldBeep(distance: 2, now: 0.8))
        XCTAssertTrue(timing.shouldBeep(distance: 2, now: 0.9))
        XCTAssertFalse(timing.shouldBeep(distance: 1, now: 1.3))
        XCTAssertTrue(timing.shouldBeep(distance: 1, now: 1.4))
        XCTAssertFalse(timing.shouldBeep(distance: 0.5, now: 1.5))
        XCTAssertTrue(timing.shouldBeep(distance: 0.5, now: 1.7))
    }

    // MARK: Object tracks

    func testTracksConfirmAfterSightingsAndMerge() {
        var tr = ObjectTracks()
        let s = shot(), box = Obstacle(center: [0, 0, -1.6], halfSize: [0.2, 0.2])
        var moved = box; moved.center.x += 0.1
        tr.update([box], shot: s, now: 0, tuning: t)
        tr.update([moved], shot: s, now: 0.2, tuning: t)
        XCTAssertTrue(tr.obstacles(tuning: t).isEmpty)
        tr.update([box, Obstacle(center: [2, 0, -1.6], halfSize: [0.2, 0.2])], shot: s, now: 0.4, tuning: t)
        let obs = tr.obstacles(tuning: t)
        XCTAssertEqual(obs.count, 1)
        XCTAssertEqual(obs[0].center.x, 0.1 / 3, accuracy: 1e-4)
    }

    func testTrackForgottenOnlyWhenItsSpotIsInViewAndEmpty() {
        var tr = ObjectTracks()
        let box = Obstacle(center: [0, 0, -1.4], halfSize: [0.2, 0.2])
        for i in 0..<3 { tr.update([box], shot: shot(), now: Double(i) * 0.2, tuning: t) }
        let rays: [SIMD3<Float>] = [-0.1, 0, 0.1].map { x in shot().position + (SIMD3<Float>(x, 0.25, -1.4) - shot().position) * 2 }
        for i in 0..<t.objectForgetMisses {
            tr.update([], shot: shot(yaw: .pi), now: 1 + Double(i) * 0.2, tuning: t, rays: rays)
        }
        for i in 0..<t.objectForgetMisses {
            tr.update([], shot: shot(), now: 3 + Double(i) * 0.2, tuning: t, rays: rays, masks: [CGRect(x: 0, y: 0, width: 1, height: 1)])
        }
        for i in 0..<t.objectForgetMisses {
            tr.update([], shot: shot(), now: 5 + Double(i) * 0.2, tuning: t)
        }
        XCTAssertEqual(tr.obstacles(tuning: t).count, 1)
        // At 50 Hz only the first miss in each 200 ms interval counts.
        for i in 0..<(10 * (t.objectForgetMisses - 1)) { tr.update([], shot: shot(), now: 7 + Double(i) * 0.02, tuning: t, rays: rays) }
        XCTAssertEqual(tr.obstacles(tuning: t).count, 1)
        tr.update([], shot: shot(), now: 7 + Double(t.objectForgetMisses - 1) * 0.2, tuning: t, rays: rays)
        XCTAssertTrue(tr.obstacles(tuning: t).isEmpty)
    }

    func testFastFramesDoNotConfirmTracksPrematurely() {
        var tr = ObjectTracks()
        let s = shot(), box = Obstacle(center: [0, 0, -1.4], halfSize: [0.2, 0.2])
        for i in 0..<20 { tr.update([box], shot: s, now: Double(i) * 0.02, tuning: t) }
        XCTAssertTrue(tr.obstacles(tuning: t).isEmpty)
        tr.update([box], shot: s, now: 0.4, tuning: t)
        XCTAssertEqual(tr.obstacles(tuning: t).count, 1)
    }

    func testDuplicateBoxesDoNotCountAsMultipleSightings() {
        var tr = ObjectTracks()
        let s = shot(), box = Obstacle(center: [0, 0, -1.4], halfSize: [0.2, 0.2])
        tr.update([box, box], shot: s, now: 0, tuning: t)
        tr.update([box, box], shot: s, now: 0.4, tuning: t)
        XCTAssertTrue(tr.obstacles(tuning: t).isEmpty)
        tr.update([box], shot: s, now: 0.42, tuning: t)
        XCTAssertEqual(tr.obstacles(tuning: t).count, 1)
    }

    func testMovingChairKeepsIdentityBesideStationaryChair() throws {
        var tr = ObjectTracks()
        let s = shot()
        func chair(_ x: Float) -> Obstacle { Obstacle(center: [x, 0, -2], halfSize: [0.15, 0.15]) }
        for i in 0..<3 { tr.update([chair(0), chair(0.45)], shot: s, now: Double(i) * 0.2, tuning: t) }
        let ids = tr.confirmed(tuning: t).map(\.id)
        XCTAssertEqual(ids.count, 2)
        for i in 1...5 {
            tr.update([chair(0.45), chair(-0.2 * Float(i))], shot: s, now: 0.4 + Double(i) * 0.2, tuning: t)
        }
        let result = tr.confirmed(tuning: t)
        XCTAssertEqual(result.map(\.id), ids)
        XCTAssertEqual(try XCTUnwrap(result.first).obstacle.center.x, -1, accuracy: 0.05)
        XCTAssertEqual(try XCTUnwrap(result.last).obstacle.center.x, 0.45, accuracy: 0.01)
    }

    func testRemovedTrackStaysRemoved() {
        var tr = ObjectTracks()
        let box = Obstacle(center: [0, 0, -1.4], halfSize: [0.2, 0.2])
        for i in 0..<3 { tr.update([box], shot: shot(), now: Double(i) * 0.2, tuning: t) }
        tr.ignore(around: [0, -1.4], radius: 0.5)
        for i in 3..<6 { tr.update([box], shot: shot(), now: Double(i) * 0.2, tuning: t) }
        XCTAssertTrue(tr.obstacles(tuning: t).isEmpty)
    }
}
