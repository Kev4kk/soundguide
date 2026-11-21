import XCTest
import simd
@testable import SoundGuide

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

    // MARK: Navigator

    func testNavigatorReplansWhenObstaclesChange() {
        var nav = Navigator()
        let wps: [SIMD3<Float>] = [[0, 0, -4]], l = SIMD3<Float>(0, 1.5, 0)
        _ = nav.tick(listener: l, forward: [0, 0, -1], now: 0, waypoints: wps, obstacles: [], tuning: t)
        _ = nav.tick(listener: l, forward: [0, 0, -1], now: 0.1, waypoints: wps, obstacles: [], tuning: t)
        XCTAssertEqual(nav.replans, 1)
        let box = Obstacle(center: [0, 0, -2], halfSize: [0.2, 0.2])
        _ = nav.tick(listener: l, forward: [0, 0, -1], now: 0.2, waypoints: wps, obstacles: [box], tuning: t)
        XCTAssertEqual(nav.replans, 2)
        XCTAssertTrue(nav.path!.allSatisfy { box.distance(to: $0) > 0.2 })
        _ = nav.tick(listener: l, forward: [0, 0, -1], now: 0.3, waypoints: wps, obstacles: [box], tuning: t)
        XCTAssertEqual(nav.replans, 2)
    }
}
