import XCTest
import simd
@testable import SoundGuide

final class GuidanceTests: XCTestCase {
    let t = Tuning()
    let origin = SIMD3<Float>(0, 1.5, 0)
    let north = SIMD3<Float>(0, 0, -1)   // ARKit forward is -z

    /// 40 cm box on the floor, like a quick tap places.
    func box(_ x: Float, _ z: Float, half: SIMD2<Float> = [0.2, 0.2], yaw: Float = 0) -> Obstacle {
        Obstacle(center: [x, 0, z], halfSize: half, yaw: yaw)
    }

    func step(_ l: SIMD3<Float>, wps: [SIMD3<Float>], obs: [Obstacle] = [], index: Int = 0,
              velocity: SIMD2<Float> = .zero, path: [SIMD2<Float>]? = nil) -> GuidanceOutput {
        Guidance.step(listener: l, forward: north, velocity: velocity, waypoints: wps, obstacles: obs,
                      index: index, path: path, tuning: t)
    }

    func testArrivalAdvancesAndFinishes() {
        let wps: [SIMD3<Float>] = [[0, 0, -0.3], [0, 0, -3]]
        let a = step(origin, wps: wps)
        XCTAssertTrue(a.arrived); XCTAssertEqual(a.index, 1); XCTAssertFalse(a.finished)
        let b = step([0, 1.5, -2.8], wps: wps, index: 1)
        XCTAssertTrue(b.finished); XCTAssertNil(b.beaconPosition)
    }

    func testArrivalUsesHorizontalDistanceOnly() {
        // Target on a shelf 1 m above you, 0.3 m away horizontally: arrived.
        XCTAssertTrue(step(origin, wps: [[0.3, 2.5, 0]]).arrived)
    }

    func testPingIntervalClamps() {
        XCTAssertEqual(Guidance.pingInterval(distance: 10, t), t.pingFarInterval)
        XCTAssertEqual(Guidance.pingInterval(distance: 0.1, t), t.pingNearInterval)
        let mid = Guidance.pingInterval(distance: 2.25, t)
        XCTAssertEqual(mid, (t.pingFarInterval + t.pingNearInterval) / 2, accuracy: 1e-6)
    }

    func testOnAxisCone() {
        func target(deg: Float) -> SIMD3<Float> { [sin(deg * .pi / 180) * 3, 0, -cos(deg * .pi / 180) * 3] }
        XCTAssertTrue(step(origin, wps: [target(deg: 0)]).onAxis)
        XCTAssertTrue(step(origin, wps: [target(deg: 14)]).onAxis)
        XCTAssertFalse(step(origin, wps: [target(deg: 16)]).onAxis)
        XCTAssertFalse(step(origin, wps: [target(deg: 180)]).onAxis)
    }

    func testElevationClamp() {
        // Floor target 3 m away, 1.5 m below: ~27°, inside the 35° cap, keeps real height.
        XCTAssertEqual(Guidance.clampedElevation(listener: origin, target: [0, 0, -3], maxDegrees: 35).y, 0, accuracy: 1e-5)
        // Floor target 0.6 m away would be ~68° down: capped at 35°.
        let near = Guidance.clampedElevation(listener: origin, target: [0, 0, -0.6], maxDegrees: 35)
        XCTAssertEqual(origin.y - near.y, 0.6 * tan(35 * .pi / 180), accuracy: 1e-5)
    }

    func testYawConvention() {
        XCTAssertEqual(Guidance.yaw(of: north), 0, accuracy: 1e-6)
        let left = Guidance.forward(yaw: .pi / 2)              // +yaw turns left
        XCTAssertEqual(left.x, -1, accuracy: 1e-6); XCTAssertEqual(left.z, 0, accuracy: 1e-6)
        let f = SIMD3<Float>(0.6, 0, -0.8)
        let back = Guidance.forward(yaw: Guidance.yaw(of: f))
        XCTAssertEqual(back.x, f.x, accuracy: 1e-6); XCTAssertEqual(back.z, f.z, accuracy: 1e-6)
    }

    // MARK: Time to collision

    let walkNorth = SIMD2<Float>(0, -1)   // 1 m/s toward -z

    func ttc(_ o: Obstacle, _ v: SIMD2<Float>, from p: SIMD2<Float> = .zero) -> Float? {
        Guidance.timeToCollision(from: p, velocity: v, obstacle: o, clearance: t.bodyClearance, horizon: 3)
    }

    func testTimeToCollision() {
        // Box edge at z = -1.8, keep-out 0.35 m: hit at 1.45 m → 1.45 s (sampled every 50 ms).
        XCTAssertEqual(ttc(box(0, -2), walkNorth)!, 1.45, accuracy: 0.051)
        XCTAssertNil(ttc(box(0, -2), -walkNorth))            // walking away
        XCTAssertNil(ttc(box(1, -2), walkNorth))             // passing by, 0.8 m from its edge
        XCTAssertEqual(ttc(box(0, -0.3), walkNorth), 0)      // already inside keep-out, closing in
        XCTAssertNil(ttc(box(0, 0.3), walkNorth))            // inside keep-out, moving away
        // A 3 m wide box you'd miss as a point is still hit off-centre.
        XCTAssertNotNil(ttc(box(1.2, -2, half: [1.5, 0.2]), walkNorth))
    }

    func testObstacleWarningsUseTimeToCollision() {
        let wps: [SIMD3<Float>] = [[3, 0, -5]]
        // Standing still right next to an obstacle: no warning.
        XCTAssertNil(step(origin, wps: wps, obs: [box(0, -0.6)]).obstaclePosition)
        // Walking past it, not toward it: no warning.
        XCTAssertNil(step(origin, wps: wps, obs: [box(1.2, -1)], velocity: walkNorth).obstaclePosition)
        // Obstacle behind you: no warning.
        XCTAssertNil(step(origin, wps: wps, obs: [box(0, 0.8)], velocity: walkNorth).obstaclePosition)

        // Heading for it, ~1.45 s away: buzz, duck the beacon, slow-down taps.
        let soon = step(origin, wps: wps, obs: [box(0, -2)], velocity: walkNorth)
        XCTAssertNotNil(soon.obstaclePosition)
        XCTAssertEqual(soon.beaconVolume, t.duckLevel)
        XCTAssertEqual(soon.haptic, .slowDown(interval: soon.buzzInterval))
        // ~0.45 s away: stop.
        let now = step(origin, wps: wps, obs: [box(0, -1)], velocity: walkNorth)
        XCTAssertTrue(now.obstacleStop); XCTAssertEqual(now.haptic, .stop)
    }

    func testWaypointSlowDownHaptics() {
        XCTAssertEqual(step(origin, wps: [[0, 0, -5]]).haptic, .none)
        let near = step(origin, wps: [[0, 0, -0.8]])
        XCTAssertEqual(near.haptic, .slowDown(interval: near.pingInterval))
    }

    // MARK: Route planning

    func testPlanIsStraightWithoutObstacles() {
        XCTAssertEqual(PathPlanner.plan(from: [0, 0], to: [0, -4], obstacles: [], clearance: t.bodyClearance), [[0, 0], [0, -4]])
    }

    func assertRouteClear(_ path: [SIMD2<Float>], of o: Obstacle, file: StaticString = #filePath, line: UInt = #line) {
        // Every point along the route keeps clear of the box (minus one grid cell of slack).
        for k in 0..<(path.count - 1) {
            for s in 0...40 {
                let p = simd_mix(path[k], path[k + 1], SIMD2(repeating: Float(s) / 40))
                XCTAssertGreaterThan(o.distance(to: p), t.bodyClearance - 0.1, file: file, line: line)
            }
        }
    }

    func testPlanGoesAroundObstacle() {
        let o = box(0, -2)
        let path = PathPlanner.plan(from: [0, 0], to: [0, -4], obstacles: [o], clearance: t.bodyClearance)
        XCTAssertEqual(path.first, [0, 0]); XCTAssertEqual(path.last, [0, -4])
        XCTAssertLessThan(path.count, 8, "smoothing should leave a few corners, not every grid cell")
        assertRouteClear(path, of: o)
    }

    func testPlanGoesAroundWideRotatedObstacle() {
        // 3 m × 0.4 m sofa, turned 30°, right across the way.
        let o = box(0, -2, half: [1.5, 0.2], yaw: .pi / 6)
        let path = PathPlanner.plan(from: [0, 0], to: [0, -4], obstacles: [o], clearance: t.bodyClearance)
        XCTAssertEqual(path.last, [0, -4])
        assertRouteClear(path, of: o)
    }

    func testPlanFromInsideKeepOutZoneStillReachesGoal() {
        let path = PathPlanner.plan(from: [0, -1.8], to: [0, -4], obstacles: [box(0, -2)], clearance: t.bodyClearance)
        XCTAssertEqual(path.last, [0, -4])
    }

    func testPursuitLooksAheadAndMeasuresOffset() {
        let path: [SIMD2<Float>] = [[0, 0], [0, -2], [2, -2]]
        let a = PathPlanner.pursuit(path: path, from: [0.5, -0.5], lookahead: 1)
        XCTAssertEqual(a.offset, 0.5, accuracy: 1e-5)
        XCTAssertEqual(a.carrot.x, 0, accuracy: 1e-5); XCTAssertEqual(a.carrot.y, -1.5, accuracy: 1e-5)
        // Lookahead wraps around the corner.
        let b = PathPlanner.pursuit(path: path, from: [0, -1.5], lookahead: 1)
        XCTAssertEqual(b.carrot.x, 0.5, accuracy: 1e-5); XCTAssertEqual(b.carrot.y, -2, accuracy: 1e-5)
        // Never past the goal.
        XCTAssertEqual(PathPlanner.pursuit(path: path, from: [1.8, -2], lookahead: 1).carrot, [2, -2])
    }

    // MARK: Navigator

    func testNavigatorSteersAroundObstacleAndReplansWhenOffPath() {
        var nav = Navigator()
        let wps: [SIMD3<Float>] = [[0, 0, -4]], obs = [box(0, -2)]
        let out = nav.tick(listener: origin, forward: north, now: 0, waypoints: wps, obstacles: obs, tuning: t)
        XCTAssertGreaterThan(abs(out.beaconPosition!.x), 0.1, "beacon should lead around the obstacle, not straight at the goal")
        XCTAssertEqual(nav.replans, 1)
        _ = nav.tick(listener: origin, forward: north, now: 0.1, waypoints: wps, obstacles: obs, tuning: t)
        XCTAssertEqual(nav.replans, 1, "on the path: no replan")
        _ = nav.tick(listener: [-3, 1.5, 0], forward: north, now: 0.2, waypoints: wps, obstacles: obs, tuning: t)
        XCTAssertEqual(nav.replans, 2, "drifted off: replan")
    }

    func testNavigatorVelocity() {
        var nav = Navigator()
        for i in 0...5 {
            _ = nav.tick(listener: [0, 1.5, -Float(i) * 0.1], forward: north, now: Double(i) * 0.1,
                         waypoints: [[0, 0, -9]], obstacles: [], tuning: t)
        }
        XCTAssertEqual(nav.velocity.x, 0, accuracy: 1e-4); XCTAssertEqual(nav.velocity.y, -1, accuracy: 1e-4)
    }

    // MARK: Obstacle boxes

    func testObstacleFromCorners() {
        // Quick tap: default 40 cm box.
        let tapBox = Obstacle.fromCorners([1, 0, -1], [1.05, 0, -1], yaw: 0, tuning: t)
        XCTAssertEqual(tapBox.halfSize, [0.2, 0.2])
        // Drag 2 m sideways and 1 m forward, facing north: 2 × 1 m box centred between the corners.
        let a = Obstacle.fromCorners([0, 0, 0], [2, 0, -1], yaw: 0, tuning: t)
        XCTAssertEqual(a.center, [1, 0, -0.5])
        XCTAssertEqual(a.halfSize.x, 1, accuracy: 1e-5); XCTAssertEqual(a.halfSize.y, 0.5, accuracy: 1e-5)
        // Same corners while facing west (yaw 90°): the box's axes swap.
        let b = Obstacle.fromCorners([0, 0, 0], [2, 0, -1], yaw: .pi / 2, tuning: t)
        XCTAssertEqual(b.halfSize.x, 0.5, accuracy: 1e-5); XCTAssertEqual(b.halfSize.y, 1, accuracy: 1e-5)
    }

    func testObstacleDistance() {
        let o = box(0, 0, half: [1, 0.5])
        XCTAssertEqual(o.distance(to: [0, 0]), 0)              // inside
        XCTAssertEqual(o.distance(to: [3, 0]), 2, accuracy: 1e-5)
        XCTAssertEqual(o.distance(to: [0, -1.5]), 1, accuracy: 1e-5)
        // Turned 90°: the long side now runs along z.
        let r = box(0, 0, half: [1, 0.5], yaw: .pi / 2)
        XCTAssertEqual(r.distance(to: [0, -1.5]), 0.5, accuracy: 1e-5)
        XCTAssertEqual(r.distance(to: [1.5, 0]), 1, accuracy: 1e-5)
    }
}
