import XCTest
import simd
@testable import SoundGuide

final class GuidanceTests: XCTestCase {
    let t = Tuning()
    let origin = SIMD3<Float>(0, 1.5, 0)
    let north = SIMD3<Float>(0, 0, -1)   // ARKit forward is -z

    func step(_ l: SIMD3<Float>, wps: [SIMD3<Float>], obs: [SIMD3<Float>] = [], index: Int = 0,
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

    func testTimeToCollision() {
        let o = SIMD2<Float>(0, -2)
        XCTAssertEqual(Guidance.timeToCollision(from: .zero, velocity: walkNorth, obstacle: o, radius: 0.5)!, 1.5, accuracy: 1e-5)
        XCTAssertNil(Guidance.timeToCollision(from: .zero, velocity: -walkNorth, obstacle: o, radius: 0.5))   // walking away
        XCTAssertNil(Guidance.timeToCollision(from: .zero, velocity: walkNorth, obstacle: [1, -2], radius: 0.5)) // passing by
        XCTAssertEqual(Guidance.timeToCollision(from: .zero, velocity: walkNorth, obstacle: [0, -0.3], radius: 0.5), 0)
    }

    func testObstacleWarningsUseTimeToCollision() {
        let wps: [SIMD3<Float>] = [[3, 0, -5]]
        // Standing still right next to an obstacle: no warning.
        XCTAssertNil(step(origin, wps: wps, obs: [[0, 0, -0.6]]).obstaclePosition)
        // Walking past it, not toward it: no warning.
        XCTAssertNil(step(origin, wps: wps, obs: [[1.2, 0, -1]], velocity: walkNorth).obstaclePosition)
        // Obstacle behind you: no warning.
        XCTAssertNil(step(origin, wps: wps, obs: [[0, 0, 0.8]], velocity: walkNorth).obstaclePosition)

        // Heading for it, ~1.45 s away: buzz, duck the beacon, slow-down taps.
        let soon = step(origin, wps: wps, obs: [[0, 0, -2]], velocity: walkNorth)
        XCTAssertNotNil(soon.obstaclePosition)
        XCTAssertEqual(soon.beaconVolume, t.duckLevel)
        XCTAssertEqual(soon.haptic, .slowDown(interval: soon.buzzInterval))
        // ~0.45 s away: stop.
        let now = step(origin, wps: wps, obs: [[0, 0, -1]], velocity: walkNorth)
        XCTAssertTrue(now.obstacleStop); XCTAssertEqual(now.haptic, .stop)
    }

    func testWaypointSlowDownHaptics() {
        XCTAssertEqual(step(origin, wps: [[0, 0, -5]]).haptic, .none)
        let near = step(origin, wps: [[0, 0, -0.8]])
        XCTAssertEqual(near.haptic, .slowDown(interval: near.pingInterval))
    }

    // MARK: Route planning

    func testPlanIsStraightWithoutObstacles() {
        XCTAssertEqual(PathPlanner.plan(from: [0, 0], to: [0, -4], obstacles: [], clearance: 0.55), [[0, 0], [0, -4]])
    }

    func testPlanGoesAroundObstacle() {
        let o = SIMD2<Float>(0, -2)
        let path = PathPlanner.plan(from: [0, 0], to: [0, -4], obstacles: [o], clearance: 0.55)
        XCTAssertEqual(path.first, [0, 0]); XCTAssertEqual(path.last, [0, -4])
        XCTAssertLessThan(path.count, 8, "smoothing should leave a few corners, not every grid cell")
        // Every point along the route keeps clear of the obstacle (minus one grid cell of slack).
        for k in 0..<(path.count - 1) {
            for s in 0...20 {
                let p = simd_mix(path[k], path[k + 1], SIMD2(repeating: Float(s) / 20))
                XCTAssertGreaterThan(simd_distance(p, o), 0.55 - 0.1)
            }
        }
    }

    func testPlanFromInsideKeepOutZoneStillReachesGoal() {
        let path = PathPlanner.plan(from: [0, -1.8], to: [0, -4], obstacles: [[0, -2]], clearance: 0.55)
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
        let wps: [SIMD3<Float>] = [[0, 0, -4]], obs: [SIMD3<Float>] = [[0, 0, -2]]
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
}
