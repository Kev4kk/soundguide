import XCTest
import simd
@testable import SoundGuide

final class GuidanceTests: XCTestCase {
    let t = Tuning()
    let origin = SIMD3<Float>(0, 1.5, 0)
    let north = SIMD3<Float>(0, 0, -1)   // ARKit forward is -z
    let walkNorth = SIMD2<Float>(0, -1)  // 1 m/s toward -z

    func box(_ x: Float, _ z: Float, half: SIMD2<Float> = [0.2, 0.2], yaw: Float = 0) -> Obstacle {
        Obstacle(center: [x, 0, z], halfSize: half, yaw: yaw)
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

    func warn(_ obs: [Obstacle], velocity: SIMD2<Float> = .zero) -> Warning {
        Guidance.warning(listener: origin, velocity: velocity, obstacles: obs, tuning: t)
    }

    func testWarningsUseTimeToCollision() {
        XCTAssertNil(warn([box(0, -0.6)]).position)                          // standing still next to it
        XCTAssertNil(warn([box(1.2, -1)], velocity: walkNorth).position)     // walking past it
        XCTAssertNil(warn([box(0, 0.8)], velocity: walkNorth).position)      // it's behind you

        // Heading for it, ~1.45 s away: buzz from it, slow-down taps in time with the buzz.
        let far = box(0, -2), soon = warn([box(3, -3), far], velocity: walkNorth)
        XCTAssertEqual(soon.obstacle, far)
        XCTAssertEqual(soon.haptic, .slowDown(interval: soon.buzzInterval))
        // ~0.45 s away: stop.
        let now = warn([box(0, -1)], velocity: walkNorth)
        XCTAssertTrue(now.stop); XCTAssertEqual(now.haptic, .stop)
        XCTAssertGreaterThan(now.volume, soon.volume)
    }

    func testVelocity() {
        var v = VelocityTracker()
        for i in 0...5 { v.add([0, -Float(i) * 0.1], now: Double(i) * 0.1, window: t.velocityWindow) }
        XCTAssertEqual(v.velocity.x, 0, accuracy: 1e-4); XCTAssertEqual(v.velocity.y, -1, accuracy: 1e-4)
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

    func testObstacleCorners() {
        let c = box(1, -1, half: [0.5, 0.25]).corners
        XCTAssertEqual(Set(c.map { [$0.x, $0.y] }), [[1.5, -1.25], [0.5, -1.25], [0.5, -0.75], [1.5, -0.75]])
    }

    func testMapIsHeadingUp() {
        // Facing west (yaw 90°): the point 1 m west is straight up, the point 1 m north is to your right.
        let f = MapFrame(me: [2, 3], heading: .pi / 2, anchor: CGPoint(x: 100, y: 200), pointsPerMetre: 50)
        let ahead = f.screen([1, 3]), right = f.screen([2, 2])
        XCTAssertEqual(ahead.x, 100, accuracy: 1e-3); XCTAssertEqual(ahead.y, 150, accuracy: 1e-3)
        XCTAssertEqual(right.x, 150, accuracy: 1e-3); XCTAssertEqual(right.y, 200, accuracy: 1e-3)
        // Taps map back to the same world point.
        let w = f.world(f.screen([0.5, -1]))
        XCTAssertEqual(w.x, 0.5, accuracy: 1e-4); XCTAssertEqual(w.y, -1, accuracy: 1e-4)
    }
}
