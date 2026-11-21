import XCTest
@testable import SoundGuide

final class GuidanceTests: XCTestCase {
    let t = Tuning()
    let origin = SIMD3<Float>(0, 1.5, 0)
    let north = SIMD3<Float>(0, 0, -1)   // ARKit forward is -z

    func step(_ l: SIMD3<Float>, wps: [SIMD3<Float>], obs: [SIMD3<Float>] = [], index: Int = 0) -> GuidanceOutput {
        Guidance.step(listener: l, forward: north, waypoints: wps, obstacles: obs, index: index, tuning: t)
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

    func testObstacleRadiusAndDucking() {
        let wps: [SIMD3<Float>] = [[0, 0, -5]]
        let far = step(origin, wps: wps, obs: [[1.5, 0, 0]])
        XCTAssertNil(far.obstaclePosition); XCTAssertEqual(far.beaconVolume, 1)

        let near = step(origin, wps: wps, obs: [[1.5, 0, 0], [0.8, 0, 0]])
        XCTAssertEqual(near.obstaclePosition?.x, 0.8)   // nearest wins
        XCTAssertEqual(near.beaconVolume, t.duckLevel)
        XCTAssertFalse(near.obstacleTooClose)

        XCTAssertTrue(step(origin, wps: wps, obs: [[0.3, 0, 0]]).obstacleTooClose)
    }
}
