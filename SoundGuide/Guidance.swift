import Foundation
import simd

/// Every number you might tweak during a blindfold test lives here.
struct Tuning {
    var arrivalRadius: Float = 0.5
    var obstacleRadius: Float = 1.2
    var obstacleHapticRadius: Float = 0.4
    var pingFarDistance: Float = 4, pingFarInterval: Double = 1.0
    var pingNearDistance: Float = 0.5, pingNearInterval: Double = 0.2
    var buzzFarInterval: Double = 0.5, buzzNearInterval: Double = 0.12
    var onAxisDegrees: Float = 15
    var maxElevationDegrees: Float = 35
    var duckLevel: Float = 0.3
    // Phone is held at chest; ears sit above it. Calibration knob, varies per person.
    var earHeightAbovePhone: Float = 0.3
}

struct GuidanceOutput: Equatable {
    var index: Int                 // waypoint being guided to after this step
    var arrived: Bool              // reached a waypoint this step (index already advanced)
    var finished: Bool             // reached the last waypoint
    var beaconPosition: SIMD3<Float>?
    var pingInterval: Double
    var onAxis: Bool
    var beaconVolume: Float
    var obstaclePosition: SIMD3<Float>?
    var obstacleVolume: Float
    var buzzInterval: Double
    var obstacleTooClose: Bool
}

enum Guidance {
    static func horizontalDistance(_ a: SIMD3<Float>, _ b: SIMD3<Float>) -> Float {
        simd_length(SIMD2(b.x - a.x, b.z - a.z))
    }

    /// Keeps the source at its real height, but caps the vertical angle so a floor
    /// target right at your feet doesn't collapse into "somewhere underneath".
    static func clampedElevation(listener l: SIMD3<Float>, target p: SIMD3<Float>, maxDegrees: Float) -> SIMD3<Float> {
        let h = horizontalDistance(l, p)
        let maxDy = h * tan(maxDegrees * .pi / 180)
        return SIMD3(p.x, l.y + min(max(p.y - l.y, -maxDy), maxDy), p.z)
    }

    static func pingInterval(distance d: Float, _ t: Tuning) -> Double {
        let f = Double((d - t.pingNearDistance) / (t.pingFarDistance - t.pingNearDistance))
        let k = min(max(f, 0), 1)
        return t.pingNearInterval + k * (t.pingFarInterval - t.pingNearInterval)
    }

    /// Horizontal angle between where you face and where the target is.
    static func isOnAxis(listener l: SIMD3<Float>, forward: SIMD3<Float>, target p: SIMD3<Float>, degrees: Float) -> Bool {
        let f = SIMD2(forward.x, forward.z), d = SIMD2(p.x - l.x, p.z - l.z)
        guard simd_length(f) > 1e-4, simd_length(d) > 1e-4 else { return true }
        let cos = simd_dot(simd_normalize(f), simd_normalize(d))
        return cos >= Foundation.cos(degrees * .pi / 180)
    }

    /// One guidance tick. `listener` is ear position, `forward` is where the body faces.
    static func step(listener l: SIMD3<Float>, forward: SIMD3<Float>,
                     waypoints: [SIMD3<Float>], obstacles: [SIMD3<Float>],
                     index: Int, tuning t: Tuning) -> GuidanceOutput {
        var out = GuidanceOutput(index: index, arrived: false, finished: false, beaconPosition: nil,
                                 pingInterval: t.pingFarInterval, onAxis: false, beaconVolume: 1,
                                 obstaclePosition: nil, obstacleVolume: 0,
                                 buzzInterval: t.buzzFarInterval, obstacleTooClose: false)

        if index < waypoints.count, horizontalDistance(l, waypoints[index]) < t.arrivalRadius {
            out.arrived = true
            out.index = index + 1
        }
        guard out.index < waypoints.count else {
            out.finished = !waypoints.isEmpty
            return out
        }

        let target = waypoints[out.index]
        out.beaconPosition = clampedElevation(listener: l, target: target, maxDegrees: t.maxElevationDegrees)
        out.pingInterval = pingInterval(distance: horizontalDistance(l, target), t)
        out.onAxis = isOnAxis(listener: l, forward: forward, target: target, degrees: t.onAxisDegrees)

        // ponytail: only the nearest obstacle buzzes; one player per obstacle if rooms get crowded.
        if let (pos, d) = obstacles.map({ ($0, horizontalDistance(l, $0)) }).min(by: { $0.1 < $1.1 }),
           d < t.obstacleRadius {
            let p = 1 - d / t.obstacleRadius   // 0 at edge of radius, 1 touching
            out.obstaclePosition = clampedElevation(listener: l, target: pos, maxDegrees: t.maxElevationDegrees)
            out.obstacleVolume = 0.3 + 0.7 * p
            out.buzzInterval = t.buzzFarInterval + Double(p) * (t.buzzNearInterval - t.buzzFarInterval)
            out.obstacleTooClose = d < t.obstacleHapticRadius
            out.beaconVolume = t.duckLevel
        }
        return out
    }
}
