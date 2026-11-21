import Foundation
import simd

/// Every number you might tweak during a blindfold test lives here.
struct Tuning {
    var arrivalRadius: Float = 0.5
    var pingFarDistance: Float = 4, pingFarInterval: Double = 1.0
    var pingNearDistance: Float = 0.5, pingNearInterval: Double = 0.2
    var buzzFarInterval: Double = 0.5, buzzNearInterval: Double = 0.12
    var onAxisDegrees: Float = 15
    var maxElevationDegrees: Float = 35
    var duckLevel: Float = 0.3
    // Phone is held at chest; ears sit above it. Calibration knob, varies per person.
    var earHeightAbovePhone: Float = 0.3
    var slowDownRadius: Float = 1.0        // light haptic taps when this close to the next waypoint
    var stopPulseInterval: Double = 0.15   // strong haptic repeat rate for "stop"
    var useHeadTracking = true
    // Flip to -1 if turning your head left makes sounds swing the wrong way.
    var headYawSign: Float = 1

    // Obstacles: warned by time to collision, not distance.
    var obstacleClearance: Float = 0.55    // obstacle half-size + half a body width; also the route's keep-out zone
    var warnTime: Float = 1.5              // seconds to impact: buzz + slow-down taps
    var stopTime: Float = 0.6              // seconds to impact: stop pulses
    var minWarnSpeed: Float = 0.2          // m/s; standing or shuffling never warns
    var velocityWindow: Double = 0.5       // seconds of positions used to estimate walking velocity

    // Route following.
    var lookahead: Float = 1.2             // beacon sits this far ahead along the planned route
    var offPathDistance: Float = 0.5       // replan when you drift this far off the route
}

enum Haptic: Equatable {
    case none
    case slowDown(interval: Double)   // light taps, faster as you get closer
    case stop                         // strong rapid pulses
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
    var obstacleStop: Bool
    var haptic: Haptic = .none
}

func xz(_ v: SIMD3<Float>) -> SIMD2<Float> { SIMD2(v.x, v.z) }

enum Guidance {
    /// Heading in radians, counterclockwise seen from above, 0 = ARKit's -z.
    static func yaw(of f: SIMD3<Float>) -> Float { atan2(-f.x, -f.z) }
    static func forward(yaw: Float) -> SIMD3<Float> { SIMD3(-sin(yaw), 0, -cos(yaw)) }

    static func horizontalDistance(_ a: SIMD3<Float>, _ b: SIMD3<Float>) -> Float { simd_distance(xz(a), xz(b)) }

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
        let f = xz(forward), d = xz(p) - xz(l)
        guard simd_length(f) > 1e-4, simd_length(d) > 1e-4 else { return true }
        return simd_dot(simd_normalize(f), simd_normalize(d)) >= cos(degrees * .pi / 180)
    }

    /// Seconds until walking at `v` brings you within `radius` of `o`; nil if your heading misses it
    /// or you're moving away. 0 if already inside and still approaching.
    static func timeToCollision(from p: SIMD2<Float>, velocity v: SIMD2<Float>, obstacle o: SIMD2<Float>, radius r: Float) -> Float? {
        let d = o - p, vv = simd_dot(v, v), dv = simd_dot(d, v)
        guard vv > 1e-6, dv > 0 else { return nil }
        let c = simd_dot(d, d) - r * r
        if c <= 0 { return 0 }
        let disc = dv * dv - vv * c
        guard disc >= 0 else { return nil }
        return (dv - disc.squareRoot()) / vv
    }

    /// One guidance tick. `listener` is ear position, `forward` is where the head/body faces,
    /// `velocity` is horizontal walking velocity, `path` is the planned route to `waypoints[index]`.
    static func step(listener l: SIMD3<Float>, forward: SIMD3<Float>, velocity: SIMD2<Float>,
                     waypoints: [SIMD3<Float>], obstacles: [SIMD3<Float>],
                     index: Int, path: [SIMD2<Float>]?, tuning t: Tuning) -> GuidanceOutput {
        var out = GuidanceOutput(index: index, arrived: false, finished: false, beaconPosition: nil,
                                 pingInterval: t.pingFarInterval, onAxis: false, beaconVolume: 1,
                                 obstaclePosition: nil, obstacleVolume: 0,
                                 buzzInterval: t.buzzFarInterval, obstacleStop: false)

        if index < waypoints.count, horizontalDistance(l, waypoints[index]) < t.arrivalRadius {
            out.arrived = true
            out.index = index + 1
        }
        guard out.index < waypoints.count else {
            out.finished = !waypoints.isEmpty
            return out
        }

        let target = waypoints[out.index]
        // Follow the route's lookahead point; the path is for the old target on the frame we arrive.
        let steer = (out.arrived ? nil : path).map { PathPlanner.pursuit(path: $0, from: xz(l), lookahead: t.lookahead).carrot } ?? xz(target)
        let steerPoint = SIMD3(steer.x, target.y, steer.y)
        out.beaconPosition = clampedElevation(listener: l, target: steerPoint, maxDegrees: t.maxElevationDegrees)
        out.pingInterval = pingInterval(distance: horizontalDistance(l, target), t)
        out.onAxis = isOnAxis(listener: l, forward: forward, target: steerPoint, degrees: t.onAxisDegrees)

        // ponytail: only the soonest-hit obstacle buzzes; one player per obstacle if rooms get crowded.
        if simd_length(velocity) >= t.minWarnSpeed,
           let (o, ttc) = obstacles
               .compactMap({ o in timeToCollision(from: xz(l), velocity: velocity, obstacle: xz(o), radius: t.obstacleClearance).map { (o, $0) } })
               .min(by: { $0.1 < $1.1 }),
           ttc < t.warnTime {
            let urgency = 1 - ttc / t.warnTime   // 0 at warnTime, 1 at impact
            out.obstaclePosition = clampedElevation(listener: l, target: o, maxDegrees: t.maxElevationDegrees)
            out.obstacleVolume = 0.3 + 0.7 * urgency
            out.buzzInterval = t.buzzFarInterval + Double(urgency) * (t.buzzNearInterval - t.buzzFarInterval)
            out.obstacleStop = ttc < t.stopTime
            out.beaconVolume = t.duckLevel
        }

        // Tap rate reuses the sound's rate, so feel and sound stay in step.
        var slow: [Double] = []
        if horizontalDistance(l, target) < t.slowDownRadius { slow.append(out.pingInterval) }
        if out.obstaclePosition != nil { slow.append(out.buzzInterval) }
        out.haptic = out.obstacleStop ? .stop : slow.min().map { .slowDown(interval: $0) } ?? .none
        return out
    }
}

/// Per-session guidance state: current waypoint, planned route, recent positions for velocity.
struct Navigator {
    private(set) var index = 0
    private(set) var path: [SIMD2<Float>]?
    private(set) var replans = 0
    private var samples: [(t: Double, p: SIMD2<Float>)] = []

    var velocity: SIMD2<Float> {
        guard let a = samples.first, let b = samples.last, b.t - a.t > 0.2 else { return .zero }
        return (b.p - a.p) / Float(b.t - a.t)
    }

    mutating func tick(listener l: SIMD3<Float>, forward: SIMD3<Float>, now: Double,
                       waypoints: [SIMD3<Float>], obstacles: [SIMD3<Float>], tuning t: Tuning) -> GuidanceOutput {
        let p = xz(l)
        samples.append((now, p))
        samples.removeAll { now - $0.t > t.velocityWindow }

        if index < waypoints.count {
            let offPath = path.map { PathPlanner.pursuit(path: $0, from: p, lookahead: t.lookahead).offset > t.offPathDistance } ?? true
            if offPath {
                path = PathPlanner.plan(from: p, to: xz(waypoints[index]), obstacles: obstacles.map(xz), clearance: t.obstacleClearance)
                replans += 1
            }
        }

        let out = Guidance.step(listener: l, forward: forward, velocity: velocity, waypoints: waypoints,
                                obstacles: obstacles, index: index, path: path, tuning: t)
        if out.index != index { path = nil }   // new target: plan on the next tick
        index = out.index
        return out
    }
}
