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
    var bodyClearance: Float = 0.35        // half a body width plus margin; keep-out distance from an obstacle's edge
    var tapObstacleHalfSize: Float = 0.2   // a quick tap places a 40 cm box
    var minDragSize: Float = 0.15          // press-to-release shorter than this counts as a tap
    var warnTime: Float = 1.5              // seconds to impact: buzz + slow-down taps
    var stopTime: Float = 0.6              // seconds to impact: stop pulses
    var minWarnSpeed: Float = 0.2          // m/s; standing or shuffling never warns
    var velocityWindow: Double = 0.5       // seconds of positions used to estimate walking velocity

    // Spoken directions when the walker stands still.
    var stoppedSpeed: Float = 0.15         // m/s; below this counts as standing still
    var stoppedDelay: Double = 1.5         // seconds still before speaking
    var calloutRepeat: Double = 8          // repeat while still standing
    var calloutObstacleRange: Float = 2.5  // mention obstacles whose edge is this close
    var calloutMaxObstacles = 2

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
    var callout: String?           // speak this (set by Navigator when you've stopped)
}

func xz(_ v: SIMD3<Float>) -> SIMD2<Float> { SIMD2(v.x, v.z) }

/// Box on the floor plane, rotated by `yaw` (Guidance.yaw convention). `center.y` is its height, used for sound.
struct Obstacle: Equatable {
    var center: SIMD3<Float>
    var halfSize: SIMD2<Float>   // along its own right / forward axes
    var yaw: Float = 0

    private var right: SIMD2<Float> { SIMD2(cos(yaw), -sin(yaw)) }
    private var fwd: SIMD2<Float> { SIMD2(-sin(yaw), -cos(yaw)) }

    /// Box spanned by two opposite corners, lined up with the direction the phone faced (`yaw`).
    /// ponytail: two corners can't say how the box is turned, so it follows the phone; a third tap if that's not enough.
    static func fromCorners(_ a: SIMD3<Float>, _ b: SIMD3<Float>, yaw: Float, tuning t: Tuning) -> Obstacle {
        var o = Obstacle(center: (a + b) / 2, halfSize: SIMD2(repeating: t.tapObstacleHalfSize), yaw: yaw)
        let d = xz(b) - xz(a)
        if simd_length(d) >= t.minDragSize {
            o.halfSize = simd_max(SIMD2(abs(simd_dot(d, o.right)), abs(simd_dot(d, o.fwd))) / 2, SIMD2(repeating: 0.05))
        }
        return o
    }

    func closestPoint(to p: SIMD2<Float>) -> SIMD2<Float> {
        let d = p - xz(center)
        let u = min(max(simd_dot(d, right), -halfSize.x), halfSize.x)
        let w = min(max(simd_dot(d, fwd), -halfSize.y), halfSize.y)
        return xz(center) + right * u + fwd * w
    }

    /// 0 inside the box, otherwise distance to its edge.
    func distance(to p: SIMD2<Float>) -> Float { simd_distance(p, closestPoint(to: p)) }

    var boundingRadius: Float { simd_length(halfSize) }
}

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

    /// Seconds until walking at `v` brings you within `clearance` of the box's edge, up to `horizon`.
    /// nil if your heading misses it, you're moving away, or it's further off than `horizon`.
    /// ponytail: sampled every 50 ms; a closed-form ray vs rounded box if that's ever too coarse.
    static func timeToCollision(from p: SIMD2<Float>, velocity v: SIMD2<Float>, obstacle o: Obstacle,
                                clearance: Float, horizon: Float) -> Float? {
        guard simd_length(v) > 1e-3 else { return nil }
        let dt: Float = 0.05
        if o.distance(to: p) < clearance {
            return o.distance(to: p + v * dt) < o.distance(to: p) ? 0 : nil   // inside: only if still closing in
        }
        for t in stride(from: dt, through: horizon, by: dt) where o.distance(to: p + v * t) < clearance { return t }
        return nil
    }

    /// Signed horizontal angle in degrees from `forward` to `dir`, positive = to your left.
    static func relativeAngle(forward: SIMD2<Float>, to dir: SIMD2<Float>) -> Float {
        let a = (yaw(of: SIMD3(dir.x, 0, dir.y)) - yaw(of: SIMD3(forward.x, 0, forward.y))) * 180 / .pi
        return a > 180 ? a - 360 : (a < -180 ? a + 360 : a)
    }

    /// 12 = ahead, 3 = right, 9 = left.
    static func clock(_ degrees: Float) -> Int {
        let h = (12 - Int((degrees / 30).rounded())) % 12
        return h <= 0 ? h + 12 : h
    }

    static func turnPhrase(_ degrees: Float) -> String {
        let side = degrees > 0 ? "left" : "right"
        switch abs(degrees) {
        case ..<15: return "Straight ahead"
        case ..<45: return "Slightly \(side)"
        case ..<135: return "Turn \(side)"
        default: return "Turn around"
        }
    }

    static func metres(_ d: Float) -> String { d < 1 ? String(format: "%.0f centimetres", (d * 10).rounded() * 10) : String(format: "%.1f metres", d) }

    /// e.g. "Waypoint 2, 3.4 metres. Turn right, 3 o'clock. Obstacle at 1 o'clock, 80 centimetres."
    static func callout(listener l: SIMD3<Float>, forward: SIMD3<Float>, steer: SIMD2<Float>, waypointNumber: Int,
                        waypointDistance: Float, obstacles: [Obstacle], tuning t: Tuning) -> String {
        let f = xz(forward), p = xz(l)
        let a = relativeAngle(forward: f, to: steer - p)
        var parts = ["Waypoint \(waypointNumber), \(metres(waypointDistance))",
                     abs(a) < 15 ? "Straight ahead" : "\(turnPhrase(a)), \(clock(a)) o'clock"]
        let near = obstacles.map { ($0.closestPoint(to: p), $0.distance(to: p)) }
            .filter { $0.1 < t.calloutObstacleRange }
            .sorted { $0.1 < $1.1 }
            .prefix(t.calloutMaxObstacles)
        for (point, d) in near {
            let where_ = d < 0.05 ? "right next to you" : "at \(clock(relativeAngle(forward: f, to: point - p))) o'clock, \(metres(d))"
            parts.append("Obstacle \(where_)")
        }
        return parts.joined(separator: ". ") + "."
    }

    /// One guidance tick. `listener` is ear position, `forward` is where the head/body faces,
    /// `velocity` is horizontal walking velocity, `path` is the planned route to `waypoints[index]`.
    static func step(listener l: SIMD3<Float>, forward: SIMD3<Float>, velocity: SIMD2<Float>,
                     waypoints: [SIMD3<Float>], obstacles: [Obstacle],
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
               .compactMap({ o in timeToCollision(from: xz(l), velocity: velocity, obstacle: o,
                                                  clearance: t.bodyClearance, horizon: t.warnTime).map { (o, $0) } })
               .min(by: { $0.1 < $1.1 }),
           ttc < t.warnTime {
            let urgency = 1 - ttc / t.warnTime   // 0 at warnTime, 1 at impact
            // Buzz from the nearest point of the box, so a sofa sounds like where you'd hit it.
            let near = o.closestPoint(to: xz(l))
            out.obstaclePosition = clampedElevation(listener: l, target: SIMD3(near.x, o.center.y, near.y), maxDegrees: t.maxElevationDegrees)
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
    private var stillSince: Double?
    private var lastCallout = -Double.infinity

    var velocity: SIMD2<Float> {
        guard let a = samples.first, let b = samples.last, b.t - a.t > 0.2 else { return .zero }
        return (b.p - a.p) / Float(b.t - a.t)
    }

    mutating func tick(listener l: SIMD3<Float>, forward: SIMD3<Float>, now: Double,
                       waypoints: [SIMD3<Float>], obstacles: [Obstacle], tuning t: Tuning) -> GuidanceOutput {
        let p = xz(l)
        samples.append((now, p))
        samples.removeAll { now - $0.t > t.velocityWindow }

        if index < waypoints.count {
            let offPath = path.map { PathPlanner.pursuit(path: $0, from: p, lookahead: t.lookahead).offset > t.offPathDistance } ?? true
            if offPath {
                path = PathPlanner.plan(from: p, to: xz(waypoints[index]), obstacles: obstacles, clearance: t.bodyClearance)
                replans += 1
            }
        }

        var out = Guidance.step(listener: l, forward: forward, velocity: velocity, waypoints: waypoints,
                                obstacles: obstacles, index: index, path: path, tuning: t)

        // Standing still for a moment: say which way to go and what's nearby, then repeat now and then.
        if simd_length(velocity) >= t.stoppedSpeed || out.arrived {
            stillSince = nil
            lastCallout = -.infinity
        } else {
            let since = stillSince ?? now
            stillSince = since
            if now - since >= t.stoppedDelay, now - lastCallout >= t.calloutRepeat, let b = out.beaconPosition {
                out.callout = Guidance.callout(listener: l, forward: forward, steer: xz(b), waypointNumber: out.index + 1,
                                               waypointDistance: Guidance.horizontalDistance(l, waypoints[out.index]),
                                               obstacles: obstacles, tuning: t)
                lastCallout = now
            }
        }

        if out.index != index { path = nil }   // new target: plan on the next tick
        index = out.index
        return out
    }
}
