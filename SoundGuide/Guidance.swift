import Foundation
import simd

/// Every number you might tweak during a blindfold test lives here.
struct Tuning {
    var buzzFarInterval: Double = 0.5, buzzNearInterval: Double = 0.12
    var maxElevationDegrees: Float = 35
    // Phone is held at chest; ears sit above it. Calibration knob, varies per person.
    var earHeightAbovePhone: Float = 0.3
    var stopPulseInterval: Double = 0.3    // repeat rate of the triple "stop" buzz
    // Live range (phone to what's ahead), only while walking.
    var rangeBumpDistance: Float = 1.2     // metres: "about to bump" taps
    var rangeStopDistance: Float = 0.65    // metres: "stop" buzz
    var useHeadTracking = true
    // Flip to -1 if turning your head left makes sounds swing the wrong way.
    var headYawSign: Float = 1

    // Obstacles: warned by time to collision, not distance.
    var bodyClearance: Float = 0.35        // half a body width plus margin; keep-out distance from an obstacle's edge
    var warnTime: Float = 1.5              // seconds to impact: buzz + slow-down taps
    var stopTime: Float = 0.6              // seconds to impact: stop pulses
    var minWarnSpeed: Float = 0.2          // m/s; standing or shuffling never warns
    var velocityWindow: Double = 0.5       // seconds of positions used to estimate walking velocity

    // Automatic obstacles from ARKit planes and feature points (no LiDAR on this phone).
    var detectCell: Float = 0.1            // floor cell size of the feature-point map
    var detectMinHeight: Float = 0.1       // below this is floor clutter: rugs, cables
    var detectMaxHeight: Float = 1.8       // above head height doesn't block walking
    var detectRange: Float = 3             // feature depth gets too noisy beyond this without LiDAR
    var minPointSightings = 3              // frames a feature must be tracked before it counts
    var minPointsPerCell = 3
    var minBoxFill: Float = 0.4            // split a cluster's box while it's less full than this
    var tableMinHeight: Float = 0.3, tableMaxHeight: Float = 1.2
    var minDetectedHalfSize: Float = 0.05  // walls are at least 10 cm thick
    var carveRadius: Float = 0.05          // a line of sight this close to a stored point sees through it...
    var carveMargin: Float = 0.3           // ...if the feature it reached is at least this far beyond
    var forgetMisses = 5                   // seen-through checks (no fresh sighting in between) before a point is forgotten
    var carveInterval: Double = 0.1        // seconds between seen-through checks
    var maxCarveRays = 150                 // features per check; keeps it cheap on the main thread
    var unstablePointLifetime: Double = 2  // drop points that never became stable after this long
    var detectInterval: Double = 0.5       // seconds between obstacle map rebuilds
    var removeRadius: Float = 0.3          // a map tap picks a detected obstacle this close

    // Object detector (YOLO26n via Vision): a box on the floor where each detected object stands.
    var objectEvidenceInterval: Double = 0.2   // preserve confirmation and forgetting at higher frame rates
    var hotObjectInterval: Double = 0.2        // inference backoff when iOS reports thermal pressure
    var objectConfidence: Float = 0.5
    var objectRange: Float = 4             // further boxes land too far off on the floor
    var minBoxBottom: Float = 0.03         // a box touching the image bottom hides where it meets the floor
    var minObjectSize: Float = 0.2, maxObjectSize: Float = 2
    var maxObjectDisplayHeight: Float = 3  // cap estimates from clipped or noisy image boxes
    var objectMergeDistance: Float = 0.6   // a box this close to a known object is the same object
    var minObjectSightings = 3
    var objectForgetMisses = 8             // detector runs with its spot in view and nothing found there
}

enum Haptic: Comparable {
    case none
    case slowDown(interval: Double)   // "about to bump": medium taps, faster as you get closer
    case stop                         // "stop now": repeated triple error buzz

    /// none < slowDown (slower before faster) < stop, so `max` picks the more urgent.
    static func < (a: Haptic, b: Haptic) -> Bool {
        func rank(_ h: Haptic) -> Double {
            switch h { case .none: return 0; case .slowDown(let i): return 1 + 1 / (1 + i); case .stop: return 3 }
        }
        return rank(a) < rank(b)
    }
}

/// What the obstacle buzz and haptics should do this frame.
struct Warning: Equatable {
    var obstacle: Obstacle?        // the one you'd walk into soonest
    var position: SIMD3<Float>?    // where its buzz plays from
    var volume: Float = 0
    var buzzInterval: Double = 0
    var stop = false
    var haptic: Haptic = .none
}

func xz(_ v: SIMD3<Float>) -> SIMD2<Float> { SIMD2(v.x, v.z) }

/// Box on the floor plane, rotated by `yaw` (Guidance.yaw convention). `center.y` is its height, used for sound.
struct Obstacle: Equatable {
    var center: SIMD3<Float>
    var halfSize: SIMD2<Float>   // along its own right / forward axes
    var yaw: Float = 0
    var displayY: ClosedRange<Float>? = nil  // World-space extent for the AR overlay only.

    var navigationFootprint: Obstacle {
        Obstacle(center: center, halfSize: halfSize, yaw: yaw)
    }

    private var right: SIMD2<Float> { SIMD2(cos(yaw), -sin(yaw)) }
    private var fwd: SIMD2<Float> { SIMD2(-sin(yaw), -cos(yaw)) }

    func closestPoint(to p: SIMD2<Float>) -> SIMD2<Float> {
        let d = p - xz(center)
        let u = min(max(simd_dot(d, right), -halfSize.x), halfSize.x)
        let w = min(max(simd_dot(d, fwd), -halfSize.y), halfSize.y)
        return xz(center) + right * u + fwd * w
    }

    /// 0 inside the box, otherwise distance to its edge.
    func distance(to p: SIMD2<Float>) -> Float { simd_distance(p, closestPoint(to: p)) }

    var boundingRadius: Float { simd_length(halfSize) }

    var corners: [SIMD2<Float>] {
        let c = xz(center), r = right * halfSize.x, f = fwd * halfSize.y
        return [c + r + f, c - r + f, c - r - f, c + r - f]
    }
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

    /// Haptic from the live range. Standing still never vibrates; the beeps still say how close it is.
    static func rangeHaptic(distance d: Float, speed: Float, tuning t: Tuning) -> Haptic {
        guard speed >= t.minWarnSpeed else { return .none }
        return d <= t.rangeStopDistance ? .stop : d <= t.rangeBumpDistance ? .slowDown(interval: 0.45) : .none
    }

    /// Buzz from the obstacle you'd reach soonest at your current walking velocity.
    /// ponytail: only the soonest-hit obstacle buzzes; one player per obstacle if rooms get crowded.
    static func warning(listener l: SIMD3<Float>, velocity: SIMD2<Float>, obstacles: [Obstacle], tuning t: Tuning) -> Warning {
        var out = Warning(buzzInterval: t.buzzFarInterval)
        guard simd_length(velocity) >= t.minWarnSpeed,
              let (o, ttc) = obstacles
                  .compactMap({ o in timeToCollision(from: xz(l), velocity: velocity, obstacle: o,
                                                     clearance: t.bodyClearance, horizon: t.warnTime).map { (o, $0) } })
                  .min(by: { $0.1 < $1.1 }),
              ttc < t.warnTime else { return out }
        let urgency = 1 - ttc / t.warnTime   // 0 at warnTime, 1 at impact
        // Buzz from the nearest point of the box, so a sofa sounds like where you'd hit it.
        let near = o.closestPoint(to: xz(l))
        out.obstacle = o
        out.position = clampedElevation(listener: l, target: SIMD3(near.x, o.center.y, near.y), maxDegrees: t.maxElevationDegrees)
        out.volume = 0.3 + 0.7 * urgency
        out.buzzInterval = t.buzzFarInterval + Double(urgency) * (t.buzzNearInterval - t.buzzFarInterval)
        out.stop = ttc < t.stopTime
        out.haptic = out.stop ? .stop : .slowDown(interval: out.buzzInterval)   // taps keep time with the buzz
        return out
    }
}

/// Walking velocity from the last `window` seconds of positions.
struct VelocityTracker {
    private var samples: [(t: Double, p: SIMD2<Float>)] = []

    mutating func add(_ p: SIMD2<Float>, now: Double, window: Double) {
        samples.append((now, p))
        samples.removeAll { now - $0.t > window }
    }

    var velocity: SIMD2<Float> {
        guard let a = samples.first, let b = samples.last, b.t - a.t > 0.2 else { return .zero }
        return (b.p - a.p) / Float(b.t - a.t)
    }
}
