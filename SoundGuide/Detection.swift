import simd
import CoreGraphics

/// Turns ARKit's world-space planes and feature points into floor-plane `Obstacle`s.
/// No ARKit types here, so all of it runs in unit tests.
enum Detection {
    /// Smallest-area box around `pts` among the candidate yaws (Obstacle.yaw convention).
    static func box(around pts: [SIMD2<Float>], yaws: [Float], pad: Float, minHalf: Float, y: Float) -> Obstacle {
        var best: (area: Float, o: Obstacle)?
        for yaw in yaws {
            let r = SIMD2<Float>(cos(yaw), -sin(yaw)), f = SIMD2<Float>(-sin(yaw), -cos(yaw))
            let u = pts.map { simd_dot($0, r) }, w = pts.map { simd_dot($0, f) }
            let (u0, u1, w0, w1) = (u.min()!, u.max()!, w.min()!, w.max()!)
            let half = simd_max(SIMD2(u1 - u0, w1 - w0) / 2 + pad, SIMD2(repeating: minHalf))
            let c = r * (u0 + u1) / 2 + f * (w0 + w1) / 2
            if best == nil || half.x * half.y < best!.area {
                best = (half.x * half.y, Obstacle(center: SIMD3(c.x, y, c.y), halfSize: half, yaw: yaw))
            }
        }
        return best!.o
    }

    /// A plane's floor footprint from its four world corners, in order around the rectangle.
    /// Vertical planes are walls unless they start above head height; horizontal ones count only at table height.
    static func planeObstacle(corners c: [SIMD3<Float>], vertical: Bool, floorY: Float, tuning t: Tuning) -> Obstacle? {
        let h = c.map { $0.y - floorY }
        if vertical {
            guard h.min()! < t.detectMaxHeight else { return nil }
        } else {
            let mid = h.reduce(0, +) / 4
            guard mid >= t.tableMinHeight, mid <= t.tableMaxHeight else { return nil }
        }
        let p = c.map(xz)
        // A wall's vertical edges project to nothing; its horizontal ones give the direction.
        let yaws = (0..<4).compactMap { i -> Float? in
            let e = p[(i + 1) % 4] - p[i]
            return simd_length(e) > 0.01 ? atan2(-e.y, e.x) : nil
        }
        guard !yaws.isEmpty else { return nil }
        let ys = c.map(\.y)
        var obstacle = box(around: p, yaws: yaws, pad: 0, minHalf: t.minDetectedHalfSize, y: ys.reduce(0, +) / 4)
        let base = vertical ? ys.min()! : floorY
        obstacle.displayY = base...max(base + 0.1, ys.max()!)
        return obstacle
    }

    static let clusterYaws = stride(from: Float(0), to: .pi / 2, by: .pi / 36).map { $0 }   // every 5°; a box repeats after 90°

    /// One box per cluster of cell centres, halved along its long side while it's mostly empty (an L-shaped corner).
    static func boxes(_ cells: [(center: SIMD2<Float>, top: Float)], floorY: Float, tuning t: Tuning) -> [Obstacle] {
        let s = t.detectCell
        var o = box(around: cells.map(\.center), yaws: clusterYaws, pad: s / 2, minHalf: s / 2, y: floorY)
        o.displayY = floorY...max(floorY + 0.1, cells.map(\.top).max()!)
        let fill = Float(cells.count) * s * s / (4 * o.halfSize.x * o.halfSize.y)
        guard fill < t.minBoxFill, cells.count > 1 else { return [o] }
        let long = o.halfSize.x >= o.halfSize.y ? SIMD2(cos(o.yaw), -sin(o.yaw)) : SIMD2(-sin(o.yaw), -cos(o.yaw))
        let mid = simd_dot(xz(o.center), long)
        let a = cells.filter { simd_dot($0.center, long) < mid }, b = cells.filter { simd_dot($0.center, long) >= mid }
        guard !a.isEmpty, !b.isEmpty else { return [o] }
        return boxes(a, floorY: floorY, tuning: t) + boxes(b, floorY: floorY, tuning: t)
    }
}

/// Feature points remembered by ARKit identifier, so the map outlives the camera view.
/// Requiring several sightings drops one-frame noise and most moving people (ARKit doesn't keep them as stable features).
struct PointMap {
    private struct Point {
        var p: SIMD3<Float>
        var seen = 0, misses = 0
        var last: Double
        var near: Bool   // last placed from within detectRange; far estimates are too noisy to count
    }
    private var points: [UInt64: Point] = [:]
    private var cells: [SIMD2<Int32>: [(id: UInt64, p: SIMD3<Float>)]] = [:]   // stable in-band points by floor cell, as of the last build
    private var lastCarve = -Double.infinity
    private var ignored: [(center: SIMD2<Float>, radius: Float)] = []

    var count: Int { points.count }

    private func cell(_ p: SIMD2<Float>, _ size: Float) -> SIMD2<Int32> { SIMD2<Int32>(p / size, rounding: .down) }
    private func isIgnored(_ p: SIMD2<Float>) -> Bool { ignored.contains { simd_distance($0.center, p) < $0.radius } }

    /// One frame of raw feature points. Also forgets stored points that this frame saw straight through.
    mutating func add(_ pts: [SIMD3<Float>], ids: [UInt64], camera: SIMD3<Float>, now: Double, tuning t: Tuning) {
        var fresh = Set<UInt64>(), rays: [SIMD3<Float>] = []
        for (p, id) in zip(pts, ids) where !isIgnored(xz(p)) {
            let near = simd_distance(xz(p), xz(camera)) <= t.detectRange
            var pt = points[id] ?? Point(p: p, last: now, near: near)
            if near || !pt.near { pt.p = p; pt.near = near }
            pt.seen += 1; pt.last = now; pt.misses = 0
            points[id] = pt
            fresh.insert(id)
            if pt.seen >= t.minPointSightings { rays.append(p) }   // new features' depth is too unsure to carve with
        }
        guard !cells.isEmpty, now - lastCarve >= t.carveInterval else { return }
        lastCarve = now

        // The line of sight to a feature passing right by a stored point means that point is gone.
        // ponytail: capped rays at 10 Hz keep this a few ms on the main thread; move to a queue if maps get huge.
        var missed = Set<UInt64>()
        let a = xz(camera), every = max(1, rays.count / t.maxCarveRays)
        for p in stride(from: 0, to: rays.count, by: every).map({ rays[$0] }) {
            let d = p - camera, len = simd_length(d), b = xz(p), flat = simd_distance(a, b)
            guard len > t.carveMargin, flat > 1e-3 else { continue }
            var prev: SIMD2<Int32>?
            for s in stride(from: Float(0), through: min(flat, t.detectRange), by: t.detectCell / 2) {
                let c = cell(a + (b - a) * (s / flat), t.detectCell)
                guard c != prev else { continue }
                prev = c
                for (id, q) in cells[c] ?? [] where !fresh.contains(id) {
                    let along = simd_dot(q - camera, d) / len
                    if along > 0, along < len - t.carveMargin, simd_distance(q, camera + d * (along / len)) < t.carveRadius {
                        missed.insert(id)
                    }
                }
            }
        }
        for id in missed where points[id] != nil {
            points[id]!.misses += 1
            if points[id]!.misses >= t.forgetMisses { points[id] = nil }
        }
    }

    /// Never detect anything here again (the helper removed a false detection).
    mutating func ignore(around c: SIMD2<Float>, radius: Float) {
        ignored.append((c, radius))
        points = points.filter { !isIgnored(xz($0.value.p)) }
    }

    /// Stable points standing 0.1–1.8 m above the floor → occupied cells → one box per 8-connected cluster.
    mutating func build(floorY: Float, now: Double, tuning t: Tuning) -> [Obstacle] {
        points = points.filter { $0.value.seen >= t.minPointSightings || now - $0.value.last < t.unstablePointLifetime }
        cells = [:]
        for (id, pt) in points where pt.seen >= t.minPointSightings && pt.near {
            let h = pt.p.y - floorY
            guard h >= t.detectMinHeight, h <= t.detectMaxHeight else { continue }
            cells[cell(xz(pt.p), t.detectCell), default: []].append((id, pt.p))
        }
        var left = Set(cells.filter { $0.value.count >= t.minPointsPerCell }.keys)
        var out: [Obstacle] = []
        while let seed = left.min(by: { ($0.x, $0.y) < ($1.x, $1.y) }) {
            left.remove(seed)
            var cluster = [seed], i = 0
            while i < cluster.count {
                let c = cluster[i]; i += 1
                for dx: Int32 in -1...1 { for dy: Int32 in -1...1 {
                    let n = c &+ SIMD2(dx, dy)
                    if left.remove(n) != nil { cluster.append(n) }
                }}
            }
            let occupied = cluster.map { key in
                ((SIMD2<Float>(key) + 0.5) * t.detectCell, cells[key]!.map { $0.p.y }.max()!)
            }
            out += Detection.boxes(occupied, floorY: floorY, tuning: t)
        }
        return out
    }
}

/// One camera image's geometry, kept with the frame so a late detector result still uses the pose it was taken at.
struct CameraShot {
    var transform: simd_float4x4
    var intrinsics: simd_float3x3   // fx = [0][0], fy = [1][1], cx = [2][0], cy = [2][1]
    var resolution: SIMD2<Float>    // sensor image, landscape

    var position: SIMD3<Float> { SIMD3(transform.columns.3.x, transform.columns.3.y, transform.columns.3.z) }
}

extension Detection {
    /// Vision's normalized point on the portrait image (orientation .right, origin bottom-left) → sensor pixel.
    static func sensorPixel(portrait n: SIMD2<Float>, _ s: CameraShot) -> SIMD2<Float> {
        SIMD2((1 - n.y) * s.resolution.x, (1 - n.x) * s.resolution.y)
    }

    static func portraitPoint(sensor p: SIMD2<Float>, _ s: CameraShot) -> SIMD2<Float> {
        SIMD2(1 - p.y / s.resolution.y, 1 - p.x / s.resolution.x)
    }

    private static func worldRay(pixel p: SIMD2<Float>, _ s: CameraShot) -> SIMD3<Float> {
        let k = s.intrinsics
        let ray = SIMD3((p.x - k[2][0]) / k[0][0], -(p.y - k[2][1]) / k[1][1], -1)
        let w = s.transform * SIMD4(ray, 0)
        return SIMD3(w.x, w.y, w.z)
    }

    /// Where the camera ray through a sensor pixel meets the floor; nil at or above the horizon.
    static func floorHit(pixel p: SIMD2<Float>, _ s: CameraShot, floorY: Float) -> SIMD3<Float>? {
        let d = worldRay(pixel: p, s)
        guard d.y < -1e-4 else { return nil }
        return s.position + d * ((floorY - s.position.y) / d.y)
    }

    /// Sensor pixel where a world point appears; nil if it's behind the camera.
    static func project(_ p: SIMD3<Float>, _ s: CameraShot) -> SIMD2<Float>? {
        let c = s.transform.inverse * SIMD4(p, 1), k = s.intrinsics
        guard c.z < -1e-3 else { return nil }
        return SIMD2(k[0][0] * c.x / -c.z + k[2][0], k[2][1] - k[1][1] * c.y / -c.z)
    }

    /// A detector box (portrait, normalized) as a floor footprint: its bottom edge is where the object stands.
    /// ponytail: one camera can't see depth, so the footprint is as deep as it is wide; the point map measures what it can.
    static func objectFootprint(minX: Float, maxX: Float, minY: Float, maxY: Float, _ s: CameraShot, floorY: Float, tuning t: Tuning, features: [SIMD3<Float>] = []) -> Obstacle? {
        fitObject(minX: minX, maxX: maxX, minY: minY, maxY: maxY, s, floorY: floorY, tuning: t, features: features)?.obstacle
    }

    static func fitObject(minX: Float, maxX: Float, minY: Float, maxY: Float, _ s: CameraShot, floorY: Float,
                          tuning t: Tuning, features: [SIMD3<Float>]) -> (obstacle: Obstacle, measured: Bool)? {
        let midX = (minX + maxX) / 2
        // Prefer metric features on the object over assuming that its image bottom touches the floor.
        let centerRay = worldRay(pixel: sensorPixel(portrait: [midX, (minY + maxY) / 2], s), s)
        let forward = simd_normalize(xz(centerRay))
        let dx = (maxX - minX) * 0.15, dy = (maxY - minY) * 0.15
        if minX + dx < maxX - dx, minY + dy < maxY - dy, forward.x.isFinite {
            let depths = features.compactMap { p -> Float? in
                guard p.y > floorY + 0.07, let px = project(p, s) else { return nil }
                let n = portraitPoint(sensor: px, s)
                guard (minX + dx...maxX - dx).contains(n.x), (minY + dy...maxY - dy).contains(n.y) else { return nil }
                let d = simd_dot(xz(p - s.position), forward)
                return d > 0.15 && d <= t.objectRange ? d : nil
            }.sorted()
            // Nearest supported depth layer; isolated points and a distant background don't stretch the box.
            var start = 0
            for end in depths.indices {
                if end > 0, depths[end] - depths[end - 1] > 0.18 { start = end }
                guard end + 1 == depths.count || depths[end + 1] - depths[end] > 0.18 else { continue }
                let count = end - start + 1
                guard count >= max(4, depths.count / 4) else { continue }
                let near = depths[start + count / 10], far = depths[end - count / 10]
                func face(_ x: Float, _ y: Float) -> SIMD3<Float>? {
                    let r = worldRay(pixel: sensorPixel(portrait: [x, y], s), s)
                    let dot = simd_dot(xz(r), forward)
                    return dot > 0.05 ? s.position + r * (near / dot) : nil
                }
                let midY = (minY + maxY) / 2
                guard let left = face(minX, midY), let right = face(maxX, midY),
                      let bottom = face(midX, minY), let top = face(midX, maxY), top.y > bottom.y else { break }
                let width = min(t.maxObjectSize, max(0.08, simd_distance(xz(left), xz(right))))
                // Only visible surfaces are sampled: allow a shallow inferred thickness, not a full square.
                let depth = min(t.maxObjectSize, max(far - near, 0.08, width * 0.25))
                let center = xz((left + right) / 2) + forward * depth / 2
                let base = max(floorY, bottom.y)
                return (Obstacle(center: SIMD3(center.x, floorY, center.y), halfSize: [width / 2, depth / 2],
                                 yaw: Guidance.yaw(of: SIMD3(forward.x, 0, forward.y)),
                                 displayY: base...max(base + 0.08, min(top.y, base + t.maxObjectDisplayHeight))), true)
            }
        }
        guard minY > t.minBoxBottom,   // bottom cut off: where it meets the floor isn't in the picture
              let l = floorHit(pixel: sensorPixel(portrait: [minX, minY], s), s, floorY: floorY),
              let r = floorHit(pixel: sensorPixel(portrait: [maxX, minY], s), s, floorY: floorY),
              let b = floorHit(pixel: sensorPixel(portrait: [midX, minY], s), s, floorY: floorY)
        else { return nil }
        let away = xz(b) - xz(s.position), dist = simd_length(away)
        guard dist > 0.1, dist <= t.objectRange else { return nil }
        let f = away / dist, w = min(max(simd_distance(xz(l), xz(r)), t.minObjectSize), t.maxObjectSize)
        let c = xz(b) + f * w / 2
        // Intersect the top-centre ray with the upright front face through the ground hit.
        // This estimates height at the near edge without changing the routing footprint.
        let ray = worldRay(pixel: sensorPixel(portrait: [midX, maxY], s), s)
        let along = simd_dot(xz(ray), f)
        let estimatedHeight = along > 1e-4 ? s.position.y + ray.y * (dist / along) - floorY : w
        let height = min(max(estimatedHeight.isFinite ? estimatedHeight : w, t.minObjectSize), t.maxObjectDisplayHeight)
        return (Obstacle(center: SIMD3(c.x, floorY, c.y), halfSize: SIMD2(repeating: w / 2),
                         yaw: Guidance.yaw(of: SIMD3(f.x, 0, f.y)), displayY: floorY...(floorY + height)), false)
    }
}

/// Forward clearance in metres from the phone, not distance from the user's feet.
/// Only compare currently visible, repeatable ARKit landmarks. A missing estimate is unknown.
struct LiveRange {
    private var previous: [UInt64: SIMD3<Float>] = [:]
    private var lastSample = -Double.infinity
    static let reach: Float = 2.0
    static let halfWidth: Float = 0.4

    mutating func clearance(points: [SIMD3<Float>], ids: [UInt64], shot: CameraShot,
                            floor: Float?, plane: Float?, now: Double) -> Float? {
        let origin = shot.position, axis = shot.transform.columns.2
        let flat = SIMD2(-axis.x, -axis.z)
        guard simd_length(flat) > 0.5 else { previous.removeAll(); return nil }
        let forward = simd_normalize(flat), right = SIMD2(-forward.y, forward.x)
        let inverse = shot.transform.inverse
        var next: [UInt64: SIMD3<Float>] = [:], depths: [Float] = []
        for (p, id) in zip(points, ids) {
            let offset = p - origin, along = simd_dot(xz(offset), forward)
            guard along > 0, along <= Self.reach, abs(simd_dot(xz(offset), right)) <= Self.halfWidth,
                  p.y > (floor.map { $0 + 0.12 } ?? origin.y - 0.7), p.y < origin.y + 0.55 else { continue }
            let local = inverse * SIMD4(p, 1), k = shot.intrinsics
            guard local.z < -0.05 else { continue }
            let u = k[0][0] * local.x / -local.z + k[2][0], v = k[2][1] - k[1][1] * local.y / -local.z
            guard u >= 0, u < shot.resolution.x, v >= 0, v < shot.resolution.y else { continue }
            next[id] = p
            if now - lastSample < 0.35, let old = previous[id], simd_distance(p, old) <= 0.07 { depths.append(along) }
        }
        previous = next; lastSample = now
        depths.sort()
        var nearest = plane.flatMap { $0 > 0 && $0 <= Self.reach ? $0 : nil }
        // Require three supporting landmarks within a 12 cm depth layer; ignore isolated close outliers.
        for i in depths.indices where i + 2 < depths.count && depths[i + 2] - depths[i] <= 0.12 {
            nearest = min(nearest ?? .infinity, depths[i + 1]); break
        }
        return nearest
    }
}

/// Camera-relative observations, never a distance estimate or an all-clear.
struct CameraObject {
    let label: String
    let bounds: CGRect // Vision's normalized, upright portrait image

    var displayText: String {
        let direction = bounds.midX < 0.33 ? "left" : bounds.midX > 0.67 ? "right" : "center"
        return "\(label.capitalized), \(direction)"
    }

    static func primary(_ objects: [CameraObject]) -> CameraObject? {
        // Favor prominent objects in front of the camera. Area is not metric depth.
        func score(_ o: CameraObject) -> CGFloat {
            o.bounds.width * o.bounds.height / (1 + 2 * abs(o.bounds.midX - 0.5))
        }
        return objects.max { score($0) < score($1) }
    }
}

struct ProximityBeepTiming {
    private var lastTime = -Double.infinity

    mutating func shouldBeep(distance: Float, now: Double) -> Bool {
        guard distance > 0, distance <= LiveRange.reach else { return false }
        let interval = distance <= 0.65 ? 0.2 : distance <= 1.2 ? 0.45 : 0.85
        guard now - lastTime >= interval else { return false }
        lastTime = now
        return true
    }
}

struct ObjectObservation {
    let obstacle: Obstacle
    let label: String
    let imageBox: CGRect // normalized portrait, Vision origin
    var measured = false
}

/// Track identity and motion in world coordinates, not screen coordinates.
struct ObjectTracks {
    private struct Track {
        let id: Int
        let label: String
        var o: Obstacle
        var previousMeasurement: SIMD3<Float>
        var velocity = SIMD3<Float>.zero
        var firstSeen: Double
        var lastSeen: Double
        var lastMeasured = -Double.infinity
        var lastMiss = -Double.infinity
        var score: Float
        var confirmed = false   // sticky: a confirmed object stays shown until its score runs out
    }
    private var tracks: [Track] = []
    private var nextID = 0
    private var ignored: [(center: SIMD2<Float>, radius: Float)] = []

    private func isIgnored(_ p: SIMD2<Float>) -> Bool { ignored.contains { simd_distance($0.center, p) < $0.radius } }

    func obstacles(tuning t: Tuning) -> [Obstacle] {
        confirmed(tuning: t).map(\.obstacle)
    }

    func confirmed(tuning t: Tuning) -> [(id: Int, label: String, obstacle: Obstacle)] {
        tracks.compactMap { $0.confirmed ? ($0.id, $0.label, $0.o) : nil }
    }

    private static func cap(_ label: String, _ t: Tuning) -> Float { t.objectMaxScoreByLabel[label] ?? t.objectMaxScore }

    private mutating func hit(_ i: Int, now: Double, tuning t: Tuning) {
        tracks[i].score = min(tracks[i].score + t.objectHit, Self.cap(tracks[i].label, t))
        let span = Double(t.minObjectSightings - 1) * t.objectEvidenceInterval
        if tracks[i].score >= t.objectConfirmScore, now - tracks[i].firstSeen + 1e-6 >= span { tracks[i].confirmed = true }
    }

    /// Its spot is close, well inside the frame and not covered by any detection, so YOLO should have found it.
    private func inClearView(_ o: Obstacle, shot: CameraShot, occluders: [CGRect], tuning t: Tuning) -> Bool {
        guard simd_distance(xz(o.center), xz(shot.position)) < t.objectViewRange else { return false }
        let b = Self.imageBounds(o, shot)
        guard !b.isNull, b.width > 0, b.height > 0, (0.1...0.9).contains(b.midX), (0.1...0.9).contains(b.midY) else { return false }
        func covered(by r: CGRect) -> CGFloat {
            let i = b.intersection(r)
            return i.isNull ? 0 : i.width * i.height / (b.width * b.height)
        }
        // Mostly on screen, and not hidden behind (or mislabelled as) something YOLO did see.
        return covered(by: CGRect(x: 0, y: 0, width: 1, height: 1)) >= 0.7 && !occluders.contains { covered(by: $0) >= 0.3 }
    }

    /// Where an object's box lands in the portrait image (normalized, the detector's coordinates); .null if partly behind.
    static func imageBounds(_ o: Obstacle, _ shot: CameraShot) -> CGRect {
        let y = o.displayY ?? o.center.y...(o.center.y + 2 * o.halfSize.x)
        let points = o.corners.flatMap { corner in [y.lowerBound, y.upperBound].compactMap { height -> SIMD2<Float>? in
            guard let p = Detection.project([corner.x, height, corner.y], shot) else { return nil }
            return Detection.portraitPoint(sensor: p, shot)
        } }
        guard points.count == 8 else { return .null }
        let xs = points.map(\.x), ys = points.map(\.y)
        return CGRect(x: CGFloat(xs.min()!), y: CGFloat(ys.min()!), width: CGFloat(xs.max()! - xs.min()!), height: CGFloat(ys.max()! - ys.min()!))
    }

    private func overlap(_ a: CGRect, _ b: CGRect) -> Float {
        let intersection = a.intersection(b)
        guard !intersection.isNull, !intersection.isEmpty else { return 0 }
        let area = intersection.width * intersection.height
        return Float(area / (a.width * a.height + b.width * b.height - area))
    }

    /// Three separated rays must cross the interior at centre depth and terminate beyond the back.
    private func seesThrough(_ o: Obstacle, shot: CameraShot, features: [SIMD3<Float>], masks: [CGRect], margin: Float) -> Bool {
        let y = o.displayY ?? o.center.y...(o.center.y + 2 * o.halfSize.x)
        let origin = shot.position, delta = xz(o.center - origin), distance = simd_length(delta)
        guard distance > o.boundingRadius else { return false }
        let normal = delta / distance
        var hits: [SIMD3<Float>] = []
        for p in features {
            let ray = p - origin, depth = simd_dot(xz(ray), normal)
            guard depth > distance + o.boundingRadius + margin else { continue }
            let hit = origin + ray * (distance / depth)
            guard y.contains(hit.y), o.distance(to: xz(hit)) < 0.001,
                  let pixel = Detection.project(p, shot) else { continue }
            let n = Detection.portraitPoint(sensor: pixel, shot)
            guard n.x > 0.05, n.x < 0.95, n.y > 0.05, n.y < 0.95,
                  !masks.contains(where: { $0.contains(CGPoint(x: CGFloat(n.x), y: CGFloat(n.y))) }),
                  hits.allSatisfy({ simd_distance($0, hit) > 0.04 }) else { continue }
            hits.append(hit)
            if hits.count == 3 { return true }
        }
        return false
    }

    /// One detector run: `found` are its footprints, `shot` the camera it ran on.
    mutating func update(_ found: [ObjectObservation], shot: CameraShot, now: Double, tuning t: Tuning,
                         features: [SIMD3<Float>], masks: [CGRect]) {
        let input = found.filter { !isIgnored(xz($0.obstacle.center)) }
        var pairs: [(score: Float, track: Int, detection: Int)] = []
        for i in tracks.indices {
            let track = tracks[i], dt = Float(min(max(now - track.lastSeen, 0), 0.5))
            let predicted = track.previousMeasurement + track.velocity * dt
            guard !input.isEmpty, simd_distance(xz(track.o.center), xz(shot.position)) < t.objectRange + 1.5 else { continue }
            let bounds = Self.imageBounds(track.o, shot)
            for j in input.indices where input[j].label == track.label {
                let d = simd_distance(xz(predicted), xz(input[j].obstacle.center)), iou = overlap(bounds, input[j].imageBox)
                // Near on the floor, or the same thing in the image: one camera's floor estimate can be well off.
                guard d < t.objectMergeDistance + min(simd_length(track.velocity) * dt, 0.4)
                        || (d < t.objectMatchMaxDistance && iou >= t.objectMatchOverlap) else { continue }
                pairs.append((d / t.objectMergeDistance + 0.5 * (1 - iou), i, j))
            }
        }
        var matched = Set<Int>(), assignments: [Int: Int] = [:]
        for pair in pairs.sorted(by: { ($0.score, $0.track, $0.detection) < ($1.score, $1.track, $1.detection) }) where assignments[pair.detection] == nil && !matched.contains(pair.track) {
            assignments[pair.detection] = pair.track
            matched.insert(pair.track)
        }
        for (j, observation) in input.enumerated() {
            let o = observation.obstacle
            if let i = assignments[j] {
                let age = min(1 + (now - tracks[i].firstSeen) / t.objectEvidenceInterval, 10)
                let step = min((now - tracks[i].lastSeen) / t.objectEvidenceInterval, 1)
                let weight = Float(step / age)
                let shapeWeight: Float
                if observation.measured {
                    shapeWeight = now - tracks[i].lastMeasured > 1 ? 1 : max(weight, Float(step) * 0.4)
                    tracks[i].lastMeasured = now
                } else {
                    shapeWeight = now - tracks[i].lastMeasured < 1 ? 0 : weight
                }
                let dt = Float(now - tracks[i].lastSeen)
                let displacement = o.center - tracks[i].previousMeasurement
                let v = displacement / max(dt, 0.01)
                let moving = dt < 0.6 && simd_dot(v, tracks[i].velocity) > 0.12 && simd_length(displacement) > 0.01
                tracks[i].velocity = dt < 0.6 && simd_length(v) < 3 ? (tracks[i].velocity + v) / 2 : .zero
                tracks[i].previousMeasurement = o.center
                tracks[i].o.center += (o.center - tracks[i].o.center) * (moving ? 1 : weight)
                tracks[i].o.halfSize += (o.halfSize - tracks[i].o.halfSize) * shapeWeight
                // Rectangular footprints repeat after pi; interpolate along the shorter turn.
                let angle = 2 * (o.yaw - tracks[i].o.yaw)
                tracks[i].o.yaw += 0.5 * atan2(sin(angle), cos(angle)) * weight
                if let old = tracks[i].o.displayY, let new = o.displayY {
                    let bottom = old.lowerBound + (new.lowerBound - old.lowerBound) * shapeWeight
                    let top = old.upperBound + (new.upperBound - old.upperBound) * shapeWeight
                    tracks[i].o.displayY = bottom...top
                } else if let new = o.displayY {
                    tracks[i].o.displayY = new
                }
                tracks[i].lastSeen = now
                tracks[i].lastMiss = -.infinity
                hit(i, now: now, tuning: t)
            } else {
                // Suppress near-identical duplicate observations without merging adjacent objects.
                if matched.contains(where: { tracks[$0].label == observation.label && simd_distance(tracks[$0].previousMeasurement, o.center) < 0.05 }) { continue }
                tracks.append(Track(id: nextID, label: observation.label, o: o, previousMeasurement: o.center, firstSeen: now, lastSeen: now,
                                    lastMeasured: observation.measured ? now : -.infinity, score: t.objectHit))
                nextID += 1
                matched.insert(tracks.count - 1)
            }
        }
        // A leftover duplicate: an unmatched track that is the same thing in the image as a detection another
        // track of its label just took. Fold it into that track instead of letting both linger.
        var merged = Set<Int>()
        for (j, k) in assignments {
            let kept = tracks[k]
            for i in tracks.indices where !matched.contains(i) && !merged.contains(i) && tracks[i].label == kept.label
                && simd_distance(xz(tracks[i].o.center), xz(kept.o.center)) < t.objectMatchMaxDistance
                && overlap(Self.imageBounds(tracks[i].o, shot), input[j].imageBox) >= t.objectMatchOverlap {
                tracks[k].score = min(max(tracks[k].score, tracks[i].score), Self.cap(kept.label, t))
                tracks[k].confirmed = tracks[k].confirmed || tracks[i].confirmed
                tracks[k].firstSeen = min(tracks[k].firstSeen, tracks[i].firstSeen)
                merged.insert(i)
            }
        }

        // Absence needs evidence: seeing through the spot, or a clear view of it with nothing detected.
        // Looking away, or a cluttered view, is unknown and changes nothing.
        let occluders = masks + input.map(\.imageBox)
        for i in tracks.indices where !matched.contains(i) && !merged.contains(i) {
            guard simd_distance(xz(tracks[i].o.center), xz(shot.position)) < t.objectRange,
                  now - tracks[i].lastMiss + 1e-6 >= t.objectEvidenceInterval else { continue }
            let drop: Float
            if !features.isEmpty, seesThrough(tracks[i].o, shot: shot, features: features, masks: masks, margin: t.carveMargin) {
                drop = t.objectSeenThrough
            } else if inClearView(tracks[i].o, shot: shot, occluders: occluders, tuning: t) {
                drop = t.objectUnseen
            } else { continue }
            tracks[i].score -= drop
            tracks[i].lastMiss = now
        }
        tracks = tracks.enumerated().filter { i, track in
            !merged.contains(i) && track.score > 1e-4 && (track.confirmed || now - track.lastSeen <= 2)   // one-off detections fade
        }.map(\.element)
    }

    mutating func ignore(around c: SIMD2<Float>, radius: Float) {
        ignored.append((c, radius))
        tracks.removeAll { simd_distance(xz($0.o.center), c) < radius }
    }
}
