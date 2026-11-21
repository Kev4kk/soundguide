import simd

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
    static func objectFootprint(minX: Float, maxX: Float, minY: Float, maxY: Float, _ s: CameraShot, floorY: Float, tuning t: Tuning) -> Obstacle? {
        let midX = (minX + maxX) / 2
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
        let forward = simd_dot(xz(ray), f)
        let estimatedHeight = forward > 1e-4 ? s.position.y + ray.y * (dist / forward) - floorY : w
        let height = min(max(estimatedHeight.isFinite ? estimatedHeight : w, t.minObjectSize), t.maxObjectDisplayHeight)
        return Obstacle(center: SIMD3(c.x, floorY, c.y), halfSize: SIMD2(repeating: w / 2),
                        yaw: Guidance.yaw(of: SIMD3(f.x, 0, f.y)), displayY: floorY...(floorY + height))
    }
}

/// Detector boxes merged across frames into world-space objects. Confirmed after a few sightings;
/// forgotten after several runs in which its spot was in view and nothing was detected there.
struct ObjectTracks {
    private struct Track { var o: Obstacle; var seen = 1, misses = 0 }
    private var tracks: [Track] = []
    private var ignored: [(center: SIMD2<Float>, radius: Float)] = []

    private func isIgnored(_ p: SIMD2<Float>) -> Bool { ignored.contains { simd_distance($0.center, p) < $0.radius } }

    func obstacles(tuning t: Tuning) -> [Obstacle] { tracks.filter { $0.seen >= t.minObjectSightings }.map(\.o) }

    /// One detector run: `found` are its footprints, `shot` the camera it ran on.
    mutating func update(_ found: [Obstacle], shot: CameraShot, tuning t: Tuning) {
        var matched = Set<Int>()
        for o in found where !isIgnored(xz(o.center)) {
            let near = tracks.indices.min { simd_distance(tracks[$0].o.center, o.center) < simd_distance(tracks[$1].o.center, o.center) }
            if let i = near, simd_distance(xz(tracks[i].o.center), xz(o.center)) < t.objectMergeDistance {
                let n = Float(min(tracks[i].seen + 1, 10))   // running mean, then slow drift
                tracks[i].o.center += (o.center - tracks[i].o.center) / n
                tracks[i].o.halfSize += (o.halfSize - tracks[i].o.halfSize) / n
                if let old = tracks[i].o.displayY, let new = o.displayY {
                    let bottom = old.lowerBound + (new.lowerBound - old.lowerBound) / n
                    let top = old.upperBound + (new.upperBound - old.upperBound) / n
                    tracks[i].o.displayY = bottom...top
                } else if let new = o.displayY {
                    tracks[i].o.displayY = new
                }
                tracks[i].seen += 1
                tracks[i].misses = 0
                matched.insert(i)
            } else {
                tracks.append(Track(o: o))
                matched.insert(tracks.count - 1)
            }
        }
        // Its spot is well inside the picture and close enough to be found, yet nothing was: a miss.
        for i in tracks.indices where !matched.contains(i) {
            guard simd_distance(xz(tracks[i].o.center), xz(shot.position)) < t.objectRange,
                  let px = Detection.project(tracks[i].o.center, shot) else { continue }
            let n = Detection.portraitPoint(sensor: px, shot)
            if n.x > 0.1, n.x < 0.9, n.y > 0.1, n.y < 0.9 { tracks[i].misses += 1 }
        }
        tracks.removeAll { $0.misses >= t.objectForgetMisses }
    }

    mutating func ignore(around c: SIMD2<Float>, radius: Float) {
        ignored.append((c, radius))
        tracks.removeAll { simd_distance(xz($0.o.center), c) < radius }
    }
}
