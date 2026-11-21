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
        return box(around: p, yaws: yaws, pad: 0, minHalf: t.minDetectedHalfSize, y: c.map(\.y).reduce(0, +) / 4)
    }

    static let clusterYaws = stride(from: Float(0), to: .pi / 2, by: .pi / 36).map { $0 }   // every 5°; a box repeats after 90°

    /// One box per cluster of cell centres, halved along its long side while it's mostly empty (an L-shaped corner).
    static func boxes(_ pts: [SIMD2<Float>], y: Float, tuning t: Tuning) -> [Obstacle] {
        let s = t.detectCell
        let o = box(around: pts, yaws: clusterYaws, pad: s / 2, minHalf: s / 2, y: y)
        let fill = Float(pts.count) * s * s / (4 * o.halfSize.x * o.halfSize.y)
        guard fill < t.minBoxFill, pts.count > 1 else { return [o] }
        let long = o.halfSize.x >= o.halfSize.y ? SIMD2(cos(o.yaw), -sin(o.yaw)) : SIMD2(-sin(o.yaw), -cos(o.yaw))
        let mid = simd_dot(xz(o.center), long)
        let a = pts.filter { simd_dot($0, long) < mid }, b = pts.filter { simd_dot($0, long) >= mid }
        guard !a.isEmpty, !b.isEmpty else { return [o] }
        return boxes(a, y: y, tuning: t) + boxes(b, y: y, tuning: t)
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
            out += Detection.boxes(cluster.map { (SIMD2<Float>($0) + 0.5) * t.detectCell }, y: floorY, tuning: t)
        }
        return out
    }
}
