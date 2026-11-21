import simd

/// Floor-plane route planning. Everything is 2D (x, z); height doesn't matter for walking.
enum PathPlanner {
    /// A* on a floor grid, smoothed to straight segments. Cells within `clearance` of an obstacle
    /// cost extra instead of being walls, so a start or goal inside that zone still gets a route
    /// (it just leaves the zone by the shortest way).
    /// ponytail: grid rebuilt per plan; ~150×150 cells at 10 cm, cache it if rooms get much bigger.
    static func plan(from start: SIMD2<Float>, to goal: SIMD2<Float>, obstacles: [SIMD2<Float>],
                     clearance: Float, cell: Float = 0.1) -> [SIMD2<Float>] {
        guard !obstacles.isEmpty else { return [start, goal] }

        let pad: Float = 2
        let pts = obstacles + [start, goal]
        let lo = pts.reduce(pts[0]) { simd_min($0, $1) } - pad
        let hi = pts.reduce(pts[0]) { simd_max($0, $1) } + pad
        let w = Int(((hi.x - lo.x) / cell).rounded(.up)), h = Int(((hi.y - lo.y) / cell).rounded(.up))

        func center(_ i: Int) -> SIMD2<Float> { lo + (SIMD2(Float(i % w), Float(i / w)) + 0.5) * cell }
        func index(_ p: SIMD2<Float>) -> Int {
            let q = (p - lo) / cell
            let x = min(max(Int(q.x.rounded(.down)), 0), w - 1), y = min(max(Int(q.y.rounded(.down)), 0), h - 1)
            return y * w + x
        }
        let blocked = (0..<(w * h)).map { i in obstacles.contains { simd_distance(center(i), $0) < clearance } }

        let s = index(start), g = index(goal)
        let penalty: Float = 50
        var cost = [Float](repeating: .infinity, count: w * h)
        var parent = [Int](repeating: -1, count: w * h)
        var open = MinHeap()
        cost[s] = 0
        open.push(s, simd_distance(center(s), center(g)))

        while let cur = open.pop() {
            if cur == g { break }
            let cx = cur % w, cy = cur / w
            for dy in -1...1 { for dx in -1...1 where dx != 0 || dy != 0 {
                let nx = cx + dx, ny = cy + dy
                guard nx >= 0, ny >= 0, nx < w, ny < h else { continue }
                let n = ny * w + nx
                let step = (dx != 0 && dy != 0 ? 1.4142 : 1) * cell * (blocked[n] ? penalty : 1)
                let c = cost[cur] + step
                if c < cost[n] {
                    cost[n] = c
                    parent[n] = cur
                    open.push(n, c + simd_distance(center(n), center(g)))
                }
            }}
        }

        var cells = [g]
        while let p = cells.last, p != s, parent[p] >= 0 { cells.append(parent[p]) }
        cells.reverse()
        let onPath = Set(cells)

        // A segment is "clear" if it only crosses free cells or cells the raw path already had to use.
        func clear(_ a: SIMD2<Float>, _ b: SIMD2<Float>) -> Bool {
            let n = max(1, Int(simd_distance(a, b) / (cell / 2)))
            return (0...n).allSatisfy { k in
                let i = index(simd_mix(a, b, SIMD2(repeating: Float(k) / Float(n))))
                return !blocked[i] || onPath.contains(i)
            }
        }

        // String pulling: from each kept point, jump to the farthest point still in line of sight.
        let raw = [start] + cells.dropFirst().dropLast().map(center) + [goal]
        var out = [start], i = 0
        while i < raw.count - 1 {
            var j = raw.count - 1
            while j > i + 1 && !clear(raw[i], raw[j]) { j -= 1 }
            out.append(raw[j])
            i = j
        }
        return out
    }

    /// Pure pursuit: the point `lookahead` metres further along the path than the closest point to `p`.
    /// Steering at it curves you around corners and pulls you back if you drift. `offset` is how far off the path you are.
    static func pursuit(path: [SIMD2<Float>], from p: SIMD2<Float>, lookahead: Float) -> (carrot: SIMD2<Float>, offset: Float) {
        guard path.count > 1 else { return (path.first ?? p, 0) }
        var best = (seg: 0, t: Float(0), d: Float.infinity)
        for k in 0..<(path.count - 1) {
            let a = path[k], ab = path[k + 1] - a
            let len2 = simd_length_squared(ab)
            let t = len2 > 0 ? min(max(simd_dot(p - a, ab) / len2, 0), 1) : 0
            let d = simd_distance(p, a + ab * t)
            if d < best.d { best = (k, t, d) }
        }
        var k = best.seg
        var pos = path[k] + (path[k + 1] - path[k]) * best.t
        var remaining = lookahead
        while true {
            let toEnd = simd_distance(pos, path[k + 1])
            if remaining <= toEnd { return (pos + simd_normalize(path[k + 1] - pos) * remaining, best.d) }
            remaining -= toEnd
            pos = path[k + 1]
            k += 1
            if k == path.count - 1 { return (pos, best.d) }
        }
    }
}

/// Binary min-heap of (cell, priority). Stale duplicates are fine: A* skips nothing it needs.
private struct MinHeap {
    private var items: [(Int, Float)] = []

    mutating func push(_ v: Int, _ p: Float) {
        items.append((v, p))
        var i = items.count - 1
        while i > 0, items[(i - 1) / 2].1 > items[i].1 { items.swapAt(i, (i - 1) / 2); i = (i - 1) / 2 }
    }

    mutating func pop() -> Int? {
        guard !items.isEmpty else { return nil }
        let top = items[0].0
        items[0] = items[items.count - 1]
        items.removeLast()
        var i = 0
        while true {
            let l = 2 * i + 1, r = l + 1
            var m = i
            if l < items.count, items[l].1 < items[m].1 { m = l }
            if r < items.count, items[r].1 < items[m].1 { m = r }
            if m == i { return top }
            items.swapAt(i, m)
            i = m
        }
    }
}
