import SwiftUI
import simd

/// Snapshot of what the guidance thinks, for the top-down map.
struct GuideMap: Equatable {
    var me: SIMD2<Float>
    var heading: Float               // Guidance.yaw convention
    var waypoints: [SIMD2<Float>]
    var index: Int                   // current waypoint
    var obstacles: [Obstacle]
    var path: [SIMD2<Float>]
    var touching: Set<Int>
    var everHit: Set<Int>
    var hits: Int
}

/// Top-down view, auto-fitted. Up on screen is -z: the way the phone faced when the AR session started.
struct GuideMapView: View {
    let map: GuideMap

    var body: some View {
        Canvas { ctx, size in
            var pts = map.waypoints + map.path + [map.me]
            for o in map.obstacles { pts += o.corners }
            let lo = pts.reduce(pts[0]) { simd_min($0, $1) } - 0.5
            let hi = pts.reduce(pts[0]) { simd_max($0, $1) } + 0.5
            let mid = (lo + hi) / 2
            let scale = min(size.width, size.height) / CGFloat(max(hi.x - lo.x, hi.y - lo.y, 2))
            func pt(_ p: SIMD2<Float>) -> CGPoint {
                CGPoint(x: size.width / 2 + CGFloat(p.x - mid.x) * scale, y: size.height / 2 + CGFloat(p.y - mid.y) * scale)
            }
            func dot(_ p: SIMD2<Float>, _ r: CGFloat) -> Path {
                Path(ellipseIn: CGRect(x: pt(p).x - r, y: pt(p).y - r, width: 2 * r, height: 2 * r))
            }

            for (i, o) in map.obstacles.enumerated() {
                var box = Path()
                box.addLines(o.corners.map(pt))
                box.closeSubpath()
                let color: Color = map.touching.contains(i) ? .yellow : map.everHit.contains(i) ? .orange : .red
                ctx.fill(box, with: .color(color.opacity(0.7)))
            }

            if map.path.count > 1 {
                var route = Path()
                route.addLines(map.path.map(pt))
                ctx.stroke(route, with: .color(.cyan), style: StrokeStyle(lineWidth: 2, dash: [5, 4]))
            }

            for (i, w) in map.waypoints.enumerated() {
                let color: Color = i < map.index ? .gray : .green
                ctx.fill(dot(w, i == map.index ? 8 : 5), with: .color(color))
                ctx.draw(Text("\(i + 1)").font(.caption2.bold()).foregroundColor(.white), at: CGPoint(x: pt(w).x, y: pt(w).y - 14))
            }

            // You: dot plus a heading wedge.
            let f = SIMD2(-sin(map.heading), -cos(map.heading)), side = SIMD2(-f.y, f.x)
            var wedge = Path()
            wedge.addLines([pt(map.me + f * 0.45), pt(map.me + side * 0.15), pt(map.me - side * 0.15)])
            wedge.closeSubpath()
            ctx.fill(wedge, with: .color(.white))
            ctx.fill(dot(map.me, 6), with: .color(.blue))
        }
        .background(Color.black.opacity(0.6), in: RoundedRectangle(cornerRadius: 12))
        .accessibilityHidden(true)   // the walker gets the same information by speech
    }
}
