import SwiftUI
import simd

/// Snapshot of the room for the top-down map.
struct GuideMap: Equatable {
    var me: SIMD2<Float>
    var heading: Float               // Guidance.yaw convention
    var obstacles: [Detected]
    var warned: Obstacle?            // buzzing right now (navigation footprint)
    var beepDistance: Float?         // proximity beeps: measured clearance straight ahead
}

/// Heading-up projection: the way you face is always up, you sit at `anchor`.
struct MapFrame {
    var me: SIMD2<Float>, heading: Float
    var anchor: CGPoint
    var pointsPerMetre: CGFloat

    private var fwd: SIMD2<Float> { SIMD2(-sin(heading), -cos(heading)) }
    private var right: SIMD2<Float> { SIMD2(cos(heading), -sin(heading)) }

    func screen(_ p: SIMD2<Float>) -> CGPoint {
        let d = p - me
        return CGPoint(x: anchor.x + CGFloat(simd_dot(d, right)) * pointsPerMetre,
                       y: anchor.y - CGFloat(simd_dot(d, fwd)) * pointsPerMetre)
    }

    func world(_ s: CGPoint) -> SIMD2<Float> {
        let x = Float((s.x - anchor.x) / pointsPerMetre), ahead = Float((anchor.y - s.y) / pointsPerMetre)
        return me + right * x + fwd * ahead
    }
}

struct GuideMapView: View {
    let map: GuideMap
    let remove: (SIMD2<Float>) -> Void
    let find: (SIMD2<Float>) -> Detected?
    @State private var pending: SIMD2<Float>?

    static let visibleMetres: CGFloat = 7   // across the narrow side of the view

    private func frame(_ size: CGSize) -> MapFrame {
        // You sit low on the screen, so most of the view is what's ahead.
        MapFrame(me: map.me, heading: map.heading, anchor: CGPoint(x: size.width / 2, y: size.height * 0.68),
                 pointsPerMetre: min(size.width, size.height) / Self.visibleMetres)
    }

    var body: some View {
        GeometryReader { geo in
            let f = frame(geo.size)
            Canvas { ctx, _ in
                func circle(_ c: CGPoint, _ r: CGFloat) -> Path {
                    Path(ellipseIn: CGRect(x: c.x - r, y: c.y - r, width: 2 * r, height: 2 * r))
                }

                for r in [1, 2] as [CGFloat] {
                    ctx.stroke(circle(f.anchor, r * f.pointsPerMetre), with: .color(.white.opacity(0.25)),
                               style: StrokeStyle(lineWidth: 1, dash: [4, 4]))
                    ctx.draw(Text("\(Int(r)) m").font(.caption2).foregroundColor(.white.opacity(0.5)),
                             at: CGPoint(x: f.anchor.x + 14, y: f.anchor.y - r * f.pointsPerMetre + 8))
                }

                // Same colours as the camera's hitboxes: orange = walls, surfaces and points, yellow = recognised objects.
                for d in map.obstacles {
                    var box = Path()
                    box.addLines(d.obstacle.corners.map(f.screen))
                    box.closeSubpath()
                    let warned = map.warned == d.obstacle.navigationFootprint
                    let color: Color = warned ? .red : d.source == .object ? .yellow : .orange
                    ctx.fill(box, with: .color(color.opacity(warned ? 0.9 : 0.55)))
                    if warned { ctx.stroke(box, with: .color(.white), lineWidth: 2) }
                }

                // Where the proximity beeps say the way ahead is blocked.
                if let d = map.beepDistance {
                    let y = f.anchor.y - CGFloat(d) * f.pointsPerMetre
                    var bar = Path()
                    bar.move(to: CGPoint(x: f.anchor.x - 0.3 * f.pointsPerMetre, y: y))
                    bar.addLine(to: CGPoint(x: f.anchor.x + 0.3 * f.pointsPerMetre, y: y))
                    ctx.stroke(bar, with: .color(.red), style: StrokeStyle(lineWidth: 5, lineCap: .round))
                }

                // You: always pointing up.
                var wedge = Path()
                wedge.addLines([CGPoint(x: f.anchor.x, y: f.anchor.y - 22),
                                CGPoint(x: f.anchor.x - 9, y: f.anchor.y + 4), CGPoint(x: f.anchor.x + 9, y: f.anchor.y + 4)])
                wedge.closeSubpath()
                ctx.fill(wedge, with: .color(.white))
                ctx.fill(circle(f.anchor, 7), with: .color(.blue))
            }
            .contentShape(Rectangle())
            .onTapGesture { p in
                let w = f.world(p)
                if find(w) != nil { pending = w }
            }
        }
        .confirmationDialog("Remove this obstacle?", isPresented: Binding(get: { pending != nil }, set: { if !$0 { pending = nil } }),
                            titleVisibility: .visible) {
            Button("Remove", role: .destructive) { if let p = pending { remove(p) } }
        } message: {
            Text("It won't come back if it's detected again.")
        }
        .accessibilityHidden(true)   // the walker gets the same information by sound
    }
}
