import SwiftUI

struct ContentView: View {
    @StateObject private var ar = ARController()
    @State private var holdingObstacle = false
    @State private var coverScreen = true

    var body: some View {
        ZStack {
            ARViewContainer(controller: ar).ignoresSafeArea()
            if ar.guiding { guideScreen } else { setupOverlay }
        }
    }

    private var setupOverlay: some View {
        VStack {
            Text(ar.status.isEmpty ? "Aim the + at the floor or a surface" : ar.status)
                .font(.headline).padding(10)
                .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 10))
                .padding(.top)
            headphoneStatus
            Spacer()
            Image(systemName: "plus").font(.system(size: 36, weight: .light))
                .foregroundStyle(.white).shadow(radius: 2)
                .accessibilityHidden(true)
            Spacer()
            HStack {
                Button("Waypoint", action: ar.placeWaypoint).tint(.green)
                obstacleButton
                Button("Undo", action: ar.undo).disabled(ar.placed.isEmpty)
                Button("Clear", action: ar.clear).disabled(ar.placed.isEmpty)
            }
            .buttonStyle(.borderedProminent)
            .disabled(!ar.trackingNormal)
            Button { ar.startGuiding() } label: {
                Text("Start guiding (\(ar.waypoints.count) waypoints)").frame(maxWidth: .infinity).padding(8)
            }
            .buttonStyle(.borderedProminent).tint(.blue)
            .disabled(ar.waypoints.isEmpty)
            .padding()
        }
    }

    // Press = first corner, release = opposite corner. Not a Button: it needs touch-down and touch-up.
    private var obstacleButton: some View {
        Text("Obstacle")
            .font(.body.weight(.semibold)).foregroundStyle(.white)
            .padding(.horizontal, 12).padding(.vertical, 7)
            .background(holdingObstacle ? Color.red.opacity(0.6) : Color.red, in: Capsule())
            .opacity(ar.trackingNormal ? 1 : 0.4)
            .gesture(DragGesture(minimumDistance: 0)
                .onChanged { _ in
                    guard ar.trackingNormal, !holdingObstacle else { return }
                    holdingObstacle = true
                    ar.beginObstacle()
                }
                .onEnded { _ in
                    guard holdingObstacle else { return }
                    holdingObstacle = false
                    ar.endObstacle()
                })
            .accessibilityAddTraits(.isButton)
            .accessibilityHint("Hold at one corner, move to the opposite corner, release")
            .accessibilityAction { ar.beginObstacle(); ar.endObstacle() }
    }

    private var headphoneStatus: some View {
        let (text, color): (String, Color) =
            ar.motionAccessDenied ? ("Motion access off: enable in Settings", .orange)
            : ar.headphonesReady ? ("Head tracking AirPods connected", .green)
            : ("No head tracking AirPods, using phone direction", .secondary)
        return Label(text, systemImage: "airpodspro")
            .font(.caption).foregroundStyle(color)
            .padding(.horizontal, 10).padding(.vertical, 6)
            .background(.ultraThinMaterial, in: Capsule())
    }

    // Covers the camera; the walker only needs one big button.
    private var guideScreen: some View {
        ZStack(alignment: .topTrailing) {
            if coverScreen {
                Button { ar.stopGuiding() } label: {
                    VStack(spacing: 24) {
                        guideInfo
                        Text("Stop").font(.system(size: 64, weight: .bold))
                    }
                    .padding()
                    .frame(maxWidth: .infinity, maxHeight: .infinity)
                    .foregroundStyle(.white)
                    .background(Color.black)
                }
                .ignoresSafeArea()
                .accessibilityLabel("Stop guiding. \(ar.status)")
            } else {
                // Camera visible (helper watching): info on top, normal Stop button at the bottom.
                VStack {
                    guideInfo
                        .padding(12).frame(maxWidth: .infinity)
                        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 12))
                        .padding(.horizontal).padding(.top, 56)
                    Spacer()
                    Button { ar.stopGuiding() } label: {
                        Text("Stop").font(.title2.bold()).frame(maxWidth: .infinity).padding(8)
                    }
                    .buttonStyle(.borderedProminent).tint(.red).padding()
                }
            }

            Button { coverScreen.toggle() } label: {
                Label(coverScreen ? "Show camera" : "Cover screen", systemImage: coverScreen ? "eye" : "eye.slash")
                    .font(.subheadline.weight(.semibold))
                    .padding(.horizontal, 12).padding(.vertical, 8)
            }
            .buttonStyle(.bordered).tint(.white)
            .padding()
        }
    }

    private var guideInfo: some View {
        VStack(spacing: 8) {
            Text(ar.status).font(.title3).multilineTextAlignment(.center)
            Text(ar.usingHeadTracking ? "Head tracking: AirPods" : "Direction: phone").font(.subheadline)
            if let map = ar.map {
                GuideMapView(map: map).frame(height: 260)
                Text(String(format: "You: x %.1f m, z %.1f m · %d hit%@", map.me.x, map.me.y, map.hits, map.hits == 1 ? "" : "s"))
                    .font(.caption.monospacedDigit())
            }
        }
    }
}
