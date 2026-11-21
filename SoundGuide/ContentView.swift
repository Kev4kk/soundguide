import SwiftUI

struct ContentView: View {
    @StateObject private var ar = ARController()

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
            Spacer()
            Image(systemName: "plus").font(.system(size: 36, weight: .light))
                .foregroundStyle(.white).shadow(radius: 2)
                .accessibilityHidden(true)
            Spacer()
            HStack {
                Button("Waypoint") { ar.place(.waypoint) }.tint(.green)
                Button("Obstacle") { ar.place(.obstacle) }.tint(.red)
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

    // Covers the camera; the walker only needs one big button.
    private var guideScreen: some View {
        Button { ar.stopGuiding() } label: {
            VStack(spacing: 24) {
                Text(ar.status).font(.title2)
                Text("Stop").font(.system(size: 64, weight: .bold))
            }
            .frame(maxWidth: .infinity, maxHeight: .infinity)
            .foregroundStyle(.white)
            .background(Color.black)
        }
        .ignoresSafeArea()
        .accessibilityLabel("Stop guiding. \(ar.status)")
    }
}
