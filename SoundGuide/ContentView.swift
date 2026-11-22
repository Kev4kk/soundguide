import SwiftUI

struct ContentView: View {
    enum Mode: String, CaseIterable { case camera = "Camera", map = "Map" }

    @StateObject private var ar = ARController()
    @State private var mode = Mode.camera
    @State private var covered = false

    var body: some View {
        ZStack {
            // Always running, even behind the map: tracking and detection need the camera.
            ARViewContainer(controller: ar).ignoresSafeArea()
            if mode == .map { mapView }
            VStack(spacing: 8) {
                Picker("View", selection: $mode) {
                    ForEach(Mode.allCases, id: \.self) { Text($0.rawValue) }
                }
                .pickerStyle(.segmented).padding(.horizontal)
                statusPills
                Spacer()
                bottomBar
            }
            .padding(.vertical, 8)
            if covered { coverScreen }
        }
    }

    @ViewBuilder private var mapView: some View {
        Color.black.ignoresSafeArea()
        if let map = ar.map {
            GuideMapView(map: map, remove: ar.removeDetected(near:), find: ar.detected(near:))
                .padding(.top, 50)
        } else {
            Text("Move the phone slowly to start the map").foregroundStyle(.secondary)
        }
    }

    private var statusPills: some View {
        VStack(spacing: 6) {
            if ar.autoDetect {
                pill(ar.proximityStatus, font: .headline)
                if mode == .camera { pill(ar.cameraStatus, font: .subheadline) }
            }
            if !ar.status.isEmpty { pill(ar.status, font: .subheadline) }
            if mode == .map { pill("Tap an obstacle to remove it", font: .caption) }
            headphoneStatus
            ForEach(ar.webURLs, id: \.self) { pill("3D view: \($0)", font: .caption) }
        }
    }

    private func pill(_ text: String, font: Font) -> some View {
        Text(text).font(font).multilineTextAlignment(.center)
            .padding(.horizontal, 12).padding(.vertical, 6)
            .background(.ultraThinMaterial, in: Capsule())
    }

    private var bottomBar: some View {
        VStack(spacing: 10) {
            AskButton(asker: ar.asker, begin: ar.beginAsk, end: ar.endAsk, height: 64, fontSize: 24)
            HStack {
                Toggle(isOn: $ar.autoDetect) {
                    Text("Detect (\(ar.detectedCount))").font(.subheadline.weight(.semibold))
                }
                .tint(.orange).fixedSize()
                .padding(.horizontal, 12).padding(.vertical, 6)
                .background(.ultraThinMaterial, in: Capsule())
                Spacer()
                Button { covered = true } label: {
                    Label("Cover screen", systemImage: "eye.slash").font(.subheadline.weight(.semibold))
                }
                .buttonStyle(.borderedProminent).tint(.black.opacity(0.7))
            }
        }
        .padding(.horizontal)
    }

    // Walker's screen: nothing to see, one big target to ask.
    private var coverScreen: some View {
        VStack(spacing: 16) {
            Text(ar.proximityStatus).font(.title2).multilineTextAlignment(.center).padding(.top, 40)
            AskButton(asker: ar.asker, begin: ar.beginAsk, end: ar.endAsk, height: .infinity, fontSize: 44)
            Button { covered = false } label: {
                Label("Show screen", systemImage: "eye").font(.title3.bold())
                    .frame(maxWidth: .infinity, minHeight: 60)
            }
            .buttonStyle(.bordered).tint(.white)
        }
        .padding()
        .foregroundStyle(.white)
        .frame(maxWidth: .infinity, maxHeight: .infinity)
        .background(Color.black.ignoresSafeArea())
    }

    private var headphoneStatus: some View {
        let (text, color): (String, Color) =
            ar.motionAccessDenied ? ("Motion access off: enable in Settings", .orange)
            : ar.usingHeadTracking ? ("Head tracking: AirPods", .green)
            : ("Direction: phone", .secondary)
        return Label(text, systemImage: "airpodspro")
            .font(.caption).foregroundStyle(color)
            .padding(.horizontal, 10).padding(.vertical, 6)
            .background(.ultraThinMaterial, in: Capsule())
    }

    // Hidden on phones without Apple Intelligence or before iOS 27.
    private struct AskButton: View {
        @ObservedObject var asker: Asker
        let begin: () -> Void, end: () -> Void
        let height: CGFloat, fontSize: CGFloat
        @State private var holding = false

        var body: some View {
            if asker.state != .unsupported {
                // Not a Button: needs touch-down and touch-up, like the Obstacle button.
                Label(label, systemImage: "mic.fill")
                    .font(.system(size: fontSize, weight: .bold)).lineLimit(2).minimumScaleFactor(0.5)
                    .foregroundStyle(.white)
                    .frame(maxWidth: .infinity, minHeight: height == .infinity ? 140 : height, maxHeight: height)
                    .background(Color.purple.opacity(holding ? 1 : 0.75), in: RoundedRectangle(cornerRadius: 20))
                    .opacity(enabled ? 1 : 0.5)
                    .gesture(DragGesture(minimumDistance: 0)
                        .onChanged { _ in
                            guard enabled, !holding else { return }
                            holding = true
                            begin()
                        }
                        .onEnded { _ in
                            guard holding else { return }
                            holding = false
                            end()
                        })
                    .accessibilityAddTraits(.isButton)
                    .accessibilityHint("Double-tap and hold, ask about what's in front of the camera, then let go")
            }
        }

        private var enabled: Bool { asker.state == .ready || asker.state == .thinking }

        private var label: String {
            switch asker.state {
            case .unsupported, .ready: "Hold to ask"
            case .unavailable(let why): why
            case .listening: asker.heard.isEmpty ? "Listening…" : asker.heard
            case .transcribing, .thinking: "Thinking…"
            }
        }
    }

}
