import SwiftUI

struct ContentView: View {
    enum Mode: String, CaseIterable { case camera = "Camera", map = "Map", user = "User" }

    @StateObject private var ar = ARController()
    @State private var mode = Mode.camera

    var body: some View {
        ZStack {
            // Always running, even behind the map: tracking and detection need the camera.
            ARViewContainer(controller: ar).ignoresSafeArea()
            if mode == .map { mapView }
            if mode == .user { UserView(asker: ar.asker, begin: ar.beginAsk, end: ar.endAsk) }
            VStack(spacing: 8) {
                Picker("View", selection: $mode) {
                    ForEach(Mode.allCases, id: \.self) { Text($0.rawValue) }
                }
                .pickerStyle(.segmented).padding(.horizontal)
                if mode != .user {
                    statusPills
                    Spacer()
                    bottomBar
                }
            }
            .padding(.vertical, 8)
            .frame(maxHeight: .infinity, alignment: .top)
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
            Toggle(isOn: $ar.autoDetect) {
                Text("Detect (\(ar.detectedCount))").font(.subheadline.weight(.semibold))
            }
            .tint(.orange).fixedSize()
            .padding(.horizontal, 12).padding(.vertical, 6)
            .background(.ultraThinMaterial, in: Capsule())
        }
        .padding(.horizontal)
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
                    .opacity(asker.canAsk ? 1 : 0.5)
                    .modifier(HoldToAsk(enabled: asker.canAsk, holding: $holding, begin: begin, end: end))
                    .accessibilityAddTraits(.isButton)
                    .accessibilityHint("Double-tap and hold, ask about what's in front of the camera, then let go")
            }
        }

        private var label: String {
            switch asker.state {
            case .unsupported, .ready: "Hold to ask"
            case .unavailable(let why): why
            case .listening: asker.heard.isEmpty ? "Listening…" : asker.heard
            case .transcribing, .thinking: "Thinking…"
            }
        }
    }

    // The walker's view: blank, and the whole screen is the Ask button.
    private struct UserView: View {
        @ObservedObject var asker: Asker
        let begin: () -> Void, end: () -> Void
        @State private var holding = false

        var body: some View {
            Color.black.ignoresSafeArea()
                .contentShape(Rectangle())
                .modifier(HoldToAsk(enabled: asker.canAsk, holding: $holding, begin: begin, end: end))
                .accessibilityElement()
                .accessibilityLabel("Hold to ask")
                .accessibilityAddTraits(.isButton)
                .accessibilityHint("Double-tap and hold anywhere, ask about what's in front of the camera, then let go")
        }
    }

    /// Press starts listening, release asks. Not a Button: it needs touch-down and touch-up.
    private struct HoldToAsk: ViewModifier {
        let enabled: Bool
        @Binding var holding: Bool
        let begin: () -> Void, end: () -> Void

        func body(content: Content) -> some View {
            content
                .gesture(DragGesture(minimumDistance: 0)
                    .onChanged { _ in
                        guard enabled, !holding else { return }
                        holding = true
                        begin()
                    }
                    .onEnded { _ in release() })
                .onDisappear(perform: release)   // switching views mid-hold must not leave the mic on
        }

        private func release() {
            guard holding else { return }
            holding = false
            end()
        }
    }
}

private extension Asker {
    var canAsk: Bool { state == .ready || state == .thinking }
}
