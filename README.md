# SoundGuide

An iPhone app that guides someone through a room with 3D sound. A sighted helper places waypoints and obstacles with ARKit. The walker then hears the next waypoint ping from its real direction, and obstacles buzz when they're close. It runs on any recent iPhone and doesn't need LiDAR. Wear headphones.

## Run
1. Open `SoundGuide.xcodeproj`, or regenerate it with `xcodegen generate` from `project.yml`.
2. Set your signing team. Choose an iPhone as the target and press Run. ARKit doesn't run in the Simulator.
3. Run the tests (guidance logic and Core ML/Vision integration) with `xcodebuild test -scheme SoundGuide -destination 'platform=iOS Simulator,name=iPhone 17 Pro'` (substitute an installed simulator if needed).

All the guidance numbers, such as radii, ping speeds and the elevation cap, live in the `Tuning` struct in `SoundGuide/Guidance.swift`. The design is described in [PLAN.md](PLAN.md).

With auto detection on, the app starts recognizing objects as soon as the camera and model are ready. Before guiding, it announces one prominent object and its direction relative to the phone (for example, “Person, left”). No floor scan or waypoint is needed for these cues. They do not measure distance, and silence does not mean a clear path. Mapped obstacles and route guidance still need AR tracking.

Automatic obstacles use ARKit planes and feature points plus on-device YOLO26n. See the [model license and export notes](LICENSES/YOLO26n.md).

This is a hackathon demo, not a mobility aid. Automatic detection can miss obstacles. Don't rely on it for navigation safety.
