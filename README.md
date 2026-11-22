# SoundGuide

An iPhone app that guides someone through a room with 3D sound. A sighted helper places waypoints and obstacles with ARKit. The walker then hears the next waypoint ping from its real direction, and obstacles buzz when they're close. It runs on any recent iPhone and doesn't need LiDAR. Wear headphones.

## Run
1. Open `SoundGuide.xcodeproj`, or regenerate it with `xcodegen generate` from `project.yml`.
2. Set your signing team. Choose an iPhone as the target and press Run. ARKit doesn't run in the Simulator.
3. Run the tests (guidance logic and Core ML/Vision integration) with `xcodebuild test -scheme SoundGuide -destination 'platform=iOS Simulator,name=iPhone 17 Pro'` (substitute an installed simulator if needed).

All the guidance numbers, such as radii, ping speeds and the elevation cap, live in the `Tuning` struct in `SoundGuide/Guidance.swift`. The design is described in [PLAN.md](PLAN.md).

With auto detection on, the app starts recognizing objects as soon as the camera and model are ready. Before guiding, the screen shows one prominent object and its direction relative to the phone (for example, “Person, left”). No floor scan or waypoint is needed for these cues. These label cues do not measure distance. A separate on-device proximity check runs at up to 10 Hz when AR tracking is normal. It checks an 80 cm wide corridor up to 2 m forward from the phone for observed vertical planes and groups of repeatable feature points. It displays approximate forward clearance; beeps and haptic pulses repeat faster as clearance shrinks. Detection does not narrate objects or distances. Spoken route instructions remain available while guiding. No waypoint, YOLO label or floor scan is required. Before a classified floor exists, point-based warnings only cover a band near phone height. Hold the phone facing your direction of travel. The measurement is from the phone, not your feet; it is not a prediction of impact. Silence does not mean a clear path.

Automatic obstacles use ARKit planes and feature points plus on-device YOLO26n. See the [model license and export notes](LICENSES/YOLO26n.md).

To validate range, use a sighted tester standing still and measure from the phone to an obstacle's front surface at 0.5, 1 and 1.5 m. Repeat with a textured box, a plain wall and a chair from different angles. Record displayed range, beep cadence, time to first warning, and missing or false warnings (including the empty corridor). Do not walk toward an obstacle to test it.

This is a hackathon demo, not a mobility aid. Automatic detection can miss obstacles. Don't rely on it for navigation safety.

## 3D web view
While the app is open, it serves a live 3D view of what it has mapped. The setup screen shows the address (for example `http://192.168.1.20:8080`); open it in a browser on a laptop on the same Wi-Fi and allow the local-network prompt on the phone. The page shows the phone's position and trail, ARKit feature points, and a labelled box for each tracked YOLO object. Objects only appear once a floor has been found, and people are never mapped. The page loads three.js from a CDN, so the laptop needs internet. There is no password: anyone on the same network can view it.
