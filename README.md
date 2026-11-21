# SoundGuide

An iPhone app that guides someone through a room with 3D sound. A sighted helper places waypoints and obstacles with ARKit. The walker then hears the next waypoint ping from its real direction, and obstacles buzz when they're close. It runs on any recent iPhone and doesn't need LiDAR. Wear headphones.

## Run
1. Open `SoundGuide.xcodeproj`, or regenerate it with `xcodegen generate` from `project.yml`.
2. Set your signing team. Choose an iPhone as the target and press Run. ARKit doesn't run in the Simulator.
3. Run the unit tests (guidance logic only) with `xcodebuild test -scheme SoundGuide -destination 'platform=iOS Simulator,name=iPhone 17e'`.

All the guidance numbers, such as radii, ping speeds and the elevation cap, live in the `Tuning` struct in `SoundGuide/Guidance.swift`. The design is described in [PLAN.md](PLAN.md).

This is a hackathon demo, not a mobility aid. It only knows about the obstacles you place.
