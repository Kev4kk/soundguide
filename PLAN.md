# SoundGuide: spatial-audio room guide (plan)

## Goal
A sighted helper places objects in a room with an iPhone: **waypoints** to walk to, in order, and **obstacles** to keep away from. Then someone wearing headphones is guided through the room by 3D sound alone. The next waypoint calls to them from its real direction, and obstacles warn them when they get close.

## Assumptions
- Any recent non-Pro iPhone. ARKit world tracking works from camera images plus the motion sensors, so it doesn't need LiDAR. The trade-offs: it takes a few seconds of moving the phone around before surfaces are found, and blank floors or walls track worse.
- **Wired or low-latency headphones.** HRTF (the filtering that makes a sound seem to come from a direction) doesn't work through the phone speaker. AirPods add about 150–250 ms of lag, which is usable at walking speed.
- The walker holds the phone in front of their chest, pointing the way they face, or wears it on a chest strap. **The phone's pose is the listener.** AirPods head tracking is skipped (see below).
- Setup and guiding happen in the **same AR session**: the helper places objects and hands over the phone. Saving a layout to reuse later is a stretch milestone.
- iOS 17+, SwiftUI + RealityKit `ARView`, AVAudioEngine. No third-party dependencies and no audio asset files, since the tones are generated in code.

## How it works

### Setup mode (sighted helper)
1. Start a world-tracking session with `planeDetection = [.horizontal, .vertical]`. Show a "move the phone slowly" hint (`ARCoachingOverlayView`, which is built in) until tracking is normal.
2. A center crosshair raycasts to any surface: `raycast(from: center, allowing: .estimatedPlane, alignment: .any)`. Objects therefore sit where they really are, whether that's the floor, a table or a shelf, and **the sound plays from that real height**.
3. Two buttons: **Add Waypoint** adds a numbered green sphere, in order, and **Add Obstacle** adds a red cylinder. **Undo** and **Clear** are also there.
4. Lines connect the waypoints so the helper can see the route.

### Guide mode (walker)
The screen is one big Start/Stop button with VoiceOver labels, and `isIdleTimerDisabled = true` keeps it awake. Everything else is sound, speech and haptics.

On every `ARFrame`:
1. **Listener.** `env.listenerPosition` is set to the camera position, and the orientation comes from the camera's forward (`-transform.columns.2`) and up (`columns.1`). ARKit and AVAudio both use right-handed axes with y up, so no conversion is needed.
2. **Real height, clamped.** Each source plays from the object's real 3D position, except that its vertical angle from the listener is limited to **±35°**. Without the limit, a floor object at 0.5 m sits about 70° below the ears, and the sound turns into a confusing "somewhere underneath". Arrival and distance still use horizontal distance, so a target on a table counts as reached once you're standing at the table.
3. **Beacon.** A short ping plays from the current waypoint. The repeat interval maps from distance: **1.0 s at ≥4 m down to 0.2 s at 0.5 m**, like a parking sensor.
4. **On-axis cue.** When the walker faces within **±15°** of the beacon, the ping switches to a brighter tone. This resolves HRTF's front/back confusion, the same trick Soundscape uses.
5. **Arrival.** Within **0.5 m**, play a chime and a haptic, and the speech synthesizer says "Waypoint 2 of 4" with `AVSpeechSynthesizer`. Then advance to the next waypoint. After the last one, say "Arrived" and stop.
6. **Obstacles.** Inside **1.2 m**, a low buzz plays from the obstacle's position, getting louder and faster as the walker gets closer, and the beacon ducks to 30%. Below **0.4 m**, a strong haptic fires.
7. **Tracking lost.** If `camera.trackingState != .normal`, mute everything and say "Hold still". The app must never guide from a stale position.

### Audio graph
```
AVAudioPlayerNode (beacon, mono) ──┐
AVAudioPlayerNode × N (obstacles) ─┼─> AVAudioEnvironmentNode ─> mainMixer ─> out
AVAudioPlayerNode (chime, mono) ───┘
```
- Every source **must be mono**, because a stereo buffer bypasses spatialization. Set `renderingAlgorithm = .HRTFHQ` on each player.
- `distanceAttenuationParameters`: reference distance 1 m, maximum 8 m, with mild rolloff so far sounds stay audible.
- Tones are sine bursts with a 5 ms fade in and out, written into an `AVAudioPCMBuffer` at startup. The beacon is 880 Hz, or 1320 Hz on-axis. The obstacle buzz is 220 Hz square-ish. The chime is two notes.
- Audio session: `.playback` with `.mixWithOthers` off.

### Tuning knob
All the numbers above live in one `Tuning` struct: arrival radius, obstacle radius, ping interval curve, on-axis cone, elevation clamp and duck level. Room acoustics and individual hearing vary, so these values are meant to be adjusted during testing.

## Files
```
SoundGuide/
  SoundGuideApp.swift       // @main
  ContentView.swift         // setup overlay (crosshair, buttons) + guide mode screen
  ARCoordinator.swift       // UIViewRepresentable, session config, raycast, entities, per-frame callback
  SpatialAudio.swift        // engine graph, tone generation, listener/source updates, ping scheduler
  Guidance.swift            // pure logic: (pose, objects, state, Tuning) -> GuidanceOutput
SoundGuideTests/
  GuidanceTests.swift       // arrival advances index; ping interval clamps; on-axis at 0°/14°/16°;
                            // elevation clamp at near/far floor targets;
                            // obstacle inside/outside radius; uses horizontal distance only
Info.plist                  // NSCameraUsageDescription
```
`Guidance.swift` doesn't depend on ARKit or audio. It takes positions as `SIMD3<Float>` and returns what should be playing, which is why it's the only part with unit tests. ARKit doesn't run in the Simulator, so everything else has to be tested on a device.

## Milestones (hackathon pace)
| # | Step | Est. |
|---|------|------|
| 1 | Xcode project, camera permission, `ARView` running with plane detection + coaching overlay | 30m |
| 2 | Crosshair, raycast, place waypoints and obstacles, undo/clear | 1h |
| 3 | Audio graph: one hard-coded mono ping at a fixed world point, listener follows camera. **Check by ear** that left/right/behind and high/low sound right | 1h |
| 4 | `Guidance.swift` + tests: current waypoint, distance-based ping interval, arrival/advance | 1h |
| 5 | On-axis tone, obstacle proximity buzz with ducking, haptics, speech | 1h30 |
| 6 | Guide-mode screen, tracking-lost handling, VoiceOver labels | 45m |
| 7 | Blindfold test with a sighted tester; tune `Tuning` | 1h |
| 8 | *Stretch:* save and load the layout with `ARWorldMap` (Documents dir), relocalize on load | 1h30 |

## Risks
- **Front/back confusion.** Generic HRTF without head tracking often puts sounds behind the listener when they're in front. The on-axis tone fixes most of it, and turning the body to scan helps with the rest.
- **Drift and relocalization.** Room scale is fine, but tracking slips in rooms with blank walls or poor lighting. Without LiDAR this matters more. Put down a rug or a few objects with texture, keep the room well lit, and set up the waypoints in the same lighting the walker will have.
- **Phone pointing ≠ body facing.** If the walker lets the phone drift sideways, the directions rotate with it. A chest strap is the fix.
- **Only placed obstacles exist.** Anything nobody placed is invisible to the app. This is a hackathon demo, not a mobility aid, so a cane or a spotter is still required.

## Skipped (YAGNI)
- **Soundscape / OSM backend.** It's outdoor and GPS-based, so it doesn't apply at room scale.
- **AirPods head tracking** (`CMHeadphoneMotionManager`). Its yaw reference has to be aligned with the AR frame, which is fiddly. Add it if body-only orientation turns out to be too coarse.
- **Automatic path planning** around obstacles (A* on a floor grid). The helper's manual waypoints are the route. Add it if layouts get complex.
- **Detecting unplaced obstacles automatically.** This needs the LiDAR mesh, which these phones don't have.
- **Height mapped to pitch** (a higher object pings at a higher note). Generic HRTF gets up and down only roughly. Add this if testing shows people can't tell a shelf target from a floor target.
- Multi-room maps, sharing layouts between phones, Android or WebXR.
