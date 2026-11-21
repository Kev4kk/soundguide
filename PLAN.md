# SoundGuide: spatial-audio room guide (plan)

## Goal
A sighted helper places objects in a room with an iPhone: **waypoints** to walk to, in order, and **obstacles** to keep away from. Then someone wearing headphones is guided through the room by 3D sound alone. The next waypoint calls to them from its real direction, and obstacles warn them when they get close.

## Assumptions
- Any recent non-Pro iPhone. ARKit world tracking works from camera images plus the motion sensors, so it doesn't need LiDAR. The trade-offs: it takes a few seconds of moving the phone around before surfaces are found, and blank floors or walls track worse.
- **Wired or low-latency headphones.** HRTF (the filtering that makes a sound seem to come from a direction) doesn't work through the phone speaker. AirPods add about 150–250 ms of lag, which is usable at walking speed.
- The walker holds the phone in front of their chest, pointing the way they face, or wears it on a chest strap. **The phone's pose is the listener**, unless head-tracking AirPods are connected. Then the AirPods' head yaw is aligned to the room using the phone's heading when guiding starts, and the app falls back to the phone whenever the AirPods drop out.
- Setup and guiding happen in the **same AR session**: the helper places objects and hands over the phone. Saving a layout to reuse later is a stretch milestone.
- iOS 17+, SwiftUI + RealityKit `ARView`, AVAudioEngine. No third-party dependencies and no audio asset files, since the tones are generated in code.

## How it works

### Setup mode (sighted helper)
1. Start a world-tracking session with `planeDetection = [.horizontal, .vertical]`. Show a "move the phone slowly" hint (`ARCoachingOverlayView`, which is built in) until tracking is normal.
2. A center crosshair raycasts to any surface: `raycast(from: center, allowing: .estimatedPlane, alignment: .any)`. Objects therefore sit where they really are, whether that's the floor, a table or a shelf, and **the sound plays from that real height**.
3. **Waypoint** adds a green sphere; waypoints are numbered in placement order. **Obstacle** is press-and-hold: the crosshair position on press is one corner, and on release it's the opposite corner. A translucent red box previews the size while you hold. The box lines up with the direction the phone faced when you pressed, so face the object squarely. A quick tap, or a drag under 15 cm, places a 40 cm box. **Undo** and **Clear** are also there.
4. Lines connect the waypoints so the helper can see the route.

### Guide mode (walker)
The covered screen is black with two big targets: **Play/Pause guide** on top and **Stop** below. `isIdleTimerDisabled = true` keeps it awake. **Guiding starts paused.** Beacon pings and spoken directions only play while the guide is on. Pressing Play also speaks the directions immediately. Safety sounds always play: obstacle buzz, "Bump", "Hold still", and arrival. Haptics always run too. A **Show camera / Cover screen** toggle in the corner swaps the black cover for the live camera, with the same Play/Pause and Stop buttons along the bottom, so a helper can watch.

On every `ARFrame`:
1. **Listener.** `env.listenerPosition` is set to the camera position, and the orientation comes from the camera's forward (`-transform.columns.2`) and up (`columns.1`). ARKit and AVAudio both use right-handed axes with y up, so no conversion is needed.
2. **Real height, clamped.** Each source plays from the object's real 3D position, except that its vertical angle from the listener is limited to **±35°**. Without the limit, a floor object at 0.5 m sits about 70° below the ears, and the sound turns into a confusing "somewhere underneath". Arrival and distance still use horizontal distance, so a target on a table counts as reached once you're standing at the table.
3. **Beacon, following a planned route.** The `Navigator` plans a route to the current waypoint with A* on a 10 cm floor grid. Grid cells within **0.35 m** of an obstacle box's edge cost 50× more, so the route stays clear of obstacles but still exists when you start inside that zone. The route is then smoothed into straight segments. The ping plays from a point **1.2 m ahead along the route** (pure pursuit), not from the waypoint itself, so the sound curves around obstacles and pulls you back if you drift. The route is replanned when you're more than **0.5 m** off it, and for each new waypoint. The ping rate still tracks distance to the waypoint: **1.0 s at ≥4 m down to 0.2 s at 0.5 m**.
4. **On-axis cue.** When the walker faces within **±15°** of the point the beacon plays from, the ping switches to a brighter tone. This resolves HRTF's front/back confusion, the same trick Soundscape uses.
5. **Arrival.** Within **0.5 m**, play a chime and a haptic, and the speech synthesizer says "Waypoint 2 of 4" with `AVSpeechSynthesizer`. Then advance to the next waypoint. After the last one, say "Arrived" and stop.
6. **Obstacles, by time to collision.** Walking velocity comes from the last **0.5 s** of positions. For each obstacle, the app works out when you'd come within **0.35 m** of its edge (the box's real size, turned the way it was placed) on your current heading. When that's under **1.5 s**, a low buzz plays from the nearest point of the box, louder and faster as impact gets closer, the beacon ducks to 30%, and the phone gives light slow-down taps. Under **0.6 s**, it gives strong stop pulses. Below **0.2 m/s** (standing or shuffling), walking away from an obstacle, or walking past it, nothing warns. Only the soonest obstacle on your heading buzzes. Within **1.0 m** of a waypoint, light taps also tell you to slow down.
7. **Tracking lost.** If `camera.trackingState != .normal`, mute everything and say "Hold still". The app must never guide from a stale position.
8. **Spoken directions when you stop.** Below **0.15 m/s** for **1.5 s**, the app says where to go and what's close, relative to where you face, using clock directions. For example: "Waypoint 2, 3.4 metres. Turn right, 3 o'clock. Obstacle at 1 o'clock, 90 centimetres." The turn is toward the route's lookahead point, not the waypoint itself. Up to 2 obstacles within 2.5 m of their edge are mentioned. It repeats every **8 s** while you stay still, and resets once you walk.
9. **Hits.** Coming within **0.15 m** of an obstacle box's edge (measured from the phone) registers one hit: an error vibration, "Bump", and a running count. The same box can only hit again after you back off another 0.15 m. Arrival announces the total, for example "Arrived, 1 obstacle hit", which makes blindfold runs comparable.
10. **Mini-map.** Both guide screens show a top-down map updated at about 10 Hz. It has your position and heading (blue dot, white wedge), the waypoints (current one large, reached ones grey), the planned route (cyan dashes) and obstacles (red, orange once hit, yellow while touching), plus your x/z coordinates and the hit count. Up on the map is the direction the phone faced when the AR session started.

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
All the numbers above live in one `Tuning` struct: arrival radius, ping curve, on-axis cone, elevation clamp, duck level, body clearance, tap obstacle size, minimum drag size, warn and stop times, minimum warning speed, lookahead and off-route distance. Room acoustics, walking speed and individual hearing vary, so these values are meant to be adjusted during testing.

## Files
```
SoundGuide/
  SoundGuideApp.swift       // @main
  ContentView.swift         // setup overlay (crosshair, buttons) + guide mode screen
  ARCoordinator.swift       // UIViewRepresentable, session config, raycast, entities, per-frame callback
  SpatialAudio.swift        // engine graph, tone generation, listener/source updates, ping scheduler
  Guidance.swift            // pure logic: Tuning, Guidance.step (one tick), Navigator (route + velocity state)
  PathPlanner.swift         // pure: A* floor grid + smoothing, pure-pursuit lookahead point
  HeadTracker.swift         // AirPods head yaw (CMHeadphoneMotionManager)
SoundGuideTests/
  GuidanceTests.swift       // arrival advances index; ping interval clamps; on-axis at 0°/14°/16°;
                            // elevation clamp at near/far floor targets;
                            // time to collision (ahead/behind/passing/still); stop vs slow-down haptics;
                            // A* detours clear of obstacles; pursuit lookahead; replan when off route; velocity
Info.plist                  // NSCameraUsageDescription
```
`Guidance.swift` and `PathPlanner.swift` don't depend on ARKit or audio. They take positions as `SIMD3<Float>` / `SIMD2<Float>` and return what should be playing, which is why they're the only parts with unit tests. ARKit doesn't run in the Simulator, so everything else has to be tested on a device.

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

## Next steps (planned, not built)
These come after testing the route following and the time-to-collision warnings.

1. **Walls and tables from ARKit planes.** No LiDAR needed. ARKit already finds flat surfaces. Add vertical planes (walls) to the planner grid as blocked cells, and horizontal planes about 0.3–1.2 m above the floor (tables, benches) as blocked areas. The helper then only places what ARKit misses, such as chair legs, bags and doorframes, and A* routes around walls with no extra code. A slow sweep of the room during setup lets ARKit map surfaces outside the current camera view, since detected planes persist.
2. **More speech.** Speaking when you stop is built. Still to do:
   - About 1.5 m before a sharp bend in the planned route, say "Turn left soon".
   - Walking more than 60° off the route for over 2 seconds triggers a short "Turn around" or left/right cue.

3. **Detecting unplaced obstacles automatically (on-device models).** No LiDAR needed.
   - **Pipeline.** Send about 10 of ARKit's 60 frames per second (`ARFrame.capturedImage`) through Vision to a YOLO11n Core ML model (`yolo export format=coreml`). For each box, raycast from its bottom-centre pixel onto the detected floor (`.estimatedPlane`, horizontal) to get the object's floor position. Merge a detection with a known obstacle if it's within about 0.5 m, add it as an obstacle only after 3 or more sightings, and drop it if it goes unseen while in view. Auto obstacles feed the same list the A* planner and collision warnings use.
   - **Limits.** It only finds the ~80 COCO object types (people, chairs, tables, couches, bikes, plants, bags). Walls come from step 1. Stairs, poles and doors would need a custom-trained model. It sees only the 1× camera's view (ARKit offers no ultra-wide format on the iPhone 17, checked on the device), and positions are off by roughly 10–30 cm.
   - **Fallback that catches anything: a depth model.** Depth Anything V2 Small (Core ML) estimates distance for every pixel at about 5 per second. Something big and close straight ahead triggers the stop warning, even when YOLO has no name for it.
   - **Size and speed** (published figures, not yet measured on this phone): YOLO11n is about 5–6 MB and takes a few ms per frame. Depth Anything V2 Small is about 50 MB and takes about 20–35 ms per frame. Both run on the Neural Engine, so they don't compete with ARKit (CPU/GPU), and together use well under a quarter of it at these rates. They add about 55 MB to the app and a few hundred MB of memory (the phone has 8 GB). Don't use the base or large depth models (200 MB or more, too slow).
   - **Watch.** Heat and battery on walks over 30 minutes; if the phone throttles, run depth only while moving or when YOLO sees nothing ahead. Load both models in the background at launch, since loading takes 1–2 seconds.
   - **First step.** A test build that times both models on the iPhone 17 before building the pipeline.

Later, if needed: saving and reloading the room map with `ARWorldMap` (also fixes drift between sessions).


## Risks
- **Front/back confusion.** Generic HRTF without head tracking often puts sounds behind the listener when they're in front. The on-axis tone fixes most of it, and turning the body to scan helps with the rest.
- **Drift and relocalization.** Room scale is fine, but tracking slips in rooms with blank walls or poor lighting. Without LiDAR this matters more. Put down a rug or a few objects with texture, keep the room well lit, and set up the waypoints in the same lighting the walker will have.
- **Phone pointing ≠ body facing.** If the walker lets the phone drift sideways, the directions rotate with it. A chest strap is the fix.
- **Only placed obstacles exist** (until next step 3 is built). Anything nobody placed is invisible to the app. This is a hackathon demo, not a mobility aid, so a cane or a spotter is still required.

## Skipped (YAGNI)
- **Soundscape / OSM backend.** It's outdoor and GPS-based, so it doesn't apply at room scale.
- **Height mapped to pitch** (a higher object pings at a higher note). Generic HRTF gets up and down only roughly. Add this if testing shows people can't tell a shelf target from a floor target.
- Multi-room maps, sharing layouts between phones, Android or WebXR.
