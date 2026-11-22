import AVFoundation
import simd

/// beacon + obstacle players → environment node (HRTF) → mixer. Chime and speech are not spatial.
final class SpatialAudio {
    private let engine = AVAudioEngine()
    private let env = AVAudioEnvironmentNode()
    private let beacon = AVAudioPlayerNode()
    private let obstacle = AVAudioPlayerNode()
    private let chime = AVAudioPlayerNode()
    private let proximity = AVAudioPlayerNode()
    private let speech = AVSpeechSynthesizer()
    // Mono is required: stereo sources bypass spatialization.
    private let mono = AVAudioFormat(standardFormatWithSampleRate: 44_100, channels: 1)!

    private lazy var ping = Self.tone(mono, [880], noteDuration: 0.06)
    private lazy var pingOnAxis = Self.tone(mono, [1320], noteDuration: 0.06)
    private lazy var buzz = Self.tone(mono, [220], noteDuration: 0.08, harmonics: true)
    private lazy var proximityTone = Self.tone(mono, [520], noteDuration: 0.065)
    private lazy var chimeTone = Self.tone(mono, [660, 990], noteDuration: 0.12)

    private var lastPing: TimeInterval = 0
    private var lastBuzz: TimeInterval = 0

    init() {
        // Record for Ask without the AirPods mic: Bluetooth recording drops AirPods to mono call audio, which kills HRTF.
        // A2DP is output-only, so input falls back to the iPhone's own mic.
        try? AVAudioSession.sharedInstance().setCategory(.playAndRecord, mode: .default,
                                                         options: [.allowBluetoothA2DP, .defaultToSpeaker])
        try? AVAudioSession.sharedInstance().setActive(true)

        [beacon, obstacle, chime, proximity].forEach(engine.attach)
        engine.connect(proximity, to: engine.mainMixerNode, format: mono)
        engine.attach(env)
        engine.connect(beacon, to: env, format: mono)
        engine.connect(obstacle, to: env, format: mono)
        engine.connect(env, to: engine.mainMixerNode,
                       format: AVAudioFormat(standardFormatWithSampleRate: 44_100, channels: 2))
        engine.connect(chime, to: engine.mainMixerNode, format: mono)

        for p in [beacon, obstacle] { p.renderingAlgorithm = .HRTFHQ }
        env.distanceAttenuationParameters.distanceAttenuationModel = .inverse
        env.distanceAttenuationParameters.referenceDistance = 1
        env.distanceAttenuationParameters.maximumDistance = 8
        env.distanceAttenuationParameters.rolloffFactor = 0.5
    }

    func start() {
        if !engine.isRunning { try? engine.start() }
        [beacon, obstacle, chime].forEach { $0.play() }
        lastPing = 0; lastBuzz = 0
    }

    func stop() {
        [beacon, obstacle, chime, proximity].forEach { $0.stop() }
        engine.pause()
    }

    /// Called every AR frame. Pings are scheduled from the frame clock, so no timers.
    func update(listener: SIMD3<Float>, forward: SIMD3<Float>, output o: GuidanceOutput, now: TimeInterval) {
        env.listenerPosition = point(listener)
        let flat = SIMD3(forward.x, 0, forward.z)
        if simd_length(flat) > 0.1 {   // phone pointing straight down: keep last heading
            env.listenerVectorOrientation = AVAudio3DVectorOrientation(
                forward: point(simd_normalize(flat)), up: AVAudio3DVector(x: 0, y: 1, z: 0))
        }

        if let b = o.beaconPosition {
            beacon.position = point(b)
            beacon.volume = o.beaconVolume
            if now - lastPing >= o.pingInterval {
                beacon.scheduleBuffer(o.onAxis ? pingOnAxis : ping, at: nil, options: .interrupts)
                lastPing = now
            }
        }
        if let ob = o.obstaclePosition {
            obstacle.position = point(ob)
            obstacle.volume = o.obstacleVolume
            if now - lastBuzz >= o.buzzInterval {
                obstacle.scheduleBuffer(buzz, at: nil, options: .interrupts)
                lastBuzz = now
            }
        }
    }

    /// A short non-spatial proximity cue also works before a route has started.
    func playProximityBeep() {
        if !engine.isRunning { do { try engine.start() } catch { return } }
        if !proximity.isPlaying { proximity.play() }
        proximity.scheduleBuffer(proximityTone, at: nil, options: .interrupts)
    }

    func playChime() { chime.scheduleBuffer(chimeTone, at: nil, options: .interrupts) }

    func say(_ text: String, interrupt: Bool = true) {
        if interrupt { speech.stopSpeaking(at: .immediate) }
        speech.speak(AVSpeechUtterance(string: text))
    }

    func stopSpeaking() { speech.stopSpeaking(at: .immediate) }

    private func point(_ v: SIMD3<Float>) -> AVAudio3DPoint { AVAudio3DPoint(x: v.x, y: v.y, z: v.z) }

    /// Sine notes played back to back, 5 ms fades so they don't click.
    static func tone(_ format: AVAudioFormat, _ freqs: [Float], noteDuration: Double, harmonics: Bool = false) -> AVAudioPCMBuffer {
        let sr = Float(format.sampleRate)
        let perNote = Int(noteDuration * format.sampleRate)
        let buf = AVAudioPCMBuffer(pcmFormat: format, frameCapacity: AVAudioFrameCount(perNote * freqs.count))!
        buf.frameLength = buf.frameCapacity
        let data = buf.floatChannelData![0]
        let fade = Int(0.005 * format.sampleRate)
        for (n, f) in freqs.enumerated() {
            for i in 0..<perNote {
                let ph = 2 * Float.pi * f * Float(i) / sr
                var s = sin(ph)
                if harmonics { s = (s + sin(3 * ph) / 3 + sin(5 * ph) / 5) * 0.8 }   // buzzy, square-ish
                let env = Float(min(i, perNote - 1 - i, fade)) / Float(fade)
                data[n * perNote + i] = 0.6 * s * min(env, 1)
            }
        }
        return buf
    }
}
