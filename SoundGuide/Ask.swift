import AVFoundation
import CoreImage
import FoundationModels
import Speech

/// Hold to ask: records a spoken question, sends it with the camera image to Apple's on-device model
/// (iOS 27+), and speaks the answer a sentence at a time. Main thread only.
final class Asker: ObservableObject {
    enum State: Equatable { case unsupported, unavailable(String), ready, listening, transcribing, thinking }
    @Published private(set) var state: State = .unsupported
    @Published private(set) var heard = ""
    var busy: Bool { state == .listening || state == .transcribing || state == .thinking }

    static let instructions = """
        You help a blind person walking indoors. The image is what their phone camera sees right now, \
        pointing the way they face. Your answer is read aloud: one or two short sentences, no lists. \
        Give directions as left, right or ahead, with rough distances. If you can't tell from the image, say so.
        """

    private let say: (String) -> Void
    private let recognizer = SFSpeechRecognizer()
    private let mic = AVAudioEngine()
    private var request: SFSpeechAudioBufferRecognitionRequest?
    private var recognition: SFSpeechRecognitionTask?
    private var transcriptDone: CheckedContinuation<Void, Never>?
    private var image: CGImage?
    private var session: AnyObject?   // LanguageModelSession; untyped because the type is iOS 26+
    private var answering: Task<Void, Never>?
    private let ci = CIContext()

    init(say: @escaping (String) -> Void) {
        self.say = say
        refresh()
    }

    /// Asks for mic and speech permission once, then re-checks. Call when guiding starts.
    func prepare() {
        SFSpeechRecognizer.requestAuthorization { _ in
            AVAudioApplication.requestRecordPermission { _ in
                DispatchQueue.main.async { self.refresh() }
            }
        }
    }

    func refresh() {
        guard !busy else { return }
        guard #available(iOS 27, *) else { state = .unsupported; return }
        let model = SystemLanguageModel.default
        guard model.capabilities.contains(.vision) else { state = .unsupported; return }
        if case .unavailable(let why) = model.availability {
            state = why == .deviceNotEligible ? .unsupported
                : .unavailable(why == .appleIntelligenceNotEnabled ? "Turn on Apple Intelligence to ask" : "Apple Intelligence isn't ready yet")
            return
        }
        guard recognizer?.supportsOnDeviceRecognition == true else {
            state = .unavailable("On-device speech isn't available for this language"); return
        }
        guard SFSpeechRecognizer.authorizationStatus() == .authorized,
              AVAudioApplication.shared.recordPermission == .granted else {
            state = .unavailable("Allow microphone and speech recognition in Settings"); return
        }
        if session == nil { warmSession() }
        state = .ready
    }

    /// Press: snapshot the camera and start listening. Pressing during an answer cuts it off.
    func begin(camera: CVPixelBuffer?) {
        guard state == .ready || state == .thinking else { return }
        answering?.cancel()
        image = snapshot(camera)
        heard = ""
        do {
            try startListening()
            state = .listening
        } catch {
            say("The microphone isn't available.")
            state = .ready
        }
    }

    /// Release: finish the transcript, then ask.
    func end() {
        guard state == .listening else { return }
        state = .transcribing
        answering = Task { @MainActor [weak self] in
            guard let self else { return }
            let question = await self.stopListening()
            guard !Task.isCancelled else { return }
            if question.isEmpty || self.image == nil {
                self.say(question.isEmpty ? "I didn't catch that. Hold the button while you speak." : "The camera isn't ready.")
            } else if #available(iOS 27, *) {
                self.state = .thinking
                await self.answer(question)
            }
            if !Task.isCancelled { self.state = .ready }
        }
    }

    /// Guiding stopped: drop everything and turn the mic off.
    func cancel() {
        answering?.cancel()
        if mic.isRunning { mic.stop(); mic.inputNode.removeTap(onBus: 0) }
        recognition?.cancel()
        recognition = nil
        request = nil
        resumeTranscript()
        if busy { state = .ready }
    }

    @available(iOS 27, *)
    private func warmSession() {
        let s = LanguageModelSession(instructions: Self.instructions)
        s.prewarm()
        session = s
    }

    @available(iOS 27, *)
    private func answer(_ question: String) async {
        guard let image else { return }
        // ponytail: one fresh session per question, so earlier images don't fill the context.
        let s = session as? LanguageModelSession ?? LanguageModelSession(instructions: Self.instructions)
        warmSession()
        var seen = 0, pending = ""
        do {
            let stream = s.streamResponse {
                Attachment(image, orientation: .right)   // sensor is landscape; phone is held upright
                question
            }
            for try await part in stream {
                guard !Task.isCancelled else { return }
                pending += part.content.dropFirst(seen)
                seen = part.content.count
                let (done, rest) = Sentences.split(pending)
                pending = rest
                done.forEach(say)
            }
            let tail = pending.trimmingCharacters(in: .whitespacesAndNewlines)
            if !tail.isEmpty && !Task.isCancelled { say(tail) }
        } catch {
            if !Task.isCancelled { say("Sorry, I couldn't answer that.") }
        }
    }

    /// Copy at half size, so ARKit gets its camera buffer back right away.
    private func snapshot(_ buffer: CVPixelBuffer?) -> CGImage? {
        guard let buffer else { return nil }
        let img = CIImage(cvPixelBuffer: buffer).transformed(by: CGAffineTransform(scaleX: 0.5, y: 0.5))
        return ci.createCGImage(img, from: img.extent)
    }

    // MARK: Speech

    private func startListening() throws {
        let req = SFSpeechAudioBufferRecognitionRequest()
        req.requiresOnDeviceRecognition = true
        req.shouldReportPartialResults = true
        let input = mic.inputNode
        input.installTap(onBus: 0, bufferSize: 1024, format: input.outputFormat(forBus: 0)) { buf, _ in req.append(buf) }
        mic.prepare()
        do { try mic.start() } catch { input.removeTap(onBus: 0); throw error }
        request = req
        recognition = recognizer?.recognitionTask(with: req) { [weak self] result, error in
            DispatchQueue.main.async {
                guard let self, self.request === req else { return }   // late callback from an earlier question
                if let result { self.heard = result.bestTranscription.formattedString }
                if error != nil || result?.isFinal == true { self.resumeTranscript() }
            }
        }
    }

    /// Mic off, then wait up to 1.5 s for the recognizer's last words.
    private func stopListening() async -> String {
        guard let req = request else { return "" }
        mic.stop()
        mic.inputNode.removeTap(onBus: 0)
        req.endAudio()
        await withCheckedContinuation { c in
            transcriptDone = c
            DispatchQueue.main.asyncAfter(deadline: .now() + 1.5) { [weak self] in
                if self?.request === req { self?.resumeTranscript() }
            }
        }
        recognition?.cancel()
        recognition = nil
        request = nil
        return heard.trimmingCharacters(in: .whitespacesAndNewlines)
    }

    private func resumeTranscript() {
        transcriptDone?.resume()
        transcriptDone = nil
    }
}

enum Sentences {
    /// Splits finished sentences off a streamed answer, so speech can start before the model is done.
    /// A sentence ends at . ! or ? followed by whitespace, so "1.5 m" stays whole.
    static func split(_ text: String) -> (done: [String], rest: String) {
        var done: [String] = []
        var start = text.startIndex, i = text.startIndex
        while i < text.endIndex {
            let next = text.index(after: i)
            if ".!?".contains(text[i]), next < text.endIndex, text[next].isWhitespace {
                let s = text[start...i].trimmingCharacters(in: .whitespacesAndNewlines)
                if !s.isEmpty { done.append(s) }
                start = next
            }
            i = next
        }
        return (done, String(text[start...]))
    }
}
