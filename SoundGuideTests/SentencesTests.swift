import XCTest
@testable import SoundGuide

final class SentencesTests: XCTestCase {
    func testSplitsFinishedSentencesOnly() {
        let (done, rest) = Sentences.split("A door is ahead. There's a chair 1.5 m to your left! Mind the")
        XCTAssertEqual(done, ["A door is ahead.", "There's a chair 1.5 m to your left!"])
        XCTAssertEqual(rest, " Mind the")
    }

    func testEndPunctuationWaitsForMoreText() {
        let (done, rest) = Sentences.split("Is that a step?")   // could still be "?!" or "?\"", spoken at stream end
        XCTAssertEqual(done, [])
        XCTAssertEqual(rest, "Is that a step?")
    }
}

final class AskHistoryTests: XCTestCase {
    func testEarlierAnswersRideInTheInstructions() {
        XCTAssertEqual(Asker.instructions(history: []), Asker.basePrompt)
        let h = (1...8).map { Asker.Exchange(question: "q\($0)", answer: "a\($0)") }
        let text = Asker.instructions(history: h)
        XCTAssertTrue(text.hasPrefix(Asker.basePrompt))
        XCTAssertTrue(text.contains("Q: q8\nA: a8"))
        XCTAssertFalse(text.contains("q2"), "only the last \(Asker.historyLimit) are kept")
        XCTAssertTrue(text.contains("Q: q3"))
    }
}
