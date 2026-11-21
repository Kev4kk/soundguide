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
