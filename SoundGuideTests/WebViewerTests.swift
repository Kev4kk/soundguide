import XCTest
@testable import SoundGuide

final class WebViewerTests: XCTestCase {
    func testSnapshotJSONAndRoutes() throws {
        let chair = Obstacle(center: [1, -1, -2], halfSize: [0.25, 0.3], yaw: 0.5, displayY: -1...(-0.2))
        let snapshot = WorldSnapshot(camera: [0, 0.5, 0], forward: [0, 0, -1], floorY: -1,
                                     objects: [(id: 7, label: "chair", obstacle: chair)], points: [[0.0015, -1, 2.25]])
        let json = try XCTUnwrap(JSONSerialization.jsonObject(with: JSONEncoder().encode(snapshot)) as? [String: Any])
        XCTAssertEqual(json["cam"] as? [Double], [0, 0.5, 0])
        XCTAssertEqual(json["points"] as? [Int], [2, -1000, 2250])
        let o = try XCTUnwrap((json["objects"] as? [[String: Any]])?.first)
        XCTAssertEqual(o["id"] as? Int, 7)
        XCTAssertEqual(o["label"] as? String, "chair")
        XCTAssertEqual(o["c"] as? [Double], [1, -2])
        let y = try XCTUnwrap(o["y"] as? [Double])
        XCTAssertEqual(y[0], -1, accuracy: 1e-6)
        XCTAssertEqual(y[1], -0.2, accuracy: 1e-6)

        XCTAssertEqual(WebViewer.route("GET / HTTP/1.1\r\nHost: x\r\n\r\n"), .page)
        XCTAssertEqual(WebViewer.route("GET /state?t=1 HTTP/1.1\r\n\r\n"), .state)
        XCTAssertEqual(WebViewer.route("GET /etc/passwd HTTP/1.1\r\n\r\n"), .notFound)
        XCTAssertEqual(WebViewer.route("POST /state HTTP/1.1\r\n\r\n"), .notFound)
        XCTAssertEqual(WebViewer.route(""), .notFound)

        XCTAssertTrue(WebViewer.isCGNAT("100.64.0.121"))
        XCTAssertTrue(WebViewer.isCGNAT("100.127.255.255"))
        XCTAssertFalse(WebViewer.isCGNAT("100.128.0.1"))
        XCTAssertFalse(WebViewer.isCGNAT("10.224.1.111"))
    }
}
