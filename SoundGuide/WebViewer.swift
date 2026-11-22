import Foundation
import Network
import simd

/// What the browser page draws. World coordinates are ARKit's: metres, y up.
struct WorldSnapshot: Encodable, Equatable {
    struct Object: Encodable, Equatable {
        let id: Int
        let label: String
        let c: [Float]      // box centre x, z
        let half: [Float]   // half extents along the box's own right / forward axes
        let yaw: Float
        let y: [Float]      // bottom, top
    }
    let cam: [Float]
    let fwd: [Float]
    let floorY: Float?
    let objects: [Object]
    let range: Float?       // forward clearance the phone is beeping for, nil when it is silent
    let head: [Float]?      // ear x, y, z + AirPods forward x, y, z; nil when the phone's pose is the listener

    init(camera: SIMD3<Float>, forward: SIMD3<Float>, floorY: Float?,
         objects: [(id: Int, label: String, obstacle: Obstacle)], range: Float? = nil,
         head: (ear: SIMD3<Float>, forward: SIMD3<Float>)? = nil) {
        self.head = head.map { [$0.ear.x, $0.ear.y, $0.ear.z, $0.forward.x, $0.forward.y, $0.forward.z] }
        self.range = range
        cam = [camera.x, camera.y, camera.z]
        fwd = [forward.x, forward.y, forward.z]
        self.floorY = floorY
        self.objects = objects.map { t in
            let o = t.obstacle, y = o.displayY ?? o.center.y...(o.center.y + 0.1)
            return Object(id: t.id, label: t.label, c: [o.center.x, o.center.z], half: [o.halfSize.x, o.halfSize.y],
                          yaw: o.yaw, y: [y.lowerBound, y.upperBound])
        }
    }
}

/// Serves the 3D viewer page and the latest snapshot to browsers on the same network. Read-only, no auth.
final class WebViewer {
    static let port: UInt16 = 8080

    enum Route: Equatable { case page, state, font, notFound }

    static func route(_ request: String) -> Route {
        let parts = request.prefix { $0 != "\r" && $0 != "\n" }.split(separator: " ")
        guard parts.count >= 2, parts[0] == "GET" else { return .notFound }
        switch parts[1].prefix(while: { $0 != "?" }) {
        case "/": return .page
        case "/state": return .state
        case "/hacked.ttf": return .font
        default: return .notFound
        }
    }

    private let queue = DispatchQueue(label: "web")
    private var listener: NWListener?
    private let lock = NSLock()
    private var state = Data("{}".utf8)
    private var lastPoll = -Double.infinity
    private let page = Bundle.main.url(forResource: "viewer", withExtension: "html").flatMap { try? Data(contentsOf: $0) } ?? Data()
    private let font = Bundle.main.url(forResource: "hacked", withExtension: "ttf").flatMap { try? Data(contentsOf: $0) } ?? Data()

    /// True while a browser is polling, so the app builds no snapshots for nobody.
    var watched: Bool {
        lock.lock(); defer { lock.unlock() }
        return ProcessInfo.processInfo.systemUptime - lastPoll < 2
    }

    func publish(_ snapshot: WorldSnapshot) {
        guard let data = try? JSONEncoder().encode(snapshot) else { return }   // NaN in a pose: keep the last good one
        lock.lock(); state = data; lock.unlock()
    }

    func start() {
        let params = NWParameters.tcp
        params.allowLocalEndpointReuse = true
        params.prohibitedInterfaceTypes = [.cellular]
        guard let l = try? NWListener(using: params, on: NWEndpoint.Port(rawValue: Self.port)!) else { return }
        // Advertising is what makes iOS show the local-network permission prompt; a bare listener may never trigger it.
        l.service = NWListener.Service(name: "SoundGuide", type: "_http._tcp")
        l.newConnectionHandler = { [weak self] c in self?.serve(c) }
        l.stateUpdateHandler = { [weak self] s in
            // iOS reclaims the socket while the app is suspended.
            guard case .failed = s else { return }
            l.cancel()
            self?.queue.asyncAfter(deadline: .now() + 1) { self?.start() }
        }
        listener = l
        l.start(queue: queue)
    }

    private func serve(_ c: NWConnection) {
        c.start(queue: queue)
        // ponytail: one read, one response, close. A GET line fits in the first segment; keep-alive if polling ever costs too much.
        c.receive(minimumIncompleteLength: 1, maximumLength: 8192) { [weak self] data, _, _, _ in
            guard let self, let data else { return c.cancel() }
            switch Self.route(String(decoding: data, as: UTF8.self)) {
            case .page: self.respond(c, "200 OK", "text/html; charset=utf-8", self.page)
            case .font: self.respond(c, "200 OK", "font/ttf", self.font)
            case .state:
                self.lock.lock()
                let body = self.state
                self.lastPoll = ProcessInfo.processInfo.systemUptime
                self.lock.unlock()
                self.respond(c, "200 OK", "application/json", body)
            case .notFound: self.respond(c, "404 Not Found", "text/plain", Data("Not found".utf8))
            }
        }
    }

    private func respond(_ c: NWConnection, _ status: String, _ type: String, _ body: Data) {
        var out = Data("HTTP/1.1 \(status)\r\nContent-Type: \(type)\r\nContent-Length: \(body.count)\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n".utf8)
        out.append(body)
        c.send(content: out, completion: .contentProcessed { _ in c.cancel() })
    }

    /// The phone's IPv4 addresses: Wi-Fi (en0), its own hotspot (bridge100), and a Tailscale-style VPN.
    /// Venue Wi-Fi often blocks device-to-device traffic, so the VPN address may be the only one that works.
    static var addresses: [String] {
        var first: UnsafeMutablePointer<ifaddrs>?
        guard getifaddrs(&first) == 0 else { return [] }
        defer { freeifaddrs(first) }
        var found: [String] = [], next = first
        while let i = next?.pointee {
            next = i.ifa_next
            guard let sa = i.ifa_addr, sa.pointee.sa_family == UInt8(AF_INET) else { continue }
            var host = [CChar](repeating: 0, count: Int(NI_MAXHOST))
            guard getnameinfo(sa, socklen_t(sa.pointee.sa_len), &host, socklen_t(host.count), nil, 0, NI_NUMERICHOST) == 0 else { continue }
            let name = String(cString: i.ifa_name), ip = String(cString: host)
            if ["en0", "bridge100"].contains(name) || (name.hasPrefix("utun") && isCGNAT(ip)) { found.append(ip) }
        }
        return found
    }

    /// 100.64.0.0/10, the range Tailscale assigns from.
    static func isCGNAT(_ ip: String) -> Bool {
        let p = ip.split(separator: ".").compactMap { UInt8($0) }
        return p.count == 4 && p[0] == 100 && (64...127).contains(p[1])
    }
}
