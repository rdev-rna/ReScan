// IMURecorder.swift
// ReScan
//
// Raw accelerometer + gyroscope to imu.csv in the Stray Scanner layout:
// timestamp, a_x, a_y, a_z (m/s^2), alpha_x, alpha_y, alpha_z (rad/s).
// Timestamps are seconds since boot, the same clock as ARFrame.timestamp.

import Foundation
import CoreMotion

/// Appends lines to a CSV file. Not thread-safe: use each instance from one queue.
final class CSVFileWriter {
    private var handle: FileHandle?

    init?(url: URL, header: String) {
        guard FileManager.default.createFile(atPath: url.path, contents: nil),
              let handle = try? FileHandle(forWritingTo: url) else { return nil }
        self.handle = handle
        write(header)
    }

    func write(_ line: String) {
        handle?.write(Data((line + "\n").utf8))
    }

    func close() {
        try? handle?.close()
        handle = nil
    }
}

final class IMURecorder {
    private static let gravity = 9.80665
    private static let rate = 1.0 / 200.0

    private let manager = CMMotionManager()
    private let queue: OperationQueue = {
        let q = OperationQueue()
        q.name = "com.rescan.imu"
        q.maxConcurrentOperationCount = 1
        return q
    }()

    // Touched only on `queue`.
    private var writer: CSVFileWriter?
    private var latestAccel: (time: TimeInterval, value: CMAcceleration)?
    private var latestGyro: (time: TimeInterval, value: CMRotationRate)?

    /// Starts writing `url`. Returns false when the device has no accelerometer or gyroscope.
    @discardableResult
    func start(url: URL) -> Bool {
        guard manager.isAccelerometerAvailable, manager.isGyroAvailable,
              let csv = CSVFileWriter(url: url, header: "timestamp, a_x, a_y, a_z, alpha_x, alpha_y, alpha_z")
        else { return false }
        queue.addOperation { [self] in
            writer = csv
            latestAccel = nil
            latestGyro = nil
        }
        manager.accelerometerUpdateInterval = Self.rate
        manager.gyroUpdateInterval = Self.rate
        manager.startAccelerometerUpdates(to: queue) { [weak self] data, _ in
            guard let self, let data else { return }
            self.latestAccel = (data.timestamp, data.acceleration)
            self.flush()
        }
        manager.startGyroUpdates(to: queue) { [weak self] data, _ in
            guard let self, let data else { return }
            self.latestGyro = (data.timestamp, data.rotationRate)
            self.flush()
        }
        return true
    }

    func stop() {
        manager.stopAccelerometerUpdates()
        manager.stopGyroUpdates()
        queue.addOperation { [self] in
            writer?.close()
            writer = nil
        }
    }

    /// One row per accelerometer/gyro pair, stamped with the later of the two (as Stray Scanner does).
    private func flush() {
        guard let a = latestAccel, let g = latestGyro, let writer else { return }
        let t = max(a.time, g.time)
        let s = Self.gravity
        writer.write("\(t), \(a.value.x * s), \(a.value.y * s), \(a.value.z * s), \(g.value.x), \(g.value.y), \(g.value.z)")
        latestAccel = nil
        latestGyro = nil
    }
}
