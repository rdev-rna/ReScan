// CaptureViewModel.swift
// ReScan
//
// Orchestrates recording: ARKit frames → Stray Scanner format export.
// Handles FPS subsampling — ARKit runs at native rate, only selected frames are captured.

import Foundation
import AVFoundation
import ARKit
import Combine
import UIKit

@Observable
final class CaptureViewModel {
    
    // MARK: - Sub-ViewModels
    
    var camera = CameraViewModel()
    var lidar = LiDARViewModel()
    
    // MARK: - Session
    
    let session = CaptureSession()
    var settings = SessionSettings()
    
    // MARK: - Services
    
    private let exportService = ExportService()
    
    /// Adaptive mesh refinement service — created when refinement is enabled.
    private var meshRefinement: AdaptiveMeshRefinement?
    
    // MARK: - State
    
    var isRecording: Bool { session.state == .recording }
    var isSaving: Bool { session.state == .saving }
    /// True while the app has queued a recording start but is waiting for the first mesh polygons.
    var isWaitingForMesh: Bool = false
    var errorMessage: String?
    var showError: Bool = false
    
    // Performance
    var thermalState: ProcessInfo.ThermalState { camera.capabilities.thermalState }
    var batteryPercentage: Int { camera.capabilities.batteryPercentage }
    var storageAvailableMB: Double { session.availableStorageMB }
    
    var elapsedTimeString: String { session.elapsedTime.recordingDurationString }
    var frameCountString: String { "\(session.frameCount)" }
    
    // Active encoding mode (resolved at recording start)
    var activeEncodingMode: VideoEncodingMode = .standardHEVC
    
    // Timer
    private var recordingTimer: Timer?
    
    // Export queue
    private let exportQueue = DispatchQueue(label: "com.rescan.capturevm.export", qos: .utility)
    
    // Intrinsics saved flag
    private var intrinsicsSaved = false

    // Index of the next saved frame. Kept here (not session.frameCount, which is bumped later on the
    // main queue) so every frame gets its own index, and advanced only when the video took the frame.
    private var nextFrameIndex = 0
    private var droppedFrames = 0
    private var videoSize: (width: Int, height: Int)?
    private var motionLock: CameraService.MotionLock?
    private var imuRecording = false
    private let imuRecorder = IMURecorder()
    
    // FPS subsampling
    private var lastCaptureTimestamp: TimeInterval = -1
    private var captureInterval: TimeInterval = 1.0 / 30.0 // Default 30fps
    
    // MARK: - Computed
    
    /// Whether Apple Log (ProRes) is available on this device
    var supportsAppleLog: Bool {
        ExportService.supportsAppleLog
    }
    
    // MARK: - Setup
    
    func setup() {
        lidar.start()
        session.updateStorageInfo()
        
        // Apply User Defaults
        let defaults = AppSettings.shared
        lidar.settings.maxDistance = Float(defaults.defaultMaxDistance)
        if let conf = ConfidenceThreshold(rawValue: defaults.defaultConfidence) {
            lidar.settings.confidenceThreshold = conf
        }
        lidar.settings.smoothingEnabled = defaults.defaultSmoothing
        
        // Attach camera controls to ARKit's device
        camera.setup(device: lidar.arService.captureDevice)
        
        // Wire up AR frame callback
        lidar.arService.onFrameUpdate = { [weak self] frame in
            self?.handleARFrame(frame)
        }
    }
    
    func teardown() {
        stopRecording()
        camera.teardown()
        lidar.stop()
        recordingTimer?.invalidate()
    }
    
    // MARK: - Encoding Mode Resolution
    
    /// Determines the actual encoding mode based on settings and device capabilities
    private func resolveEncodingMode() -> VideoEncodingMode {
        let settings = AppSettings.shared
        
        // EXR conversion is now performed manually from the media library after capture.
        // Always record as video for better performance and lower resource usage during capture.
        if settings.useAppleLog {
            if supportsAppleLog {
                return .appleLog
            } else {
                print("[CaptureVM] Apple Log not supported on this device, falling back to HDR HEVC")
                return settings.enableHDR ? .hdrHEVC : .standardHEVC
            }
        } else if settings.enableHDR {
            return .hdrHEVC
        } else {
            return .standardHEVC
        }
    }
    
    // MARK: - Recording
    
    func startRecording() {
        guard !isRecording && !isWaitingForMesh else { return }
        do {
            // Resolve encoding mode
            activeEncodingMode = resolveEncodingMode()
            session.encodingMode = activeEncodingMode
            
            // Set capture FPS interval
            let appSettings = AppSettings.shared
            let motionMode = appSettings.motionCaptureMode
            // Motion capture keeps every frame ARKit delivers
            captureInterval = motionMode ? 0 : appSettings.captureFPS.captureInterval
            lastCaptureTimestamp = -1

            // Pass LiDAR state to session so it can skip depth/confidence directories
            session.lidarEnabled = appSettings.lidarEnabled

            _ = try session.createSessionDirectory()

            // Initialize adaptive mesh refinement if enabled (only with LiDAR and a scene mesh)
            if appSettings.adaptiveMeshRefinement && appSettings.lidarEnabled && !motionMode {
                meshRefinement = AdaptiveMeshRefinement(detailLevel: appSettings.meshDetailLevel)
            } else {
                meshRefinement = nil
            }

            if !appSettings.lidarEnabled || motionMode || appSettings.meshStartMode == .bruteForce {
                // Start immediately when there is no mesh to wait for
                beginCapture()
            } else {
                // WaitForPolygons: defer actual start until first mesh anchors arrive
                isWaitingForMesh = true
            }
            
        } catch {
            showError(message: "Failed to create session: \(error.localizedDescription)")
        }
    }
    
    /// Performs the actual recording start (opens video writer, odometry file, starts timer).
    /// Called either immediately (BruteForce) or upon first mesh polygon detection.
    private func beginCapture() {
        isWaitingForMesh = false
        session.startRecording()
        intrinsicsSaved = false
        nextFrameIndex = 0
        droppedFrames = 0
        videoSize = nil
        motionLock = nil

        let settings = AppSettings.shared

        // Open odometry CSV, the per-frame camera log and the IMU log
        if let odometryURL = session.odometryURL {
            exportService.openOdometryFile(at: odometryURL, convention: settings.odometryConvention)
        }
        if let dir = session.sessionDirectory {
            exportService.openFrameLog(at: dir.appendingPathComponent("frames.csv"))
            imuRecording = settings.recordIMU && imuRecorder.start(url: dir.appendingPathComponent("imu.csv"))
        }

        // Motion capture: fast shutter, focus and white balance locked as the take starts
        if settings.motionCaptureMode {
            let denominator = settings.motionShutterDenominator
            camera.applyMotionLock(shutterDenominator: denominator) { [weak self] lock in
                self?.motionLock = lock
                if lock.isoClipped {
                    self?.showError(message: "⚠️ Too dark for 1/\(denominator) s: ISO is at its maximum, so the video will be darker. Add light or pick a slower shutter.")
                }
            }
        }
        
        // Start timer
        recordingTimer = Timer.scheduledTimer(withTimeInterval: 0.1, repeats: true) { [weak self] _ in
            self?.session.updateElapsedTime()
            self?.session.updateStorageInfo()
        }
        
        let appSettings = AppSettings.shared
        if storageAvailableMB < 1024 {
            showError(message: "⚠️ Less than 1 GB storage remaining")
        }
        
        print("[CaptureVM] Recording started — \(appSettings.captureFPS.label), encoding: \(activeEncodingMode.label)")
    }
    
    func stopRecording() {
        // Cancel pending "wait for mesh" state
        if isWaitingForMesh {
            isWaitingForMesh = false
            // Clean up the session directory that was pre-created
            if let dir = session.sessionDirectory {
                try? FileManager.default.removeItem(at: dir)
                session.sessionDirectory = nil
            }
            return
        }
        
        guard isRecording else { return }
        
        // Hide ghost mesh overlay immediately so it is not visible in subsequent scans
        lidar.ghostMeshEnabled = false
        
        session.stopRecording()
        recordingTimer?.invalidate()
        recordingTimer = nil
        
        // Close odometry, the camera log and the IMU log
        exportService.closeOdometryFile()
        exportService.closeFrameLog()
        if imuRecording {
            imuRecorder.stop()
        }

        // Take metadata and the floor plane
        writeTakeInfo(anchors: lidar.arService.currentFrame?.anchors ?? [])

        // Export mesh if available
        let meshAnchors = lidar.meshAnchors
        if !meshAnchors.isEmpty, let meshURL = session.meshURL {
            if let refinement = meshRefinement {
                // Export adaptively refined mesh
                let (vertices, faces) = refinement.buildRefinedMesh(from: meshAnchors)
                exportService.exportRefinedMeshAsOBJ(vertices: vertices, faces: faces, to: meshURL)
                let stats = refinement.statistics
                print("[CaptureVM] Refined mesh: \(stats.totalVertices) verts, \(stats.totalFaces) faces, \(stats.refinedRegions) refined regions")
            } else {
                exportService.exportMeshAsOBJ(anchors: meshAnchors, to: meshURL)
            }
        }
        
        // Clean up refinement service
        meshRefinement = nil
        
        // Finish video
        exportService.finishVideoRecording { [weak self] in
            DispatchQueue.main.async {
                self?.session.finishSaving()
            }
        }
    }
    
    // MARK: - AR Frame Handling
    
    private func handleARFrame(_ frame: ARFrame) {
        // While waiting for the first mesh polygons, watch for their arrival and then begin capture.
        if isWaitingForMesh {
            let hasMeshAnchors = frame.anchors.contains { $0 is ARMeshAnchor }
            if hasMeshAnchors {
                beginCapture()
            }
            return
        }
        
        guard session.state == .recording else { return }
        
        let timestamp = frame.timestamp
        
        // FPS subsampling: skip frames that are too close together
        if lastCaptureTimestamp >= 0 {
            let elapsed = timestamp - lastCaptureTimestamp
            if elapsed < captureInterval * 0.9 { // 0.9 factor for tolerance
                return
            }
        }
        lastCaptureTimestamp = timestamp

        let frameIndex = nextFrameIndex
        let pose = frame.camera.transform
        let intrinsics = frame.camera.intrinsics
        let capturedImage = frame.capturedImage
        
        // Save intrinsics once
        if !intrinsicsSaved {
            intrinsicsSaved = true
            if let url = session.cameraMatrixURL {
                exportService.saveCameraMatrix(intrinsics, to: url)
            }
            
            // Start video writer with actual frame dimensions
            let width = CVPixelBufferGetWidth(capturedImage)
            let height = CVPixelBufferGetHeight(capturedImage)
            videoSize = (width, height)
            if let videoURL = session.videoURL {
                do {
                    try exportService.startVideoRecording(
                        to: videoURL,
                        width: width,
                        height: height,
                        encodingMode: activeEncodingMode
                    )
                } catch {
                    print("[CaptureVM] Video start error: \(error)")
                }
            }
        }
        
        // Append video frame. If the encoder did not take it, drop the whole frame (pose, depth,
        // confidence) so video frame N, odometry row N and depth/N.png stay the same moment.
        guard exportService.appendVideoFrame(capturedImage, timestamp: timestamp) else {
            droppedFrames += 1
            return
        }
        nextFrameIndex += 1

        // Append odometry (pose + this frame's intrinsics)
        exportService.appendOdometry(timestamp: timestamp, frame: frameIndex, pose: pose, intrinsics: intrinsics)
        
        // Record mesh observations for adaptive refinement
        if let refinement = meshRefinement {
            let anchors = frame.anchors.compactMap { $0 as? ARMeshAnchor }
            if !anchors.isEmpty {
                refinement.recordObservations(anchors: anchors, timestamp: timestamp)
            }
        }
        
        // Tracking state
        let trackingState: String = {
            switch frame.camera.trackingState {
            case .normal: return "normal"
            case .notAvailable: return "notAvailable"
            case .limited(let reason):
                switch reason {
                case .initializing: return "initializing"
                case .excessiveMotion: return "excessiveMotion"
                case .insufficientFeatures: return "insufficientFeatures"
                case .relocalizing: return "relocalizing"
                @unknown default: return "limited"
                }
            }
        }()
        
        // What the camera actually used for this frame (auto modes change it frame to frame)
        let live = camera.cameraService.liveValues()
        let metadata = CaptureFrameMetadata(
            frameIndex: frameIndex,
            timestamp: timestamp,
            cameraPose: pose.flatArray,
            cameraIntrinsics: intrinsics.flatArray,
            exposureDuration: frame.camera.exposureDuration,
            iso: live?.iso ?? camera.settings.iso,
            lensPosition: live?.lensPosition ?? camera.settings.manualFocusPosition,
            trackingState: trackingState
        )
        exportService.appendFrameLog(
            metadata,
            exposureOffset: frame.camera.exposureOffset,
            whiteBalanceGains: live?.whiteBalanceGains ?? (0, 0, 0)
        )

        DispatchQueue.main.async {
            self.session.addFrame(metadata: metadata)
        }
        
        // Export depth and confidence asynchronously (only when LiDAR is enabled)
        if AppSettings.shared.lidarEnabled {
            exportQueue.async { [weak self] in
                guard let self = self else { return }
                
                // Save depth map (16-bit PNG in mm)
                if let depthMap = frame.sceneDepth?.depthMap,
                   let depthCopy = DepthMapProcessor.copyDepthMap(depthMap),
                   let url = self.session.depthURL(for: frameIndex) {
                    DepthMapProcessor.filterByDistance(depthCopy, maxDistance: self.settings.lidar.maxDistance)
                    self.exportService.saveDepthMap16BitPNG(depthCopy, to: url) { result in
                        if case .failure(let error) = result {
                            print("[CaptureVM] Depth export error: \(error)")
                        }
                    }
                }
                
                // Save confidence map
                if let confMap = frame.sceneDepth?.confidenceMap,
                   let confCopy = DepthMapProcessor.copyDepthMap(confMap),
                   let url = self.session.confidenceURL(for: frameIndex) {
                    self.exportService.saveConfidenceMap(confCopy, to: url) { result in
                        if case .failure(let error) = result {
                            print("[CaptureVM] Confidence export error: \(error)")
                        }
                    }
                }
            }
        }
    }
    
    // MARK: - Take Info (rescan.json)

    /// Writes rescan.json: what this take holds and how to read it, plus the floor plane ARKit found
    /// (motion capture mode detects horizontal planes). Older readers ignore the file.
    private func writeTakeInfo(anchors: [ARAnchor]) {
        guard let dir = session.sessionDirectory else { return }
        let settings = AppSettings.shared
        let format = lidar.arService.activeVideoFormat
        let sensorFPS = format?.framesPerSecond ?? 30
        let savedFPS = settings.motionCaptureMode ? sensorFPS : min(settings.captureFPS.rawValue, sensorFPS)

        var machine = utsname()
        uname(&machine)
        let model = withUnsafePointer(to: &machine.machine) {
            $0.withMemoryRebound(to: CChar.self, capacity: 1) { String(cString: $0) }
        }

        var info: [String: Any] = [
            "format": "stray_scanner",
            "format_version": 1,
            "app": "ReScan",
            "app_version": Bundle.main.object(forInfoDictionaryKey: "CFBundleShortVersionString") as? String ?? "",
            "device": model,
            "os_version": UIDevice.current.systemVersion,
            "created": ISO8601DateFormatter().string(from: session.startTime ?? Date()),
            "motion_capture_mode": settings.motionCaptureMode,
            "frames": nextFrameIndex,
            "dropped_frames": droppedFrames,
            "fps": savedFPS,
            "sensor_fps": sensorFPS,
            "odometry": [
                "camera_convention": settings.odometryConvention.key,
                "camera_axes": settings.odometryConvention == .opencv
                    ? "x right, y down, z forward" : "x right, y up, z back",
                "world": "ARKit, gravity aligned: y up, metres",
                "pose": "camera to world"
            ] as [String: Any],
            "confidence_levels": [0, 1, 2],
        ]
        if let size = videoSize {
            info["video"] = [
                "file": session.videoURL?.lastPathComponent ?? "",
                "width": size.width,
                "height": size.height,
                "encoding": activeEncodingMode.label
            ] as [String: Any]
        }
        if settings.lidarEnabled {
            info["depth"] = ["units": "millimetres", "format": "16-bit PNG",
                             "max_distance_m": Double(self.settings.lidar.maxDistance)] as [String: Any]
        }
        if imuRecording {
            info["imu"] = ["file": "imu.csv", "accelerometer": "m/s^2, includes gravity", "gyroscope": "rad/s",
                           "clock": "same as odometry timestamps"] as [String: Any]
        }
        if let lock = motionLock {
            var camera: [String: Any] = ["shutter_s": CMTimeGetSeconds(lock.shutter), "iso": lock.iso,
                                         "iso_at_maximum": lock.isoClipped]
            if let lens = lock.lensPosition { camera["lens_position"] = lens }
            if let wb = lock.whiteBalance {
                camera["white_balance_k"] = wb.temperature
                camera["white_balance_tint"] = wb.tint
            }
            info["camera_lock"] = camera
        }
        if let floor = Self.floorInfo(from: anchors) {
            info["floor"] = floor
        }

        let url = dir.appendingPathComponent("rescan.json")
        if let data = try? JSONSerialization.data(withJSONObject: info, options: [.prettyPrinted, .sortedKeys]) {
            try? data.write(to: url)
        }
        if droppedFrames > 0 {
            print("[CaptureVM] \(droppedFrames) frames dropped (encoder busy); their depth and poses were skipped too")
        }
    }

    /// The floor in ARKit's world (y up): a plane ARKit classified as floor (the largest), else the
    /// lowest horizontal plane bigger than 0.25 m², else the largest horizontal plane.
    private static func floorInfo(from anchors: [ARAnchor]) -> [String: Any]? {
        let planes = anchors.compactMap { $0 as? ARPlaneAnchor }.filter { $0.alignment == .horizontal }
        guard !planes.isEmpty else { return nil }
        func area(_ p: ARPlaneAnchor) -> Float { p.planeExtent.width * p.planeExtent.height }
        func isFloor(_ p: ARPlaneAnchor) -> Bool {
            if case .floor = p.classification { return true }
            return false
        }
        let largest = { (ps: [ARPlaneAnchor]) in ps.max(by: { area($0) < area($1) }) }
        guard let pick = largest(planes.filter(isFloor))
                ?? planes.filter({ area($0) > 0.25 }).min(by: { $0.transform.columns.3.y < $1.transform.columns.3.y })
                ?? largest(planes)
        else { return nil }
        let c = pick.transform * SIMD4<Float>(pick.center, 1)
        let n = simd_normalize(SIMD3<Float>(pick.transform.columns.1.x, pick.transform.columns.1.y, pick.transform.columns.1.z))
        return [
            "height_m": c.y,
            "center": [c.x, c.y, c.z],
            "normal": [n.x, n.y, n.z],
            "extent_m": [pick.planeExtent.width, pick.planeExtent.height],
            "classified_as_floor": isFloor(pick),
            "horizontal_planes": planes.count,
            "frame": "ARKit world (same as odometry positions)"
        ]
    }

    // MARK: - Thermal
    
    func checkThermalState() {
        switch thermalState {
        case .critical:
            if isRecording {
                stopRecording()
                showError(message: "🛑 Critical temperature. Recording stopped.")
            }
        default:
            break
        }
    }
    
    // MARK: - Error
    
    private func showError(message: String) {
        DispatchQueue.main.async {
            self.errorMessage = message
            self.showError = true
        }
    }
}
