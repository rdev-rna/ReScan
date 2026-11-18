// CameraService.swift
// ReScan
//
// Manual camera controls (exposure, focus, WB) applied to the ARKit-owned camera device.

import Foundation
import AVFoundation
import UIKit

@Observable
final class CameraService: NSObject {
    
    // MARK: - Device
    
    private(set) var currentDevice: AVCaptureDevice?
    
    // State
    var currentISO: Float = 100
    var currentShutterSpeed: CMTime = CMTimeMake(value: 1, timescale: 60)
    var currentEV: Float = 0
    var currentFocusPosition: Float = 0.5
    
    // Ranges
    var minISO: Float = 32
    var maxISO: Float = 3200
    var minShutterSpeed: CMTime = CMTimeMake(value: 1, timescale: 8000)
    var maxShutterSpeed: CMTime = CMTimeMake(value: 1, timescale: 3)
    var minEV: Float = -8.0
    var maxEV: Float = 8.0
    
    private let controlQueue = DispatchQueue(label: "com.rescan.camera.control")
    
    // MARK: - Setup (uses ARKit's device)
    
    func attachToDevice(_ device: AVCaptureDevice?) {
        self.currentDevice = device
        if let device = device {
            updateDeviceRanges(from: device)
        }
    }
    
    private func updateDeviceRanges(from device: AVCaptureDevice) {
        let format = device.activeFormat
        DispatchQueue.main.async {
            self.minISO = format.minISO
            self.maxISO = format.maxISO
            self.minShutterSpeed = format.minExposureDuration
            self.maxShutterSpeed = format.maxExposureDuration
            self.minEV = device.minExposureTargetBias
            self.maxEV = device.maxExposureTargetBias
        }
    }
    
    // MARK: - Exposure
    
    func setManualExposure(shutterSpeed: CMTime, iso: Float) {
        guard let device = currentDevice else { return }
        controlQueue.async {
            do {
                try device.lockForConfiguration()
                let clampedISO = max(device.activeFormat.minISO, min(iso, device.activeFormat.maxISO))
                let clampedDuration = self.clampShutterSpeed(shutterSpeed, device: device)
                device.setExposureModeCustom(duration: clampedDuration, iso: clampedISO, completionHandler: nil)
                device.unlockForConfiguration()
                DispatchQueue.main.async {
                    self.currentISO = clampedISO
                    self.currentShutterSpeed = clampedDuration
                }
            } catch {
                print("[CameraService] Exposure error: \(error)")
            }
        }
    }
    
    func setExposureMode(_ mode: ExposureMode) {
        guard let device = currentDevice, device.isExposureModeSupported(mode.avMode) else { return }
        controlQueue.async {
            do {
                try device.lockForConfiguration()
                device.exposureMode = mode.avMode
                device.unlockForConfiguration()
            } catch {
                print("[CameraService] Exposure mode error: \(error)")
            }
        }
    }
    
    func setExposureCompensation(_ ev: Float) {
        guard let device = currentDevice else { return }
        controlQueue.async {
            do {
                try device.lockForConfiguration()
                let clamped = max(device.minExposureTargetBias, min(ev, device.maxExposureTargetBias))
                device.setExposureTargetBias(clamped, completionHandler: nil)
                device.unlockForConfiguration()
                DispatchQueue.main.async { self.currentEV = clamped }
            } catch {
                print("[CameraService] EV error: \(error)")
            }
        }
    }
    
    private func clampShutterSpeed(_ speed: CMTime, device: AVCaptureDevice) -> CMTime {
        let minDuration = device.activeFormat.minExposureDuration
        let maxDuration = device.activeFormat.maxExposureDuration
        let seconds = CMTimeGetSeconds(speed)
        let clamped = max(CMTimeGetSeconds(minDuration), min(seconds, CMTimeGetSeconds(maxDuration)))
        return CMTimeMakeWithSeconds(clamped, preferredTimescale: 1000000)
    }
    
    // MARK: - Motion Capture Lock

    struct MotionLock {
        let shutter: CMTime
        let iso: Float
        /// True when the ISO needed to keep the brightness was above the camera's maximum.
        let isoClipped: Bool
        let lensPosition: Float?
        let whiteBalance: AVCaptureDevice.WhiteBalanceTemperatureAndTintValues?
    }

    /// Fixes the shutter at `shutter`, picking the ISO that keeps the picture as bright as it is now,
    /// and locks focus and white balance where they are. `completion` runs on the main thread.
    func lockForMotion(shutter: CMTime, completion: @escaping (MotionLock) -> Void) {
        guard let device = currentDevice else { return }
        controlQueue.async {
            do {
                try device.lockForConfiguration()
                defer { device.unlockForConfiguration() }
                let format = device.activeFormat
                let duration = self.clampShutterSpeed(shutter, device: device)
                // Brightness ~ exposure time x ISO: keep the product the camera has metered
                let metered = CMTimeGetSeconds(device.exposureDuration) * Double(device.iso)
                let wanted = Float(metered / max(CMTimeGetSeconds(duration), 1e-6))
                let iso = max(format.minISO, min(wanted, format.maxISO))
                device.setExposureModeCustom(duration: duration, iso: iso, completionHandler: nil)

                var lens: Float?
                if device.isLockingFocusWithCustomLensPositionSupported {
                    lens = device.lensPosition
                    device.setFocusModeLocked(lensPosition: device.lensPosition, completionHandler: nil)
                } else if device.isFocusModeSupported(.locked) {
                    device.focusMode = .locked
                }

                var wb: AVCaptureDevice.WhiteBalanceTemperatureAndTintValues?
                if device.isWhiteBalanceModeSupported(.locked) {
                    let gains = device.deviceWhiteBalanceGains
                    wb = device.temperatureAndTintValues(for: gains)
                    device.setWhiteBalanceModeLocked(with: gains, completionHandler: nil)
                }

                let lock = MotionLock(shutter: duration, iso: iso, isoClipped: wanted > format.maxISO * 1.05,
                                      lensPosition: lens, whiteBalance: wb)
                DispatchQueue.main.async {
                    self.currentISO = iso
                    self.currentShutterSpeed = duration
                    if let lens { self.currentFocusPosition = lens }
                    completion(lock)
                }
            } catch {
                print("[CameraService] Motion lock error: \(error)")
            }
        }
    }

    // MARK: - Live Values (per frame)

    struct LiveValues {
        let iso: Float
        let lensPosition: Float
        let whiteBalanceGains: (Float, Float, Float)
    }

    /// What the camera is actually using right now (not what the controls were set to).
    func liveValues() -> LiveValues? {
        guard let device = currentDevice else { return nil }
        let g = device.deviceWhiteBalanceGains
        return LiveValues(iso: device.iso, lensPosition: device.lensPosition,
                          whiteBalanceGains: (g.redGain, g.greenGain, g.blueGain))
    }

    // MARK: - Focus

    func setFocusMode(_ mode: FocusMode, lensPosition: Float? = nil) {
        guard let device = currentDevice else { return }
        controlQueue.async {
            do {
                try device.lockForConfiguration()
                switch mode {
                case .manual:
                    let position = max(0.0, min(lensPosition ?? 0.5, 1.0))
                    device.setFocusModeLocked(lensPosition: position, completionHandler: nil)
                    DispatchQueue.main.async { self.currentFocusPosition = position }
                case .auto:
                    if device.isFocusModeSupported(.autoFocus) {
                        device.focusMode = .autoFocus
                    }
                case .autoContinuous:
                    if device.isFocusModeSupported(.continuousAutoFocus) {
                        device.focusMode = .continuousAutoFocus
                    }
                }
                device.unlockForConfiguration()
            } catch {
                print("[CameraService] Focus error: \(error)")
            }
        }
    }
    
    // MARK: - White Balance
    
    func setAutoWhiteBalance() {
        guard let device = currentDevice, device.isWhiteBalanceModeSupported(.continuousAutoWhiteBalance) else { return }
        controlQueue.async {
            do {
                try device.lockForConfiguration()
                device.whiteBalanceMode = .continuousAutoWhiteBalance
                device.unlockForConfiguration()
            } catch {
                print("[CameraService] WB error: \(error)")
            }
        }
    }
    
    func setManualWhiteBalance(temperature: Float, tint: Float) {
        guard let device = currentDevice else { return }
        controlQueue.async {
            do {
                try device.lockForConfiguration()
                let tempTint = AVCaptureDevice.WhiteBalanceTemperatureAndTintValues(temperature: temperature, tint: tint)
                let gains = device.deviceWhiteBalanceGains(for: tempTint)
                let maxGain = device.maxWhiteBalanceGain
                let clamped = AVCaptureDevice.WhiteBalanceGains(
                    redGain: max(1.0, min(gains.redGain, maxGain)),
                    greenGain: max(1.0, min(gains.greenGain, maxGain)),
                    blueGain: max(1.0, min(gains.blueGain, maxGain))
                )
                device.setWhiteBalanceModeLocked(with: clamped, completionHandler: nil)
                device.unlockForConfiguration()
            } catch {
                print("[CameraService] WB manual error: \(error)")
            }
        }
    }
}
