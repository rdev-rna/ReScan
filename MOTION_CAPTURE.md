# Motion capture build (1.2.1)

Changes on the `mocap-capture` branch for markerless motion capture (SAM 3D Body / BlendCap, GEM-X).

## Settings → Motion Capture

| Setting | Default | What it does |
|---|---|---|
| Motion Capture Mode | off | ARKit's largest 60 fps format, every frame kept, no scene mesh, horizontal plane detection. When recording starts, the shutter is fixed, the ISO is set so the brightness stays as metered, and focus and white balance lock. A warning shows if the ISO hits its maximum. |
| Shutter | 1/500 s | 1/250, 1/500, 1/1000 or 1/2000. |
| Pose Axes | OpenCV (Stray Scanner) | Camera axes in `odometry.csv`. ARKit is what ReScan 1.2 wrote. |
| Record IMU | on | `imu.csv` at 200 Hz. |

## Take format (now matches Stray Scanner)

- `odometry.csv`: `timestamp, frame, x, y, z, qx, qy, qz, qw, fx, fy, cx, cy, distortion_center_x, distortion_center_y`.
  Camera to world, ARKit world (y up against gravity, metres). With OpenCV axes the camera is x right, y down,
  z forward (Stray's `q_WA * q_AC`). Frame names are `%06d`. The distortion columns are empty (ARKit frames carry no
  lens calibration).
- `confidence/*.png`: 0, 1, 2 (was 0, 127, 255).
- Every file is in step: a frame the video encoder did not take is dropped from the poses, depth and confidence too, so
  video frame N, odometry row N and `depth/N.png` are the same moment. `rescan.json` counts the dropped frames.

## New files

- `frames.csv`: per frame, what the camera actually used: `exposure_duration` (s), `exposure_offset`, `iso`,
  `lens_position`, white balance gains, tracking state.
- `imu.csv`: Stray's layout: `timestamp, a_x, a_y, a_z` (m/s², gravity included), `alpha_x, alpha_y, alpha_z` (rad/s).
  Same clock as `odometry.csv`.
- `rescan.json`: app version, device, fps, video size, pose axes, the camera lock, dropped frames, and the floor plane
  (`height_m`, `center`, `normal`, `extent_m`, in the odometry's world) when ARKit found one.
