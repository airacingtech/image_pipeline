# ART Center Stereo Calibration SOP

This SOP calibrates the following synchronized stereo pair:

- Left: `vimba_front_left_center` (`10.42.17.53`)
- Right: `vimba_front_right_center` (`10.42.17.51`)
- Target: 11 x 8 checkerboard squares, which are **10 x 7 OpenCV inner corners**
- Square size: `0.070 m`
- Capture: `2064 x 1544`, `10 Hz`, GigE Vision `Action0`

The vehicle publishes the two raw image streams. The `roar` workstation saves
synchronized pairs locally. Board detection and calibration run offline, so the
operator does not need to wait for live corner detection at every pose.

The legacy `cameracalibrator`, `cameracheck`, and monocular calibration workflow
remain available and are not modified by this SOP.

## 1. Build and source

Use the `art-jazzy` branch on both machines:

```bash
git switch art-jazzy
git pull --ff-only origin art-jazzy
source /opt/ros/jazzy/setup.bash
colcon build --packages-select camera_calibration
source install/setup.bash
```

On the vehicle, source the `race_common` installation before the
`image_pipeline` overlay so that `iac_launch` and `avt_vimba_camera` are
available:

```bash
source /opt/ros/jazzy/setup.bash
source <race_common_workspace>/install/setup.bash
source <image_pipeline_workspace>/install/setup.bash
```

## 2. Vehicle preflight

Stop the normal launch that owns either center camera. Do not run the production
camera nodes and the calibration camera nodes at the same time.

Start the dedicated full-resolution publishers on the vehicle:

```bash
ros2 launch camera_calibration art_stereo_capture.launch.py
```

This launch loads the existing center-camera parameter files, then applies a
calibration-only override. It does not edit the original vehicle files.

In a second vehicle terminal, verify both streams:

```bash
ros2 run camera_calibration art_stereo_status \
  --duration 10 \
  --json-out /tmp/art_stereo_preflight.json
```

Do not start collection unless the result is `PASS`. The required preflight
conditions are:

- both CameraInfo streams are active;
- both images are `2064 x 1544`;
- both rates are within `10 +/- 2 Hz`;
- paired timestamp delta p95 is no greater than `2 ms`.

A zero CameraInfo baseline is normal before calibration. Therefore, do not add
`--require-baseline` during the initial preflight.

## 3. Connect the roar workstation

Use the same `ROS_DOMAIN_ID` as the vehicle. When direct Cyclone DDS discovery
is required, review the installed example first. Its interface and peer address
must match the current workstation and vehicle:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$(ros2 pkg prefix camera_calibration)/share/camera_calibration/config/art_stereo_cyclonedds_roar_vehicle.xml
```

Confirm that the two raw topics are visible from `roar`:

```bash
ros2 topic list | grep vimba_calib
ros2 topic hz /vimba_calib_left/image
ros2 topic hz /vimba_calib_right/image
```

## 4. Capture 50-60 raw pairs locally

Create a new session name. The capture tool refuses to write into a non-empty
session directory:

```bash
SESSION=$HOME/stereo_calibration_data/$(date +%Y%m%d_%H%M%S)

ros2 run camera_calibration art_stereo_capture \
  --output "$SESSION" \
  --left-topic /vimba_calib_left/image \
  --right-topic /vimba_calib_right/image \
  --mode interval \
  --max-pairs 60 \
  --max-delta-ms 2.0 \
  --min-interval-sec 1.0
```

Use two people when possible:

1. The board operator moves and holds the checkerboard.
2. The computer operator watches the saved-pair count and checks for blur,
   clipping, and dropped streams.

Hold each pose for about `1.5-2 seconds`. Collect the following coverage:

1. Board centered: near, medium, and far.
2. Board near each of the four image corners.
3. Board near the top, bottom, left, and right edges.
4. Positive and negative yaw.
5. Positive and negative pitch.
6. Positive and negative roll.
7. Several combined angle and distance poses.

Keep the complete checkerboard visible in both cameras. Avoid repeated poses,
motion blur, strong glare, shadows across the board, or a board that occupies
only a small central area. The target is **50-60 raw pairs**, not 120.

After capture, verify the local dataset:

```bash
find "$SESSION/left" -name '*.png' | wc -l
find "$SESSION/right" -name '*.png' | wc -l
sed -n '1,200p' "$SESSION/capture_summary.json"
```

The left and right counts must match.

## 5. Run offline calibration

Measure the physical optical-center baseline if a reliable measurement is
available. Then run:

```bash
RESULT=${SESSION}_result

ros2 run camera_calibration art_stereo_calibrate \
  --input "$SESSION" \
  --output "$RESULT" \
  --expected-width 2064 \
  --expected-height 1544 \
  --expected-baseline-m <measured-baseline-in-metres>
```

Omit only the final `--expected-baseline-m` line when no reliable measurement
is available. The default board file already specifies 10 x 7 inner corners and
a 70 mm square size; no `--board` option is needed for the confirmed target.

The offline stage:

1. re-detects the target in every left/right pair;
2. rejects unreadable pairs and high-error views;
3. reserves a deterministic holdout subset;
4. fits both intrinsics and the left-to-right stereo transform;
5. rectifies representative pairs and writes an acceptance report.

## 6. Completion criteria

Calibration is complete only when `$RESULT/report.json` reports `PASS` and the
rectified preview images show corresponding target points on the same horizontal
rows.

Default required gates:

- at least 15 accepted stereo views;
- exact `2064 x 1544` image size;
- stereo RMS no greater than `1.0 px`;
- final vertical epipolar p95 no greater than `1.0 px`;
- holdout vertical epipolar p95 no greater than `1.5 px`;
- capture timestamp delta p95 no greater than `2 ms`;
- nonzero stereo baseline;
- nonzero right projection translation;
- calibrated baseline within 5 percent of the measured baseline, when supplied.

Review these artifacts:

```text
$RESULT/report.json
$RESULT/report.md
$RESULT/stereo_calibration.yaml
$RESULT/left_camera_info.yaml
$RESULT/right_camera_info.yaml
$RESULT/rectified_previews/
```

## 7. Deployment rule

Do not automatically overwrite production CameraInfo files. Before deployment:

1. confirm that the report is `PASS`;
2. inspect every rectified preview;
3. confirm the `2064 x 1544` image contract;
4. confirm the transform convention is left camera to right camera;
5. compare the calibrated and measured baselines;
6. back up the existing production calibration files;
7. deploy the generated CameraInfo files through the normal reviewed vehicle
   configuration process.

## Common failures

- `516 x 384` images: the calibration override was not applied; stop the old
  camera launch and restart `art_stereo_capture.launch.py`.
- Timestamp p95 above `2 ms`: check PTP state, Action0 triggering, network load,
  and whether both cameras are using the dedicated launch.
- Fewer than 15 accepted pairs: capture a new, sharper, more diverse session;
  do not relax the acceptance gates first.
- Corner detection fails: verify the whole 11 x 8-square board is visible,
  sharply focused, and not covered by glare.
- Output directory is not empty: choose a new `SESSION` or `RESULT`; the tools
  intentionally do not overwrite an earlier run.

The longer package documentation is available in
`doc/tutorial_art_center_stereo.rst`.
