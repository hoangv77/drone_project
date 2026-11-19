# Phase 8 vision and recording

## Audit and architecture

Phase 8 starts at GitHub `main` commit `dfc9b08`, which includes the Phase 6 simulation and Phase 7 CM5 deployment. The PX4 interface publishes `/drone/state`, the safety monitor publishes `/drone/safety_status`, and both mission managers publish `/drone/mission_state`. The onboard launch starts only the read-only PX4 interface, safety monitor and passive mission manager. Phase 6 uses Gazebo and a native SITL observer. It has no calibrated camera or recorder. No physical camera model, interface, mounting geometry or tag target has been specified.

A camera driver, ROS bag replay, or the synthetic source publishes `Image` and `CameraInfo`. `camera_gate` checks exact timestamps, image age, optical frame and calibration, rectifies valid monocular images with OpenCV, and passes them to `apriltag_ros`. A separate recorder subscribes to selected topics and optionally a JPEG stream. Vision and recording have separate entry points. Neither participates in safety permissions or mission state, and neither exposes flight controls. [CM5 deployment](../../deployment/cm5/README.md)

## Installation and startup

On a configured Humble machine, build the vision package on its own:

```bash
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1
rosdep install --from-paths drone_project/vision --ignore-src --rosdistro humble -y
colcon build --base-paths drone_project/vision --packages-select drone_vision
source install/local_setup.bash
ros2 launch drone_vision vision_test.launch.py
```

Use Ubuntu's NumPy 1.x and OpenCV with Humble's packaged cv_bridge. Set `PYTHONNOUSERSITE=1` when user-installed Python packages shadow those libraries. Tested dependencies on Jammy x86_64 include apriltag_ros 3.4.0, apriltag_msgs 2.0.2, OpenCV 4.5.4, cv_bridge 3.2.1, SQLite rosbag2 0.15.17 and FFmpeg 4.4.2. CM5 carrier, camera driver, thermal capacity and encoder support require hardware validation.

To add optional components to a Phase 7 release, use the same verified firmware pin and the Phase 7 activation procedure:

```bash
DRONE_WITH_VISION=1 bash deployment/cm5/install.sh "$HOME/ros2-apt-source.deb"
DRONE_WITH_VISION=1 bash deployment/cm5/build.sh "$PX4_SOURCE" "$PX4_COMMIT"
bash deployment/cm5/update.sh --activate "$RELEASE"
RUN=/opt/drone/state/current/source/deployment/cm5/run.sh
bash "$RUN" vision camera_config:=/etc/drone/camera.yaml tag_config:=/etc/drone/apriltag.yaml
```

Copy the supplied YAML files into `/etc/drone` and replace placeholder tag size with measured dimensions. An update preserves the currently enabled vision build option unless explicitly overridden. Stop optional jobs before switching releases. To run vision in a separate transient systemd service, use the same arguments with `sudo systemd-run --unit=drone-vision --property=User="$(id -un)" /bin/bash "$RUN" vision ...`. View logs with `sudo journalctl -u drone-vision -f` and stop with `sudo systemctl stop drone-vision`. Grant that account access only to the actual camera device. The default onboard boot services are unchanged. Recording starts only through an explicit recorder invocation.

## Camera and calibration

- Configure the actual camera driver to publish `/camera/image_raw` and `/camera/camera_info`, or set both topic names in `camera.yaml`. Set resolution, frame rate and camera device in the driver configuration. The optional `driver_launch:=/absolute/path/driver.launch.py` includes a driver launcher. An external driver can run independently.
- For calibration install `ros-humble-camera-calibration`. Run the following command while publishing full-resolution raw frames. Substitute the measured checkerboard inner-corner counts and square edge length in meters. Collect diverse views spanning the image at the deployed focus and resolution. Save the standard ROS camera calibration YAML. Load it through the driver’s camera-info URL or its documented calibration setting. Verify nonzero intrinsics, matching width/height, optical frame and per-frame timestamp in `CameraInfo`. Recalibrate if the deployed lens or camera mode changes. [ROS camera calibration package](https://github.com/ros-perception/image_pipeline/tree/humble/camera_calibration)
- The gate accepts full-frame monocular pinhole, rational and equidistant distortion models. Missing, mismatched, stale or invalid input remains visible on the raw image topic and diagnostics. It does not reach the metric detector. `/vision/image_rect` and `/vision/camera_info` carry corrected pixels and zero distortion with the source header.
- For replay, record the raw image and corresponding camera info explicitly in a development bag, then use `ros2 bag play BAG --clock` and `vision.launch.py use_sim_time:=true`. Keep matching input topic names. Freshness uses ROS time and disconnect diagnostics use monotonic receipt time.

```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
  --ros-args -r image:=/camera/image_raw -r camera:=/camera
ros2 topic echo /camera/camera_info --once --qos-reliability best_effort
ros2 topic echo /diagnostics
ros2 topic echo /vision/detections
```

## Tags and frames

`/vision/detections` uses `apriltag_msgs/msg/AprilTagDetectionArray`. It carries the source header, family, IDs, corners, hamming distance and decision margin. Tag poses are optical-to-tag transforms on `/tf` with the acquisition stamp. Configure the family, measured black-border edge length in meters, and optional ID-specific frames/sizes in `apriltag.yaml`. No landing ID is selected. [AprilTag ROS detector contract](https://github.com/christianrauch/apriltag_ros/blob/master/README.md)

Supply `mount:=/etc/drone/transforms.yaml` only after measuring the rigid translation and quaternion from `base_link` to `camera_link`. This publishes the measured transform and the fixed ROS body-to-optical convention. With a driver that already provides both transforms, leave `mount` empty. Use one TF authority per child frame. Body axes are x forward, y left, z up. Optical axes are z forward, x right, y down. An unavailable body transform is diagnosed separately from optical detection validity. [ROS REP 103](https://www.ros.org/reps/rep-0103.html)

Use an RViz Image display on `/vision/image_rect` and a TF display with the optical frame as the fixed frame. For body-relative inspection set the fixed frame to `base_link` after confirming measured mounting TF. The original image stream is preserved. The test launch supplies synthetic intrinsics and renders AprilTag family 36h11 IDs 0 and 1, or the `one_tag`, `blank`, `missing_info`, `wrong_resolution` and `stale` scenarios. It does not publish measured hardware geometry.

### Phase 9 handoff

Consume each new detection array once, join its family/ID to the configured TF child frame, and look up the transform at its acquisition stamp. Reject zero, stale, future or out-of-order stamps, missing exact-time transforms, unconfigured IDs or sizes, unavailable calibration, invalid rotations/ranges and detections below independently chosen quality thresholds. Keep optical and body frame semantics explicit and treat empty arrays, expired streams and TF loss as invalid tracking input. Any TF from an earlier observation cannot substitute for a current detection. Phase 9 owns tag selection, filtering, tracking, timeouts and validity semantics. Validate calibration and range/pose errors against ground truth before using results for physical control.

## Recording and retrieval

Copy `config/recording.yaml`, choose a writable data directory and review the explicit topic list. The example records advertised entries only and logs which configured topics are unavailable. Raw camera images are excluded. `bag` and `video` can be enabled independently. Configure split size, maximum session size, aggregate limit and free-space reserve in MiB. A single recorder owns a storage root. Low space stops recording and preserves completed data. The polling limit can be exceeded by in-flight writes and container finalization, so allocate a reserve and a dedicated volume or filesystem quota for a strict storage boundary. No files are deleted automatically.

```bash
ros2 run drone_vision recorder --config "$CONFIG" start --git-commit "$(git rev-parse HEAD)"
# On CM5 use bash "$RUN" recorder --config "$CONFIG" start --git-commit "$(cat /opt/drone/state/current/commit)"
ros2 run drone_vision recorder --config "$CONFIG" status
ros2 run drone_vision recorder --config "$CONFIG" stop
ros2 bag info "$SESSION/rosbag"
ros2 bag play "$SESSION/rosbag"
```

For compressed video enable `video: true` and publish JPEG via the selected driver or an image_transport compressed publisher on the configured topic. The software H.264 path uses `libx264`. Select `h264_v4l2m2m` only after verifying an actual compatible encoder, device access, throughput and correct output on the target system. Set width/height, FPS and bitrate in YAML. The recorder uses bounded input buffering and writes segmented Matroska, plus a sidecar with source ROS and monotonic receipt timestamps for frames submitted to the encoder. Frames can be dropped under load and video presentation timestamps follow configured FPS. Use the sidecar for timing correlation. A disconnected camera or encoder failure stops video and is recorded independently of a working bag.

On graceful stop, the recorder flushes SQLite and closes FFmpeg input. After abrupt interruption, status identifies an abandoned running session as interrupted. Preserve the original and work on a copy. If missing bag metadata, try `ros2 bag reindex "$SESSION_COPY/rosbag" -s sqlite3`, inspect the SQLite database, and verify with `ros2 bag info` and replay. Use `ffprobe` to check video segments. The open segment may be incomplete. [rosbag2 Humble tools](https://github.com/ros2/rosbag2/tree/humble)

PX4 ULogs are separate from ROS and XRCE-DDS. Retrieve original `.ulg` files through QGroundControl log download over a supported MAVLink link, or from the controller's supported storage interface with the controller safely stopped. Check UTC, vehicle identity and run timing before associating a file. Copy to the CM5 and attach it to a stopped session. Missing files produce an error and duplicate names are rejected. [PX4 logging](https://docs.px4.io/main/en/dev_log/logging)

```bash
ros2 run drone_vision recorder --config "$CONFIG" attach-ulog "$SESSION_ID" ./flight.ulg
ros2 run drone_vision recorder --config "$CONFIG" manifest "$SESSION_ID" > "$SESSION_ID.sha256"
ssh OPERATOR@CM5_ADDRESS 'df -h /path/to/recordings; ls -1 /path/to/recordings'
rsync -av --partial OPERATOR@CM5_ADDRESS:/path/to/recordings/"$SESSION_ID"/ "$SESSION_ID"/
scp OPERATOR@CM5_ADDRESS:"$SESSION_ID.sha256" .
(cd "$SESSION_ID" && sha256sum -c "../$SESSION_ID.sha256")
```

Use the Jammy container's reachable SSH service or copy out from the CM5 host with `lxc file pull -r` before rsync. Metadata includes UTC start and end, commit, ROS distribution, host details, camera configuration, selected topics, software versions and errors. Copy stopped sessions so files remain stable throughout hashing and transfer.

## Testing

[Executed tests and environment](TEST_RESULTS.md)

```bash
PYTHONPATH=drone_project/vision python3 -m unittest discover -s drone_project/vision/test -v
# After building and sourcing the Humble vision overlay, use an isolated test machine/domain:
PYTHONNOUSERSITE=1 PHASE8_TEST_ROS=1 python3 -m unittest discover -s drone_project/vision/test -v
```

Synthetic AprilTags check detection IDs and optical timestamps. They do not establish metric pose accuracy. Validate the physical camera, exact calibration, mounting and TF, serial/USB coexistence, compressed-video performance and disk margins on ARM64 before onboard use.
