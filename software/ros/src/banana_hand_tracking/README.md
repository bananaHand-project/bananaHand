# banana_hand_tracking

Webcam hand tracking package (Phase A: webcam capture + image publishing).

Purpose
- Capture frames from a laptop webcam.
- Publish frames as `sensor_msgs/Image` on `/camera/image_raw`.
- Run MediaPipe Hands to visualize landmarks (MVP).

Build
```
cd /home/dbhaumik/BananaHand/software/ros
pip install --user -r src/banana_hand_tracking/requirements.txt
colcon build --symlink-install --packages-select banana_interfaces banana_hand_tracking
source install/setup.bash
```

Run (webcam publisher)
```
ros2 launch banana_hand_tracking webcam.launch.py
```

Run (hand tracking node)
```
ros2 launch banana_hand_tracking hand_tracking.launch.py
```

Run (combined)
```
ros2 launch banana_hand_tracking vision_teleop.launch.py
```

Run (combined, higher-quality profile)
```
ros2 launch banana_hand_tracking vision_teleop.launch.py \
  show_preview:=false process_fps:=25.0 \
  process_width:=640 process_height:=480 \
  capture_width:=640 capture_height:=480 \
  capture_fps:=30.0 pixel_format:=MJPG
```

Topics + service
- `/camera/image_raw` (`sensor_msgs/msg/Image`)
- `/camera/frame_tick` (`std_msgs/msg/UInt32`) lightweight frame-rate probe
- `/hand/landmarks` (`banana_interfaces/msg/HandState`)
- `/hand/teleop_joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`)
- `/hand/calibrate` (`std_srvs/srv/Trigger`)

Non-ROS MVP (MediaPipe)
```
python3 /home/dbhaumik/BananaHand/software/ros/src/banana_hand_tracking/scripts/mediapipe_preview.py
```

Parameters (launch)
- `camera_index` (int, default: 0)
- `frame_id` (string, default: "camera")
- `publish_hz` (float, default: 30.0) — max publish rate
- `capture_fps` (float, default: 30.0) — requested webcam capture rate
- `capture_width` (int, default: 320 in `vision_teleop.launch.py`)
- `capture_height` (int, default: 240 in `vision_teleop.launch.py`)
- `pixel_format` (string, default: `MJPG`) — use `AUTO` to skip format request
- `repeat_last_frame` (bool, default: true) — keep publishing at `publish_hz` even if capture FPS is lower
- `publish_tick_topic` (bool, default: true) — publish `/camera/frame_tick` for lightweight rate checks
- `show_preview` (bool, default: true in `vision_teleop.launch.py`) — OpenCV live window
- `use_v4l2` (bool, default: true) — force V4L2 backend on Linux
- `log_rate_interval` (float, default: 5.0) — periodic capture/publish rate logs

Parameters (hand_tracking_node)
- `alpha` (float, default: 0.2) — teleop smoothing
- `mirror_handedness` (bool, default: true)
- `process_width` (int, default: 320 in `vision_teleop.launch.py`)
- `process_height` (int, default: 240 in `vision_teleop.launch.py`)
- `preview_fps` (float, default: 0.0 in `vision_teleop.launch.py`; uncapped)
- `process_fps` (float, default: 0.0 in `vision_teleop.launch.py`; uncapped)
- `calibration_seconds` (float, default: 2.0)

Quick checks
```
ros2 node info /webcam_node
ros2 topic list
ros2 topic hz /camera/image_raw
ros2 topic hz /camera/frame_tick
ros2 topic hz /hand/landmarks
ros2 topic hz /hand/teleop_joint_trajectory
```

Calibration
```
ros2 service call /hand/calibrate std_srvs/srv/Trigger {}
```
Notes:
- Until calibrated, teleop outputs are all zeros and a warning is logged.
- Landmarks are published only when a hand is detected.

Troubleshooting
- If the topic exists but `ros2 topic hz` hangs, the camera may be busy or using a bad index.
- Try `camera_index:=1` or close other camera apps.
- With `repeat_last_frame:=false`, `publish_hz` is only an upper bound; if capture runs at 10-15 Hz, publish will also be 10-15 Hz.
- With `repeat_last_frame:=true` (default), topic rate stays near `publish_hz`; use `stale_publish_ratio` in logs to see how often frames are repeated.
- Prefer `ros2 topic hz /camera/frame_tick` to verify source publish rate. `/camera/image_raw` can under-report in CLI due large message copy/transport load.
- Check `/webcam_node` logs for `Camera configured` and `Effective rates` to see actual capture vs publish.
- Default `vision_teleop.launch.py` values run with uncapped hand processing and preview refresh (`process_fps:=0.0`, `preview_fps:=0.0`).
- For better far-distance/precision tracking, use the higher-quality profile shown above (640x480 + higher `process_fps`).
