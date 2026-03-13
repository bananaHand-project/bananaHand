# banana_hand_tracking

Vision teleop package for webcam + RealSense hand tracking.

Purpose
- Capture frames from a laptop webcam.
- Capture frames from an Intel RealSense color stream.
- Run MediaPipe Hands on either one camera or both cameras at once.
- Publish the averaged teleop joint trajectory when both cameras are active.

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

Run (RealSense publisher)
```
ros2 launch banana_hand_tracking realsense.launch.py
```

Run (hand tracking node)
```
ros2 launch banana_hand_tracking hand_tracking.launch.py
```

Run (combined vision teleop, webcam only by default)
```
ros2 launch banana_hand_tracking vision_teleop.launch.py
```

Run (combined vision teleop with webcam + RealSense)
```
ros2 launch banana_hand_tracking vision_teleop.launch.py dual_cam:=true
```

Run (combined vision teleop with webcam + RealSense + preview windows)
```
ros2 launch banana_hand_tracking vision_teleop.launch.py dual_cam:=true show_preview:=true
```

Topics + service
- `/camera/image_raw` (`sensor_msgs/msg/Image`) from `webcam.launch.py`
- `/camera/webcam/image_raw` (`sensor_msgs/msg/Image`) from `vision_teleop.launch.py dual_cam:=true`
- `/camera/realsense/image_raw` (`sensor_msgs/msg/Image`) from `realsense.launch.py` or `vision_teleop.launch.py dual_cam:=true`
- `/hand/landmarks` (`banana_interfaces/msg/HandState`) compatibility stream for the first configured camera
- `/hand/landmarks/webcam` (`banana_interfaces/msg/HandState`) in dual-camera mode
- `/hand/landmarks/realsense` (`banana_interfaces/msg/HandState`) in dual-camera mode
- `/hand/teleop_joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`)
- `/hand/calibrate` (`std_srvs/srv/Trigger`)

Non-ROS MVP (MediaPipe)
```
python3 /home/dbhaumik/BananaHand/software/ros/src/banana_hand_tracking/scripts/mediapipe_preview.py
```

Parameters (launch)
- `camera_index` (int, default: 0)
- `dual_cam` (bool, default: `false`; enable webcam + RealSense averaging)
- `realsense_serial` (string, default: `""`; empty picks the first detected device)
- `output_topic` (string, default: `/camera/image_raw` for webcam)
- `frame_id` (string, default: "camera")
- `publish_hz` (float, default: 30.0)
- `show_preview` (bool, default: true) — OpenCV live window
- `use_v4l2` (bool, default: true) — force V4L2 backend on Linux

Parameters (hand_tracking_node)
- `alpha` (float, default: 0.2) — teleop smoothing
- `mirror_handedness` (bool, default: true)
- `process_width` (int, default: 640)
- `process_height` (int, default: 480)
- `preview_fps` (float, default: 15.0)
- `process_fps` (float, default: 0.0; set 0.0 for no cap)
- `input_topics` (string array; use this for multi-camera mode)
- `camera_names` (string array; used for preview window and landmark topic naming)
- `frame_ids` (string array; one per camera)
- `landmark_topics` (string array; one per camera)
- `camera_stale_timeout` (float, default: 0.25s; ignores a camera if its latest result is stale)
- `calibration_seconds` (float, default: 5.0)

Quick checks
```
ros2 node info /webcam_node
ros2 topic list
ros2 topic hz /camera/image_raw
```

Dual-camera quick checks
```
ros2 node info /realsense_node
ros2 topic hz /camera/webcam/image_raw
ros2 topic hz /camera/realsense/image_raw
ros2 topic hz /hand/landmarks/webcam
ros2 topic hz /hand/landmarks/realsense
```

Common quick checks
```
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
- In dual-camera mode, the teleop output averages the currently active camera measurements. If one camera temporarily stops seeing the hand, the other camera continues driving the output until both are lost.
- Calibration now expects all configured cameras to see the same hand during each stage.
- `vision_teleop.launch.py` stays webcam-only unless `dual_cam:=true` is passed.

Troubleshooting
- If the topic exists but `ros2 topic hz` hangs, the camera may be busy or using a bad index.
- Try `camera_index:=1` or close other camera apps.
- If the RealSense node cannot start, verify `pyrealsense2` is installed and that the color stream works outside ROS.
