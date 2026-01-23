# hand_tracking

Webcam hand tracking package (Phase A: webcam capture + image publishing).

Purpose
- Capture frames from a laptop webcam.
- Publish frames as `sensor_msgs/Image` on `/camera/image_raw`.
- (Later) Run hand detection/landmarks (planned: MediaPipe or similar).

Build
```
cd /home/dbhaumik/BananaHand/vision_teleop/teleop_ws
colcon build --symlink-install --packages-select hand_tracking
source install/setup.bash
```

Run
```
ros2 launch hand_tracking webcam.launch.py
```

Parameters (launch)
- `camera_index` (int, default: 0)
- `frame_id` (string, default: "camera")
- `publish_hz` (float, default: 30.0)
- `show_preview` (bool, default: true) — OpenCV live window
- `use_v4l2` (bool, default: true) — force V4L2 backend on Linux

Quick checks
```
ros2 node info /webcam_node
ros2 topic list
ros2 topic hz /camera/image_raw
```

Troubleshooting
- If the topic exists but `ros2 topic hz` hangs, the camera may be busy or using a bad index.
- Try `camera_index:=1` or close other camera apps.
