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
colcon build --symlink-install --packages-select banana_hand_tracking
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

Non-ROS MVP (MediaPipe)
```
python3 /home/dbhaumik/BananaHand/software/ros/src/banana_hand_tracking/scripts/mediapipe_preview.py
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
