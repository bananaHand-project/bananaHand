# Banana Hand Tracking

`banana_hand_tracking` contains the webcam capture and MediaPipe-based tracking nodes that produce hand landmarks and teleop joint ratios.

## Install
```bash
cd /home/lokesh/BananaHand/software/ros
source /opt/ros/humble/setup.bash
pip install --user -r requirements.txt
colcon build --symlink-install --packages-select banana_interfaces banana_hand_tracking
source install/setup.bash
```

## Run
Combined webcam + tracking:
```bash
ros2 launch banana_hand_tracking vision_teleop.launch.py
```

Tracking only:
```bash
ros2 launch banana_hand_tracking hand_tracking.launch.py
```

Webcam only:
```bash
ros2 launch banana_hand_tracking webcam.launch.py
```

## Topics and Service
- `/camera/image_raw` (`sensor_msgs/msg/Image`)
- `/hand/landmarks` (`banana_interfaces/msg/HandState`)
- `/hand/teleop_joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`)
- `/hand/calibrate` (`std_srvs/srv/Trigger`)

## Launch Files
`webcam.launch.py`
- starts `webcam_node`
- fixed parameters in the launch file:
  - `camera_index=0`
  - `frame_id=camera`
  - `publish_hz=30.0`

`hand_tracking.launch.py`
- starts `hand_tracking_node`
- fixed parameters in the launch file:
  - `input_topic=/camera/image_raw`
  - `frame_id=camera`
  - `show_preview=true`

`vision_teleop.launch.py`
- starts both `webcam_node` and `hand_tracking_node`
- launch arguments:
  - `show_preview` default `true`

## Webcam Node Parameters
`webcam_node` supports:
- `camera_index`
  default `0`
- `frame_id`
  default `camera`
- `publish_hz`
  default `30.0`
- `show_preview`
  default `true`
- `use_v4l2`
  default `true`

Example:
```bash
ros2 run banana_hand_tracking webcam_node --ros-args \
  -p camera_index:=1 \
  -p show_preview:=false
```

## Hand Tracking Node Parameters
`hand_tracking_node` supports:
- `input_topic`
  default `/camera/image_raw`
- `frame_id`
  default `camera`
- `show_preview`
  default `true`
- `mirror_handedness`
  default `true`
- `alpha`
  default `0.2`
- `process_width`
  default `640`
- `process_height`
  default `480`
- `preview_fps`
  default `15.0`
- `process_fps`
  default `0.0`
  `0.0` means uncapped processing

Example:
```bash
ros2 run banana_hand_tracking hand_tracking_node --ros-args \
  -p input_topic:=/camera/image_raw \
  -p alpha:=0.15 \
  -p process_width:=640 \
  -p process_height:=480
```

## Calibration
```bash
ros2 service call /hand/calibrate std_srvs/srv/Trigger {}
```

## Quick Checks
```bash
ros2 node info /webcam_node
ros2 node info /hand_tracking_node
ros2 topic hz /camera/image_raw
ros2 topic hz /hand/landmarks
ros2 topic hz /hand/teleop_joint_trajectory
```

## Troubleshooting
- If `/camera/image_raw` exists but no frames arrive, try a different `camera_index`
- If another app is using the webcam, close it before launching tracking
- If `/dev/video0` does not exist on your machine, use a node run command and override `camera_index`
