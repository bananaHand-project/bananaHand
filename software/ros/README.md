# BananaHand ROS Bringup

End-to-end launch now runs:
- webcam + hand tracking (`banana_hand_tracking`)
- hand mapping (`banana_hand_mapping`)  
  `/hand/teleop_joint_trajectory` -> `/tx_positions` (scaled to `0..4095`)
- serial bridge (`banana_serial_bridge`) to hardware

## Build
```bash
cd software/ros
source /opt/ros/humble/setup.bash
pip install --user -r src/banana_hand_tracking/requirements.txt
colcon build --symlink-install --packages-select \
  banana_interfaces banana_hand_tracking banana_hand_mapping \
  banana_serial_bridge banana_bringup
source install/setup.bash
```

## Run Bringup
```bash
ros2 launch banana_bringup bringup.launch.py \
  port:=/dev/ttyACM0 baud:=115200
```

## Useful Launch Args
```bash
# Disable OpenCV hand preview window (lower latency)
ros2 launch banana_bringup bringup.launch.py show_preview:=false

# Run serial bridge only (no vision/mapping)
ros2 launch banana_bringup bringup.launch.py \
  include_vision:=false include_mapping:=false
```

Args:
- `port` (default `/dev/ttyACM0`)
- `baud` (default `115200`)
- `show_preview` (default `true`)
- `include_vision` (default `true`)
- `include_mapping` (default `true`)

Vision defaults (`banana_hand_tracking/vision_teleop.launch.py`):
- `process_fps` defaults to `0.0` (uncapped)
- `preview_fps` defaults to `0.0` (uncapped)

## Calibration + Quick Checks
```bash
ros2 service call /hand/calibrate std_srvs/srv/Trigger {}
ros2 topic hz /hand/teleop_joint_trajectory
ros2 topic echo /tx_positions
ros2 topic echo /rx_positions
ros2 topic echo /rx_force
```
