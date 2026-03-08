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

## Calibration + Quick Checks
```bash
ros2 service call /hand/calibrate std_srvs/srv/Trigger {}
ros2 topic hz /hand/teleop_joint_trajectory
ros2 topic echo /tx_positions
ros2 topic echo /rx_positions
ros2 topic echo /rx_force
```

## FSR Visualizer (Desktop GUI)
2D live visualizer for fingertip force values on `rx_force`.

Dependencies:
- ROS2 (`rclpy`, `std_msgs`)
- `PySide6`

Install GUI dependency if needed:
```bash
sudo apt install python3-pyside6
# or
pip install -r src/banana_hand_fsr_visualizer/requirements.txt
```

Build the package:
```bash
cd software/ros
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select banana_hand_fsr_visualizer
source install/setup.bash
```

Run visualizer:
```bash
ros2 run banana_hand_fsr_visualizer fsr_visualizer
```

Run with launch file:
```bash
ros2 launch banana_hand_fsr_visualizer fsr_visualizer.launch.py
```

No-FSR test publisher:
```bash
ros2 run banana_hand_fsr_visualizer fsr_test_publisher
```

Quick one-liner test publish:
```bash
ros2 topic pub /rx_force std_msgs/msg/UInt16MultiArray "{data: [100, 350, 620, 460, 220, 0, 0, 0, 0, 0]}" -r 10
```

Key visualizer parameters:
- `topic_name` (default `rx_force`)
- `finger_indices` (default `[0,1,2,3,4]`)
- `alpha` (default `0.25`)
- `global_min`, `global_max`
- `per_finger_min`, `per_finger_max` (optional length 5)
- `refresh_hz` (default `30.0`)
- `contact_threshold` (default `0.7`)
- `simulate_if_no_data` (default `false`)

Example overrides:
```bash
ros2 run banana_hand_fsr_visualizer fsr_visualizer --ros-args \
  -p topic_name:=rx_force \
  -p finger_indices:="[0,2,4,6,8]" \
  -p alpha:=0.35 \
  -p global_min:=0.0 \
  -p global_max:=1500.0
```

Adapting to a different message type:
- Edit `banana_hand_fsr_visualizer/fsr_visualizer_node.py`.
- Change `create_subscription(...)` message type.
- Update `_on_force_msg()` to output 5 values ordered as thumb/index/middle/ring/pinky.
