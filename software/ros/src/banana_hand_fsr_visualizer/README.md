# banana_hand_fsr_visualizer

BananaHand-branded ROS2 desktop dashboard for:

- `rx_force` force telemetry with 5 fingertip pads + 5 palm pads
- `rx_positions` live position telemetry with rolling plots for:
  - Thumb
  - Index
  - Middle
  - Ring
  - Pinky
  - Thumb opposition

The UI stays within the existing stack:

- Python
- ROS2 / `rclpy`
- PySide6

No web stack, OpenGL, or extra plotting framework is required.

## What changed

- Redesigned cream / black / banana-yellow dashboard styling
- New branded header with BananaHand pixel-mark
- Cleaner 2D hand schematic with designed sensor pads
- Live summary cards for max force, active contacts, average force, and most loaded finger
- Realtime scrolling position plot panel integrated into the same dashboard
- `rx_positions` subscription support
- Support for either:
  - `sensor_msgs/msg/JointState` on `rx_positions` (default, matches this repo)
  - `std_msgs/msg/UInt16MultiArray` via parameter override

## Dependencies

ROS dependencies are expected from your ROS2 installation:

- `rclpy`
- `std_msgs`
- `sensor_msgs`

Python dependency:

```bash
pip install PySide6
```

Ubuntu package alternative:

```bash
sudo apt install python3-pyside6
```

## Build

```bash
cd /home/lokesh/BananaHand/software/ros
colcon build --packages-select banana_hand_fsr_visualizer
source install/setup.bash
```

## Run

```bash
ros2 run banana_hand_fsr_visualizer fsr_visualizer
```

Launch file:

```bash
ros2 launch banana_hand_fsr_visualizer fsr_visualizer.launch.py
```

## Demo / fallback testing

Built-in simulation in the dashboard:

```bash
ros2 run banana_hand_fsr_visualizer fsr_visualizer --ros-args -p simulate_if_no_data:=true
```

Dedicated test publisher for both topics:

```bash
ros2 run banana_hand_fsr_visualizer fsr_test_publisher
```

## Important parameters

- `topic_name`
  - force topic, default `rx_force`
- `position_topic_name`
  - position topic, default `rx_positions`
- `position_message_type`
  - `joint_state` or `uint16_multi_array`
  - default `joint_state`
- `finger_indices`
  - default `[0,1,2,3,4]`
- `palm_indices`
  - default `[-5,-4,-3,-2,-1]`
- `position_indices`
  - default `[0,1,2,3,4,5]`
- `alpha`
  - force smoothing factor
- `position_alpha`
  - position smoothing factor
- `global_min`, `global_max`
  - force normalization range
- `position_min`, `position_max`
  - plot range baseline
- `plot_window_seconds`
  - default `8.0`
- `contact_threshold`
  - normalized threshold for contact highlighting
- `simulate_if_no_data`
  - enables built-in animated fallback data

## Example: use UInt16MultiArray for positions

```bash
ros2 run banana_hand_fsr_visualizer fsr_visualizer --ros-args \
  -p topic_name:=rx_force \
  -p position_topic_name:=rx_positions \
  -p position_message_type:=uint16_multi_array \
  -p plot_window_seconds:=6.0 \
  -p contact_threshold:=0.65
```
