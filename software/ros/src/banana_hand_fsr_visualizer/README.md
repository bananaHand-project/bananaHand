# banana_hand_fsr_visualizer

2D desktop visualizer for five-finger FSR data using ROS2 (`rclpy`) + PySide6.

## What it does
- Subscribes to `std_msgs/UInt16MultiArray` (default topic: `rx_force`).
- Maps 5 configured indices to Thumb/Index/Middle/Ring/Pinky.
- Applies exponential smoothing (`alpha`).
- Normalizes to `[0,1]` using global or per-finger min/max.
- Draws a demo-friendly 2D hand with fingertip colors + numeric values.
- Shows color legend and ROS topic/status line.

## Dependencies
- ROS2
- `rclpy`
- `std_msgs`
- `PySide6`

Install PySide6 if needed:
```bash
sudo apt install python3-pyside6
```

## Build and run
```bash
cd /home/lokesh/BananaHand/software/ros
colcon build --packages-select banana_hand_fsr_visualizer
source install/setup.bash
ros2 run banana_hand_fsr_visualizer fsr_visualizer
```

Or launch:
```bash
ros2 launch banana_hand_fsr_visualizer fsr_visualizer.launch.py
```

## No-hardware testing options

### Option A: Dedicated test publisher node
```bash
ros2 run banana_hand_fsr_visualizer fsr_test_publisher
```

### Option B: One-liner topic publisher
```bash
ros2 topic pub /rx_force std_msgs/msg/UInt16MultiArray "{data: [100, 350, 620, 460, 220, 0, 0, 0, 0, 0]}" -r 10
```

## Key parameters (visualizer)
- `topic_name` (string, default `rx_force`)
- `finger_indices` (int[5], default `[0,1,2,3,4]`)
- `alpha` (float in `[0,1]`, default `0.25`)
- `global_min`, `global_max` (floats)
- `per_finger_min`, `per_finger_max` (float[5], optional)
- `refresh_hz` (float)
- `contact_threshold` (normalized float, default `0.7`)
- `simulate_if_no_data` (bool, default `false`)

Example overriding index mapping and filtering:
```bash
ros2 run banana_hand_fsr_visualizer fsr_visualizer --ros-args \
  -p topic_name:=rx_force \
  -p finger_indices:="[0,2,4,6,8]" \
  -p alpha:=0.35 \
  -p global_min:=0.0 \
  -p global_max:=1500.0
```

## Adapting to a different message type
Edit `banana_hand_fsr_visualizer/fsr_visualizer_node.py`:
- Change subscriber message type in `create_subscription(...)`.
- Update `_on_force_msg()` so it extracts 5 numeric values in Thumb/Index/Middle/Ring/Pinky order.

Everything else (smoothing, normalization, color mapping, drawing) remains unchanged.
