# BananaHand ROS Bringup

## Quick Start
Build and run the full stack:
```bash
cd /home/lokesh/BananaHand/software/ros
source /opt/ros/humble/setup.bash
pip install --user -r requirements.txt
colcon build --symlink-install
source install/setup.bash
ros2 launch banana_bringup bringup.launch.py serial_port:=/dev/ttyACM0 baud:=115200
```

Visualization only:
```bash
cd /home/lokesh/BananaHand/software/ros
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch banana_hand_visualization mujoco_visualization.launch.py show_mujoco_viewer:=true
```

This workspace contains the ROS 2 stack for:
- hand tracking (`banana_hand_tracking`)
- hand mapping (`banana_hand_mapping`)
- serial hardware I/O (`banana_serial_bridge`)
- visualization and debugging (`banana_hand_visualization`)
- top-level bringup (`banana_bringup`)

The default end-to-end flow is:
- webcam + tracking publishes `/hand/teleop_joint_trajectory`
- mapping converts teleop ratios to `/tx_positions`
- serial bridge exchanges `/tx_positions`, `/rx_positions`, `/rx_force`, and `/rx_current` with hardware
- visualization publishes simulated joint state and filtered debug topics for MuJoCo and Foxglove

## Build
```bash
cd software/ros
source /opt/ros/humble/setup.bash
pip install --user -r requirements.txt
colcon build --symlink-install --packages-select \
  banana_interfaces banana_hand_tracking banana_hand_mapping \
  banana_serial_bridge banana_hand_visualization banana_bringup
source install/setup.bash
```

Optional Foxglove bridge:
```bash
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
```

## Run Full Bringup
```bash
ros2 launch banana_bringup bringup.launch.py \
  serial_port:=/dev/ttyACM0 baud:=115200
```

Useful variants:
```bash
# Disable the OpenCV preview window
ros2 launch banana_bringup bringup.launch.py show_preview:=false

# Launch the desktop MuJoCo viewer too
ros2 launch banana_bringup bringup.launch.py show_mujoco_viewer:=true

# Drive the visualization from teleop instead of rx_positions
ros2 launch banana_bringup bringup.launch.py \
  show_mujoco_viewer:=true state_source:=teleop

# Skip the visualization package entirely
ros2 launch banana_bringup bringup.launch.py include_visualization:=false

# Skip vision and mapping, keep serial + visualization
ros2 launch banana_bringup bringup.launch.py \
  include_vision:=false include_mapping:=false
```

Bringup launch arguments:
- `serial_port` default `/dev/ttyACM0`
- `baud` default `115200`
- `include_vision` default `true`
- `include_mapping` default `true`
- `include_visualization` default `true`
- `show_preview` default `true`
- `show_mujoco_viewer` default `false`
- `state_source` default `rx_positions`
- `show_fsr_debugger` default `false`
- `launch_foxglove_bridge` default `true`

## Run Visualization Separately
Launch the visualization stack by itself:
```bash
cd software/ros
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch banana_hand_visualization mujoco_visualization.launch.py
```

Common variants:
```bash
# Open the desktop MuJoCo viewer
ros2 launch banana_hand_visualization mujoco_visualization.launch.py \
  show_mujoco_viewer:=true

# Drive the visualization from tracking output instead of rx_positions
ros2 launch banana_hand_visualization mujoco_visualization.launch.py \
  state_source:=teleop show_mujoco_viewer:=true

# Disable Foxglove bridge startup
ros2 launch banana_hand_visualization mujoco_visualization.launch.py \
  launch_foxglove_bridge:=false
```

## Run Hand Tracking Separately
Combined webcam + tracking:
```bash
cd software/ros
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch banana_hand_tracking vision_teleop.launch.py
```

Tracking-only node:
```bash
ros2 launch banana_hand_tracking hand_tracking.launch.py
```

Webcam-only node:
```bash
ros2 launch banana_hand_tracking webcam.launch.py
```

## Foxglove
The visualization package publishes:
- `/banana_hand/mujoco_joint_states` (`sensor_msgs/msg/JointState`)
- `/banana_hand/force_state` (`sensor_msgs/msg/JointState`)
- `/banana_hand/current_state` (`sensor_msgs/msg/JointState`)

If `foxglove_bridge` is installed, `mujoco_visualization.launch.py` or `bringup.launch.py` can start it automatically.

Suggested Foxglove panels:
1. Plot `/banana_hand/mujoco_joint_states`
2. Plot `/banana_hand/force_state`
3. Plot `/banana_hand/current_state`

Use the desktop MuJoCo viewer for the 3D hand view.

## Quick Checks
```bash
ros2 service call /hand/calibrate std_srvs/srv/Trigger {}
ros2 topic hz /hand/teleop_joint_trajectory
ros2 topic echo /tx_positions
ros2 topic echo /rx_positions
ros2 topic echo /rx_force
ros2 topic echo /banana_hand/mujoco_joint_states
```

## Package Notes
- `banana_hand_tracking`: webcam capture and MediaPipe-based teleop tracking
- `banana_hand_mapping`: converts teleop ratios to actuator-scale command values
- `banana_serial_bridge`: hardware transport for position, force, and current topics
- `banana_hand_visualization`: MuJoCo state publisher, desktop viewer integration, Foxglove-facing debug topics, and FSR debug GUI

Each package README documents its own node parameters and launch usage.
