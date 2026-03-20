# Banana Hand Visualization

`banana_hand_visualization` is the ROS 2 visualization package for the Banana Hand system.

It contains:
- the main visualization node that drives MuJoCo state from live ROS topics
- optional desktop MuJoCo viewing
- Foxglove-facing debug topics
- the FSR debug GUI and test publisher

## Install
```bash
cd /home/lokesh/BananaHand/software/ros
source /opt/ros/humble/setup.bash
pip install --user -r requirements.txt
colcon build --symlink-install --packages-select banana_hand_visualization
source install/setup.bash
```

Optional Foxglove bridge:
```bash
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
```

## Run
Launch the visualization stack:
```bash
ros2 launch banana_hand_visualization mujoco_visualization.launch.py
```

Useful variants:
```bash
# Open the desktop MuJoCo viewer too
ros2 launch banana_hand_visualization mujoco_visualization.launch.py \
  show_mujoco_viewer:=true

# Drive MuJoCo from teleop tracking instead of rx_positions
ros2 launch banana_hand_visualization mujoco_visualization.launch.py \
  state_source:=teleop show_mujoco_viewer:=true

# Do not start Foxglove bridge from the launch file
ros2 launch banana_hand_visualization mujoco_visualization.launch.py \
  launch_foxglove_bridge:=false
```

Standalone viewer only:
```bash
ros2 run banana_hand_visualization standalone_viewer
```

## Topics
Subscriptions:
- `/rx_positions` (`sensor_msgs/msg/JointState`)
- `/hand/teleop_joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`)
- `/rx_force` (`std_msgs/msg/UInt16MultiArray`)
- `/rx_current` (`sensor_msgs/msg/JointState`)

Publications:
- `/banana_hand/mujoco_joint_states` (`sensor_msgs/msg/JointState`)
- `/banana_hand/force_state` (`sensor_msgs/msg/JointState`)
- `/banana_hand/current_state` (`sensor_msgs/msg/JointState`)

`/banana_hand/force_state` is published in this order:
- `thumb`
- `index`
- `middle`
- `ring`
- `pinky`
- `palm_1`
- `palm_2`
- `palm_3`
- `palm_4`
- `palm_5`

## Launch Arguments
`mujoco_visualization.launch.py` supports:
- `show_mujoco_viewer` default `false`
- `state_source` default `rx_positions`
- `launch_foxglove_bridge` default `true`

## Main Visualization Node Parameters
The `mujoco_visualizer` node supports:

- `model_path`
  default `/share/banana_hand_visualization/mujoco/scene.xml` inside the installed package
- `state_publish_rate_hz`
  default `30.0`
- `show_mujoco_viewer`
  default `false`
- `state_source`
  default `rx_positions`
  options: `rx_positions`, `teleop`
- `teleop_topic`
  default `/hand/teleop_joint_trajectory`
- `feedback_topic`
  default `/rx_positions`
- `teleop_input_min_ratio`
  default `0.0`
- `teleop_input_max_ratio`
  default `1.0`
- `teleop_source_indices`
  default `[0, 1, 2, 3, 4, 5, -1, -1]`
- `teleop_fill_ratio`
  default `0.0`
- `force_input_topic`
  default `/rx_force`
- `current_input_topic`
  default `/rx_current`
- `joint_state_topic`
  default `/banana_hand/mujoco_joint_states`
- `force_state_topic`
  default `/banana_hand/force_state`
- `current_state_topic`
  default `/banana_hand/current_state`
- `force_sensor_indices`
  default `[0, 1, 2, 3, 4, 5, 6, 7, 8, 9]`
- `force_scale`
  default `1.0`
- `current_filter_window`
  default `10`
- `actuator_map`
  default `["indexpiston", "middlepiston", "ringpiston", "pinkypiston", "thumbpiston", "tmpiston", "", ""]`
- `direct_joint_map`
  default `["", "", "", "", "", "", "indexrev4", "thumbrev4"]`
- `motor_mins`
  default `[400, 400, 400, 400, 300, 150, 0, 0]`
- `motor_maxs`
  default `[3900, 3900, 3900, 3900, 3900, 3900, 4095, 4095]`
- `position_invert_mask`
  default `[true, true, true, true, true, true, false, false]`
  order: `index_1, middle, ring, pinky, thumb_1, thumb_2, index_2, thumb_3`

Example:
```bash
ros2 run banana_hand_visualization mujoco_visualizer --ros-args \
  -p state_source:=teleop \
  -p show_mujoco_viewer:=true \
  -p position_invert_mask:="[true, true, true, true, true, true, false, false]"
```

## FSR Debug GUI
Run against live data:
```bash
ros2 run banana_hand_visualization fsr_visualizer
```

Launch with a synthetic publisher:
```bash
ros2 launch banana_hand_visualization fsr_visualizer_test.launch.py
```

Run only the synthetic publisher:
```bash
ros2 run banana_hand_visualization fsr_visualizer_test_publisher
```

FSR GUI parameters:
- `topic_name`
  default `rx_force`
- `finger_indices`
  default `[0, 1, 2, 3, 4]`
- `alpha`
  default `0.25`
- `global_min`
  default `0.0`
- `global_max`
  default `1023.0`
- `per_finger_min`
  default `[]`
- `per_finger_max`
  default `[]`
- `refresh_hz`
  default `30.0`
- `contact_threshold`
  default `0.7`
- `simulate_if_no_data`
  default `false`

Example:
```bash
ros2 run banana_hand_visualization fsr_visualizer --ros-args \
  -p topic_name:=rx_force \
  -p finger_indices:="[0, 2, 4, 6, 8]" \
  -p alpha:=0.35
```

## Foxglove
If `foxglove_bridge` is installed, the visualization launch file can start it automatically.

Suggested panels:
1. Plot `/banana_hand/mujoco_joint_states`
2. Plot `/banana_hand/force_state`
3. Plot `/banana_hand/current_state`

Use the desktop MuJoCo viewer for 3D hand motion rather than Foxglove markers.
