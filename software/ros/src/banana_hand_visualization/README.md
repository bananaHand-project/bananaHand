# Banana Hand Visualization

Publishes the MuJoCo Banana Hand model into ROS 2 as:
- `sensor_msgs/msg/JointState` on `/banana_hand/mujoco_joint_states`
- `sensor_msgs/msg/JointState` on `/banana_hand/force_state`

Use MuJoCo itself for 3D viewing, Foxglove for plots and state traces, and the optional FSR GUI for raw force debugging.

## Install
```bash
cd software/ros
source /opt/ros/humble/setup.bash
pip install --user mujoco
sudo apt install python3-pyside6
colcon build --symlink-install --packages-select banana_hand_visualization
source install/setup.bash
```

## Run
```bash
ros2 launch banana_hand_visualization mujoco_visualization.launch.py
```

With the MuJoCo desktop viewer too:
```bash
ros2 launch banana_hand_visualization mujoco_visualization.launch.py show_mujoco_viewer:=true
```

Standalone MuJoCo viewer only:
```bash
ros2 run banana_hand_visualization standalone_viewer
```

ROS visualization without Foxglove Bridge:
```bash
ros2 launch banana_hand_visualization mujoco_visualization.launch.py \
  launch_foxglove_bridge:=false show_mujoco_viewer:=true
```

Optional FSR debugger window:
```bash
ros2 launch banana_hand_visualization fsr_visualizer.launch.py
```

FSR debugger with a synthetic test publisher:
```bash
ros2 launch banana_hand_visualization fsr_visualizer_test.launch.py
```

If `foxglove_bridge` is installed, the launch file will start it automatically.

## ROS Topics
- Subscribes: `/rx_positions` (`sensor_msgs/msg/JointState`, default MuJoCo state source)
- Subscribes: `/hand/teleop_joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`, optional MuJoCo state source when `state_source:=teleop`)
- Parameter: `state_source` (`rx_positions` or `teleop`)
- Parameter: `position_invert_mask` (8 booleans for `index_1,middle,ring,pinky,thumb_1,thumb_2,index_2,thumb_3`)
- Subscribes: `/rx_force` (`std_msgs/msg/UInt16MultiArray`)
- Publishes: `/banana_hand/mujoco_joint_states`
- Publishes: `/banana_hand/force_state` (`thumb,index,middle,ring,pinky,palm_1,palm_2,palm_3,palm_4,palm_5`)

## Foxglove Test Loop
Run MuJoCo + Foxglove + a synthetic force publisher:
```bash
ros2 launch banana_hand_visualization foxglove_test.launch.py show_mujoco_viewer:=true
```

In Foxglove:
1. Add a Plot panel.
2. Plot selected joints from `/banana_hand/mujoco_joint_states` if you want to watch internal linkage motion.
3. Plot or inspect `/banana_hand/force_state` for fingertip and palm sensors.

The test launch publishes synthetic `/rx_force` values so the force state animates without hardware.

## FSR Debug GUI
Run the GUI alone against live hardware data:
```bash
ros2 run banana_hand_visualization fsr_visualizer
```

Run only the synthetic publisher:
```bash
ros2 run banana_hand_visualization fsr_visualizer_test_publisher
```

Run both together:
```bash
ros2 launch banana_hand_visualization fsr_visualizer_test.launch.py
```

## Foxglove
1. Open a live connection to the Foxglove Bridge.
2. Add one or more Plot panels.
3. Use MuJoCo itself for the 3D hand visualization.
