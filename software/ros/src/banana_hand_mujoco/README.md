# Banana Hand MuJoCo State Publisher

Publishes the MuJoCo Banana Hand model into ROS 2 as:
- `sensor_msgs/msg/JointState` on `/banana_hand/mujoco_joint_states`
- `sensor_msgs/msg/JointState` on `/banana_hand/actuator_state`
- `sensor_msgs/msg/JointState` on `/banana_hand/command_state`
- `sensor_msgs/msg/JointState` on `/banana_hand/force_state`
- `std_msgs/msg/Float32` on `/banana_hand/force/thumb`
- `std_msgs/msg/Float32` on `/banana_hand/force/index`
- `std_msgs/msg/Float32` on `/banana_hand/force/middle`
- `std_msgs/msg/Float32` on `/banana_hand/force/ring`
- `std_msgs/msg/Float32` on `/banana_hand/force/pinky`

Use MuJoCo itself for 3D viewing, and Foxglove for plots and state traces.

## Install
```bash
cd software/ros
source /opt/ros/humble/setup.bash
pip install --user mujoco
colcon build --symlink-install --packages-select banana_hand_mujoco
source install/setup.bash
```

## Run
```bash
ros2 launch banana_hand_mujoco mujoco_visualization.launch.py
```

With the MuJoCo desktop viewer too:
```bash
ros2 launch banana_hand_mujoco mujoco_visualization.launch.py show_mujoco_viewer:=true
```

Standalone MuJoCo viewer only:
```bash
ros2 run banana_hand_mujoco standalone_viewer
```

ROS visualization without Foxglove Bridge:
```bash
ros2 launch banana_hand_mujoco mujoco_visualization.launch.py \
  launch_foxglove_bridge:=false show_mujoco_viewer:=true
```

If `foxglove_bridge` is installed, the launch file will start it automatically.

## ROS Topics
- Subscribes: `/banana_hand/actuator_commands` (`std_msgs/msg/Float32MultiArray`, 6 normalized values in `0..1`)
- Subscribes: `/tx_positions` (`std_msgs/msg/UInt16MultiArray`)
- Subscribes: `/rx_positions` (`sensor_msgs/msg/JointState`)
- Subscribes: `/rx_force` (`std_msgs/msg/UInt16MultiArray`)
- Publishes: `/banana_hand/mujoco_joint_states`
- Publishes: `/banana_hand/actuator_state`
- Publishes: `/banana_hand/command_state`
- Publishes: `/banana_hand/force_state`
- Publishes: `/banana_hand/force/thumb`
- Publishes: `/banana_hand/force/index`
- Publishes: `/banana_hand/force/middle`
- Publishes: `/banana_hand/force/ring`
- Publishes: `/banana_hand/force/pinky`

## Foxglove Test Loop
Run MuJoCo + Foxglove + a 6-channel test publisher:
```bash
ros2 launch banana_hand_mujoco foxglove_test.launch.py show_mujoco_viewer:=true
```

The test publisher drives `/banana_hand/actuator_commands` with 6 normalized sine waves. In Foxglove:
1. Add a Plot panel.
2. Plot `/banana_hand/command_state.position[0..5]` for the commanded traces.
3. Plot `/banana_hand/actuator_state.position[0..5]` for the actual actuator controls applied in MuJoCo.
4. Plot selected joints from `/banana_hand/mujoco_joint_states` if you want to watch internal linkage motion.
5. Add Gauge panels for `/banana_hand/force/thumb`, `/banana_hand/force/index`, `/banana_hand/force/middle`, `/banana_hand/force/ring`, and `/banana_hand/force/pinky`.

The test launch also publishes synthetic `/rx_force` values so the gauges animate without hardware.

## Foxglove
1. Open a live connection to the Foxglove Bridge.
2. Add one or more Plot panels.
3. Use MuJoCo itself for the 3D hand visualization.
