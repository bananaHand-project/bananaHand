### How to add packages
Example: Creating webcam_node
- Start in BananaHand root
- Navigate to ROS directory: `cd software/ros`
- Create Package: `ros2 pkg create banana_webcam --build-type ament_python --dependencies rclpy sensor_msgs`
- Add to Launch file: `Node(package="banana_webcam", executable="webcam_node", name="webcam", output="screen")`
- Create webcam main file in the banana_webcam package

### Vision Teleop
Packages added under `software/ros/src`:
- `banana_hand_tracking` (webcam capture + `/camera/image_raw`)
- `banana_hand_mapping` (placeholder)
- `banana_interfaces` (custom messages)
- `banana_sfm_reconstruction` (SfM / 3D reconstruction scaffold)
- `banana_grasp_strategy` (grip type prediction scaffold)
- `banana_rl` (MuJoCo + Gymnasium-Robotics + SB3 PPO scaffold)
- `banana_sim_bridge` (simulation rollout + ROS bridge scaffold)

### RealSense Camera + COLMAP 3D Reconstruction

See [banana_sfm_reconstruction](src/banana_sfm_reconstruction/README.md) for full details.

Quick start:
```bash
cd software/ros
source /opt/ros/humble/setup.bash
pip install --user -r src/banana_sfm_reconstruction/requirements.txt
colcon build --packages-select banana_sfm_reconstruction
source install/setup.bash
ros2 run banana_sfm_reconstruction reconstruction_node
```

In another terminal, monitor frames:
```bash
ros2 topic hz /camera/image_raw
ls -lh src/banana_sfm_reconstruction/banana_sfm_reconstruction/frames/
```

Once you have frames, process with COLMAP:
```bash
colmap automatic_reconstructor \
  --workspace_path /tmp/colmap_ws \
  --image_path software/ros/src/banana_sfm_reconstruction/banana_sfm_reconstruction/frames
```

### Vision Teleop

Run vision teleop:
```
cd software/ros
source /opt/ros/humble/setup.bash
pip install --user -r src/banana_hand_tracking/requirements.txt
colcon build --symlink-install --packages-select banana_interfaces banana_hand_tracking
source install/setup.bash
ros2 launch banana_hand_tracking vision_teleop.launch.py
```

Verify:
```
ros2 topic hz /camera/image_raw
ros2 topic hz /hand/landmarks
ros2 topic hz /hand/teleop_joint_trajectory
```

Calibrate:
```
ros2 service call /hand/calibrate std_srvs/srv/Trigger {}
```

### Serial Bridge (Current End-to-End Test Flow)

Current data flow:
- `C0 (force_sensor_test) -> G4 (uart_g4_combo) -> PC (ROS serial bridge)` for force + position telemetry
- `PC (ROS /tx_positions) -> G4` for position command frames

#### 1) Flash and run firmware
- C0 board: run `firmware/experiments/force_sensor_test`
- G4 board: run `firmware/experiments/uart_g4_combo`

#### 2) Build and launch ROS serial bridge
```bash
### SfM + Grasp + RL + Sim Scaffolds
Build scaffold packages:
```
cd software/ros
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  banana_sfm_reconstruction \
  banana_grasp_strategy \
  banana_rl \
  banana_sim_bridge
source install/setup.bash
```

Optional runtime Python deps for MuJoCo/Gymnasium/SB3 workflows:
```
pip install --user -r src/banana_rl/requirements.txt
pip install --user -r src/banana_sim_bridge/requirements.txt
```

Run placeholders:
```
ros2 launch banana_sfm_reconstruction reconstruction.launch.py
ros2 launch banana_grasp_strategy grasp_strategy.launch.py
ros2 launch banana_sim_bridge sim_bridge.launch.py
ros2 run banana_rl train_rl --help
ros2 run banana_rl eval_rl --help
ros2 run banana_sim_bridge rollout_policy --help
```

### How to Run Serial Node
- Start mcu code: `cargo run -p uart_test`
- Start ros code:
```
cd software/ros
source /opt/ros/humble/setup.bash
colcon build --packages-select banana_serial_bridge banana_bringup
source install/setup.bash
ros2 launch banana_bringup bringup.launch.py port:=/dev/ttyACM0 baud:=115200
```

#### 3) Send command from PC to G4
```bash
ros2 topic pub --once /tx_positions std_msgs/msg/UInt16MultiArray \
"{data: [1,2,3,4,5,6,7,8]}"
```

#### 4) Verify telemetry topics from G4
- Positions:
```bash
ros2 topic echo /rx_positions
```
- Force sensor array:
```bash
ros2 topic echo /rx_force
```

#### 5) Quick debug checks
```bash
ros2 topic info /tx_positions
ros2 topic hz /rx_positions
ros2 topic hz /rx_force
```
- `/tx_positions` should show one subscriber (`banana_serial_bridge`)
- `/rx_positions` and `/rx_force` should both stream while firmware is running
