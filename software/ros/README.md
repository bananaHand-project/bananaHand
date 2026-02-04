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

### How to Run Serial Node
- Start mcu code: `cargo run -p uart_test`
- Start ros code:
```
cd software/ros
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

ros2 launch banana_bringup bringup.launch.py port:=/dev/ttyACM0 baud:=115200
```
- Test ros code:
```
ros2 topic pub --once /tx_positions std_msgs/msg/Float32MultiArray \
"{data: [1,2,3,4,5,6,7,8]}"
```
- Echo topic for verification: `ros2 topic echo /rx_positions`
