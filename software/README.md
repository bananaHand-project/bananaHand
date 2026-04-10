# Software

High-level code for ROS, visualization, and Foxglove tooling.

## Quick Start
ROS bringup:
```bash
cd /home/lokesh/BananaHand/software/ros
source /opt/ros/humble/setup.bash
pip install --user -r requirements.txt
colcon build --symlink-install
source install/setup.bash
ros2 launch banana_bringup bringup.launch.py serial_port:=/dev/ttyACM0 baud:=115200
```

ROS visualization only:
```bash
cd /home/lokesh/BananaHand/software/ros
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch banana_hand_visualization mujoco_visualization.launch.py show_mujoco_viewer:=true
```

Foxglove extension:
```bash
cd /home/lokesh/BananaHand/software/foxglove/banana-hand-force-heatmap
npm install
npm run local-install
```

More detail:
- [ROS workspace](./ros/README.md)
- [Foxglove extensions](./foxglove/README.md)
