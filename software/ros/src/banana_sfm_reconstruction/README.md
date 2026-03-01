# banana_sfm_reconstruction

SfM and 3D reconstruction scaffold package.

## Purpose

- Capture images from Intel RealSense camera via ROS 2
- Save frames to disk for downstream COLMAP processing
- Provide a ROS entrypoint for future COLMAP-based reconstruction
- Prepare outputs for reconstructed geometry and metadata

## Current Features

- **RealSense Integration**: Captures color frames from connected RealSense device
- **Image Publishing**: Publishes frames on `/camera/image_raw` for ROS ecosystem
- **Frame Storage**: Automatically saves PNGs to local `frames/` directory

## Future Interfaces

- Publishes `/reconstruction/point_cloud` (`sensor_msgs/msg/PointCloud2`)
- Publishes `/reconstruction/metadata` (`std_msgs/msg/String`)
- COLMAP integration (SfM, dense reconstruction, mesh export)

## Hardware Setup

1. **Plug in RealSense camera** (e.g., D435, D435i) to USB 3.0 port

2. **Install RealSense libraries** (one-time):
  ```bash
  sudo apt-get install -y librealsense2
  ```

3. **Verify camera is detected**:
  ```bash
  rs-enumerate-devices
  ```
  Should list your connected device. If empty, check USB connection and permissions.

## Build

```bash
cd /home/dbhaumik/BananaHand/software/ros
source /opt/ros/humble/setup.bash
colcon build --packages-select banana_sfm_reconstruction
source install/setup.bash
```

## Run

```bash
ros2 run banana_sfm_reconstruction reconstruction_node
```

**Output directories:**
- Frames saved to: `software/ros/src/banana_sfm_reconstruction/banana_sfm_reconstruction/frames/`
- Named as: `frame_00000.png`, `frame_00001.png`, etc.

## Verify Capture

In another terminal:

```bash
# List active topics
ros2 topic list

# Check frame rate (should be ~30 Hz by default)
ros2 topic hz /camera/image_raw

# View live feed (requires rqt installed)
ros2 run rqt_image_view rqt_image_view
```

Then select `/camera/image_raw` from the dropdown to see live camera feed.

**Check saved frames:**
```bash
ls -lh software/ros/src/banana_sfm_reconstruction/banana_sfm_reconstruction/frames/ | head -20
```

Should see PNG files appearing in real-time as the capture runs.

## Parameters

Customize at launch:

```bash
ros2 run banana_sfm_reconstruction reconstruction_node \
  --ros-args \
  -p realsense_width:=1280 \
  -p realsense_height:=720 \
  -p realsense_fps:=15
```

### Available Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `realsense_width` | int | 640 | Camera resolution width (pixels) |
| `realsense_height` | int | 480 | Camera resolution height (pixels) |
| `realsense_fps` | int | 30 | Capture frame rate (Hz) |
| `input_image_topic` | string | `/camera/image_raw` | ROS topic for color image output |
| `output_pointcloud_topic` | string | `/reconstruction/point_cloud` | Future point cloud topic |
| `output_metadata_topic` | string | `/reconstruction/metadata` | Future metadata topic |

## Troubleshooting

**No frames appearing:**
- Run `rs-enumerate-devices` to confirm camera is detected
- Check USB connection (use USB 3.0 port for D435)
- Verify permissions: `ls -l /dev/video*`

**Topics not showing up:**
- Ensure node is actually running: `ros2 node list | grep reconstruction`
- Rebuild package: `colcon build --packages-select banana_sfm_reconstruction`

**High CPU usage:**
- Reduce `realsense_fps` parameter
- Reduce resolution (`realsense_width`/`realsense_height`)

## Next Steps (COLMAP Integration)

1. **Capture baseline dataset:**
  - Run node and move camera around object in circular motion
  - Aim for 100-300 frames with good overlap

2. **Run COLMAP reconstruction:**
  ```bash
  colmap automatic_reconstructor \
    --workspace_path /tmp/colmap_ws \
    --image_path software/ros/src/banana_sfm_reconstruction/banana_sfm_reconstruction/frames
  ```

3. **Inspect results:**
  - Point cloud: `colmap model_converter --input_path /tmp/colmap_ws/sparse/0 --output_path /tmp/colmap_ws/model.ply --output_type PLY`
  - View in CloudCompare or Meshlab

4. **Future:** Integrate COLMAP binary call directly in this node or a downstream orchestration script

## Dependencies

See [requirements.txt](requirements.txt):
- `pyrealsense2` — RealSense SDK for Python
- `opencv-python` — Image I/O and processing
- `numpy` — Numerical operations
- Standard ROS 2 (rclpy, sensor_msgs, cv_bridge)
