# banana_3d_reconstruction

Burst-mode RGB-D tabletop object scanning package for Intel RealSense depth cameras.

## What It Does

- Starts aligned RealSense color and depth streams with `pyrealsense2`.
- Shows a live OpenCV preview so you can always see the camera feed.
- Captures a short burst while the camera is held mostly still.
- Segments the tabletop object from the full frame on every burst frame.
- Fuses only the selected object-only clouds.
- Publishes the final clean cloud on `/object/point_cloud`.
- Publishes the saved scan directory on `/object_scan/completed_scan_dir` after each successful scan save.
- Saves the final cloud, downsampled cloud, one RGB image, metadata, and optional debug artifacts in a timestamped scan folder.

This package is intentionally optimized for one practical Demo 2 workflow:
- one object on a mostly clear desk
- camera held mostly still
- good diagonal viewing angle so the camera sees a few faces of the object
- clean partial object cloud for later geometric grip classification

## Recommended Workflow

1. Put one object on a mostly clear desk.
2. Hold the RealSense mostly still at a good diagonal angle.
3. Start the burst scan.
4. Let the node segment the tabletop object automatically from the full frame.
5. Use the saved partial point cloud for downstream geometry features or classification.

This is not trying to be a full 360 reconstruction system.

## Build

If you only want the scan node itself:

```bash
cd software/ros
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select banana_3d_reconstruction
source install/setup.bash
```

If you want the combined scan -> ground removal -> grasp-classification launch from `banana_grasp_classification`:

```bash
cd software/ros
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  banana_interfaces banana_3d_reconstruction banana_grasp_classification
source install/setup.bash
```

## Runtime Dependencies

ROS 2 / Python packages used here include:
- `rclpy`
- `sensor_msgs`
- `sensor_msgs_py`
- `std_msgs`
- `std_srvs`
- `cv_bridge`
- `numpy`
- `opencv-python`
- `open3d`
- `pyrealsense2`

Important:
- `colcon build` in this workspace builds the ROS package, but it does not automatically install Python packages like `open3d` and `pyrealsense2`.
- Those may need manual installation on the machine before runtime.

## Run

```bash
cd software/ros
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch banana_3d_reconstruction object_scan.launch.py
```

Optional overrides:

```bash
ros2 launch banana_3d_reconstruction object_scan.launch.py \
  output_dir:=/tmp/banana_scans \
  show_preview:=true \
  device_serial:=<optional_camera_serial>
```

Optional service start:

```bash
ros2 service call /object_scan/start_scan std_srvs/srv/Trigger "{}"
```

End-to-end scan -> ground removal -> grasp classification:

```bash
ros2 launch banana_grasp_classification scan_to_grasp.launch.py
```

## Preview Controls

- `s`: start a burst scan
- `e`: end the current burst early
- `q`: quit the node

The preview overlays show:
- burst mode only
- one object on a mostly clear desk
- hold the camera mostly still at a good diagonal angle
- current node state
- the basic controls

## Operator Instructions

1. Launch the node.
2. Place one object on a mostly clear tabletop.
3. Hold the RealSense mostly still at a good diagonal angle so it sees multiple faces of the object.
4. Press `s` or call `/object_scan/start_scan`.
5. Keep the camera still during the short burst.
6. Wait for processing to finish.
7. Check `final_object_cloud.ply` or subscribe to `/object/point_cloud`.

If you are running the combined scan-to-grasp launch, the saved scan directory is also published on `/object_scan/completed_scan_dir` so downstream grasp processing can start immediately.

## Automatic Object Selection

Each burst frame uses the same full-frame tabletop segmentation path:

1. Build the aligned RGB-D point cloud from the full frame.
2. Fit the dominant plane from the full frame.
3. Remove the desk plane.
4. Cluster the above-plane points.
5. Reject degenerate clusters such as tiny slivers, line-like fragments, and sparse junk.
6. Prefer the remaining clusters that are:
   - larger
   - closer to the image center
   - more object-like in 3D extent

This assumes there is one main tabletop object and minimal clutter.

## Burst Fusion

- The node assumes very small camera motion.
- It fuses only the selected object clouds, never the full raw scene.
- It tries conservative ICP first.
- If ICP is not good enough but the selected cluster centroid barely moved, it falls back to identity accumulation.
- If a frame looks bad, it is skipped.

## Final Cleanup

After burst fusion, the node runs aggressive cleanup:

1. One more dominant-plane removal on the fused cloud when possible.
2. Keep only points above that plane.
3. Voxel downsample.
4. Statistical outlier removal.
5. Radius outlier removal.
6. Largest-cluster extraction.
7. Normal estimation.

This is specifically meant to remove leftover desk patches and small junk before saving the final cloud.

## Topics

- `/object/point_cloud` (`sensor_msgs/msg/PointCloud2`): final cleaned object cloud
- `/object_scan/completed_scan_dir` (`std_msgs/msg/String`): absolute path to the newly saved scan directory
- `/object_scan/status` (`std_msgs/msg/String`): scan state updates
- `/object_scan/debug/raw_frame_cloud` (`sensor_msgs/msg/PointCloud2`, optional)
- `/object_scan/debug/plane_removed_cloud` (`sensor_msgs/msg/PointCloud2`, optional)
- `/object_scan/debug/selected_object_cloud` (`sensor_msgs/msg/PointCloud2`, optional)

## Services

- `/object_scan/start_scan` (`std_srvs/srv/Trigger`)
- `/object_scan/save_last_scan` (`std_srvs/srv/Trigger`)

## Saved Outputs

- `scan_YYYYMMDD_HHMMSS/final_object_cloud.ply`
- `scan_YYYYMMDD_HHMMSS/final_object_cloud_downsampled.ply`
- `scan_YYYYMMDD_HHMMSS/final_rgb_image.png`
- `scan_YYYYMMDD_HHMMSS/metadata.json`

When `save_intermediate_debug_clouds:=true`, the package also saves:
- raw cloud per keyframe
- plane-removed cloud per keyframe
- selected object cloud per keyframe
- registered selected cloud per accepted keyframe
- `fused_segmented_cloud_pre_cleanup.ply`
- `final_cleaned_object_cloud.ply`
- one debug JSON per frame with candidate cluster stats, degeneracy checks, selection reason, and accept/reject status

## Limitations

- This produces a practical partial tabletop object cloud, not a guaranteed watertight 360 reconstruction.
- The underside of an object resting on a desk is not observable in a normal tabletop scan.
- It assumes one main object on a mostly clear desk.
- Large camera motion during the burst will reduce quality because the node is intentionally tuned for mostly-still capture, not handheld reconstruction.
