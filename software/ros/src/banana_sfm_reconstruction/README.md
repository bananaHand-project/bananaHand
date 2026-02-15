# banana_sfm_reconstruction

SfM and 3D reconstruction scaffold package.

Purpose
- Provide a ROS entrypoint for future COLMAP-based reconstruction.
- Prepare outputs for reconstructed geometry and metadata.

Future interfaces
- Publishes `/reconstruction/point_cloud` (`sensor_msgs/msg/PointCloud2`).
- Publishes `/reconstruction/metadata` (`std_msgs/msg/String`, placeholder for
  serialized metadata).
- Will consume camera imagery and optional calibration inputs.

Current status
- Placeholder node only.
- No SfM, dense reconstruction, or mesh export logic is implemented yet.
