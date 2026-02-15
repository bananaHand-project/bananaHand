# banana_grasp_strategy

Grasp strategy prediction scaffold package.

Purpose
- Provide a ROS node where grasp prediction inference logic will live.
- Produce a grip type and confidence for downstream control/simulation.

Future interfaces
- Subscribes to `/object/point_cloud` (`sensor_msgs/msg/PointCloud2`) as a
  placeholder object representation interface.
- Publishes `/grasp/grip_type` (`std_msgs/msg/String`) with one of:
  `spherical`, `cylindrical`, `pinch`, `hook`, `tripod`.
- Publishes `/grasp/grip_confidence` (`std_msgs/msg/Float32`).

Current status
- Placeholder node only.
- No baseline feature model or PointNet-style inference is implemented yet.
