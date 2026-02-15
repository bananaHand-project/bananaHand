# banana_sim_bridge

MuJoCo simulation and ROS bridge scaffold package.

Purpose
- Provide a ROS-facing trigger/report interface for rollout episodes.
- Bridge grasp requests to policy rollouts in a selected dexterous-hand env.

Future interfaces
- Subscribes to `/sim/requested_grip_type` (`std_msgs/msg/String`).
- Publishes `/sim/success_rate` (`std_msgs/msg/Float32`).
- Publishes `/sim/artifact_path` (`std_msgs/msg/String`) for logs/videos.
- CLI rollout entrypoint will run episodes with a selected trained policy.

Current status
- Placeholder ROS node and placeholder rollout CLI only.
- No simulator integration, policy loading, metrics computation, or video export yet.
