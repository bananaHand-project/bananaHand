# banana_rl

MuJoCo + Gymnasium-Robotics + SB3 PPO scaffold package.

Purpose
- Host training/evaluation entrypoints for dexterous-hand tasks.
- Keep RL workflows ROS-adjacent without forcing ROS runtime nodes.

Stack choice
- MuJoCo physics engine.
- Gymnasium-Robotics dexterous-hand environments (initial target).
- Stable-Baselines3 PPO as the baseline training algorithm.

Entry points
- `train_rl` (placeholder training CLI).
- `eval_rl` (placeholder evaluation CLI).

Current status
- CLI scaffolding only with argument parsing and TODO output.
- No RL training or evaluation logic implemented yet.
