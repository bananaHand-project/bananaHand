"""Placeholder rollout entrypoint for simulation runs."""

import argparse
from pathlib import Path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run MuJoCo dexterous-hand rollouts with a selected policy (placeholder)."
    )
    parser.add_argument(
        "--env-id",
        default="TODO_ENV_ID",
        help="Gymnasium-Robotics environment id.",
    )
    parser.add_argument(
        "--policy-path",
        default="",
        help="Path to a trained policy checkpoint.",
    )
    parser.add_argument(
        "--grip-type",
        default="tripod",
        choices=["spherical", "cylindrical", "pinch", "hook", "tripod"],
        help="Requested grip strategy label.",
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=5,
        help="Number of episodes for the future rollout job.",
    )
    parser.add_argument(
        "--output-dir",
        default="artifacts/sim",
        help="Directory for future metrics and optional videos.",
    )
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    print("TODO: Implement MuJoCo rollout runner for requested grip policies.")
    print(f"TODO: env_id={args.env_id}")
    print(f"TODO: policy_path={args.policy_path or 'none'}")
    print(f"TODO: grip_type={args.grip_type}")
    print(f"TODO: episodes={args.episodes}")
    print(f"TODO: output_dir={output_dir}")


if __name__ == "__main__":
    main()
