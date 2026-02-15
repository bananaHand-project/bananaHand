"""Placeholder training entrypoint for dexterous-hand PPO."""

import argparse
from pathlib import Path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Train a dexterous-hand policy with SB3 PPO (placeholder)."
    )
    parser.add_argument(
        "--env-id",
        default="TODO_ENV_ID",
        help="Gymnasium-Robotics environment id.",
    )
    parser.add_argument(
        "--total-timesteps",
        type=int,
        default=1_000_000,
        help="Total timesteps for future training runs.",
    )
    parser.add_argument(
        "--config",
        default="",
        help="Optional path to a training YAML config file.",
    )
    parser.add_argument(
        "--output-dir",
        default="artifacts/rl",
        help="Directory for future checkpoints and logs.",
    )
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    print("TODO: Implement SB3 PPO training for selected dexterous-hand env.")
    print(f"TODO: env_id={args.env_id}")
    print(f"TODO: total_timesteps={args.total_timesteps}")
    print(f"TODO: config={args.config or 'none'}")
    print(f"TODO: output_dir={output_dir}")


if __name__ == "__main__":
    main()
