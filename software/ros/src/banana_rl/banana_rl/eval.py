"""Placeholder evaluation entrypoint for dexterous-hand PPO."""

import argparse
from pathlib import Path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Evaluate a dexterous-hand policy (placeholder)."
    )
    parser.add_argument(
        "--env-id",
        default="TODO_ENV_ID",
        help="Gymnasium-Robotics environment id.",
    )
    parser.add_argument(
        "--model-path",
        default="",
        help="Path to a trained policy checkpoint.",
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=10,
        help="Number of evaluation episodes.",
    )
    parser.add_argument(
        "--render",
        action="store_true",
        help="Enable future environment rendering.",
    )
    parser.add_argument(
        "--output-dir",
        default="artifacts/eval",
        help="Directory for future metrics and optional videos.",
    )
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    print("TODO: Implement policy evaluation for selected dexterous-hand env.")
    print(f"TODO: env_id={args.env_id}")
    print(f"TODO: model_path={args.model_path or 'none'}")
    print(f"TODO: episodes={args.episodes}")
    print(f"TODO: render={args.render}")
    print(f"TODO: output_dir={output_dir}")


if __name__ == "__main__":
    main()
