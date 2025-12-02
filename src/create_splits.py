import argparse
import json
import random
from pathlib import Path
from typing import Dict, List, Tuple


def find_images(root: Path, extensions: Tuple[str, ...]) -> List[Path]:
    images: List[Path] = []
    for ext in extensions:
        images.extend(root.rglob(f"*{ext}"))
    return images


def build_index(raw_dir: Path) -> List[Tuple[Path, str]]:
    """Return list of (image_path, label) pairs from data/raw/<class>/."""
    samples: List[Tuple[Path, str]] = []
    for class_dir in sorted(raw_dir.iterdir()):
        if not class_dir.is_dir():
            continue
        label = class_dir.name
        images = find_images(class_dir, extensions=(".jpg", ".jpeg", ".png"))
        for img in images:
            samples.append((img, label))
    return samples


def split_dataset(
    samples: List[Tuple[Path, str]],
    train_ratio: float,
    val_ratio: float,
    seed: int,
) -> Dict[str, List[Dict[str, str]]]:
    """Split per class into train/val/test using given ratios."""
    random.seed(seed)

    by_label: Dict[str, List[Path]] = {}
    for path, label in samples:
        by_label.setdefault(label, []).append(path)

    splits: Dict[str, List[Dict[str, str]]] = {"train": [], "val": [], "test": []}

    for label, paths in by_label.items():
        random.shuffle(paths)
        n = len(paths)
        n_train = int(n * train_ratio)
        n_val = int(n * val_ratio)
        n_test = n - n_train - n_val

        train_paths = paths[:n_train]
        val_paths = paths[n_train : n_train + n_val]
        test_paths = paths[n_train + n_val :]

        for subset, subset_paths in (
            ("train", train_paths),
            ("val", val_paths),
            ("test", test_paths),
        ):
            for p in subset_paths:
                splits[subset].append(
                    {
                        "path": str(p.as_posix()),
                        "label": label,
                    }
                )

    return splits


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Create train/val/test splits from data/raw/<class>/ images."
    )
    parser.add_argument(
        "--raw-dir",
        type=str,
        default="data/raw",
        help="Root directory with class subfolders.",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="data/splits.json",
        help="Output JSON file with train/val/test indices.",
    )
    parser.add_argument(
        "--train-ratio",
        type=float,
        default=0.7,
        help="Fraction of data per class used for training.",
    )
    parser.add_argument(
        "--val-ratio",
        type=float,
        default=0.15,
        help="Fraction of data per class used for validation.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed for shuffling.",
    )
    args = parser.parse_args()

    raw_dir = Path(args.raw_dir)
    if not raw_dir.exists():
        raise SystemExit(f"Raw data directory not found: {raw_dir}")

    samples = build_index(raw_dir)
    if not samples:
        raise SystemExit(f"No images found under {raw_dir}")

    splits = split_dataset(
        samples,
        train_ratio=args.train_ratio,
        val_ratio=args.val_ratio,
        seed=args.seed,
    )

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with output_path.open("w", encoding="utf-8") as f:
        json.dump(splits, f, indent=2)

    print(f"Wrote splits to {output_path}")


if __name__ == "__main__":
    main()

