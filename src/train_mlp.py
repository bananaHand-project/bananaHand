"""
Two-head MLP baselines (object + grip via fixed mapping).
- Uses precomputed splits.json; images are resized, flattened, and fed to MLPClassifier.
- Runs 5 hardcoded trials with varied hidden sizes / activation / learning rate.
- Per trial, saves plots under plots_mlp/trial_*:
  * confusion matrices (object + grip, val + test)
  * precision/recall bars (object + grip, val + test)
Summary plot compares test accuracy across trials for object vs grip heads.
"""

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from PIL import Image
from sklearn.metrics import (
    accuracy_score,
    classification_report,
    confusion_matrix,
    precision_recall_fscore_support,
)
from sklearn.neural_network import MLPClassifier
from sklearn.preprocessing import LabelEncoder

# Fixed mapping object -> grip
OBJECT_TO_GRIP = {
    "apple": "spherical",
    "bottle": "cylindrical",
    "egg": "tripod",
    "grape": "pinch",
    "mug": "hook",
}


def load_splits(path: Path) -> Dict[str, List[Dict[str, str]]]:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def load_image(path: Path, size: Tuple[int, int]) -> np.ndarray:
    img = Image.open(path).convert("RGB")
    img = img.resize(size)
    arr = np.asarray(img, dtype=np.float32) / 255.0
    return arr.reshape(-1)


def build_dataset(
    entries: List[Dict[str, str]],
    image_size: Tuple[int, int],
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    xs: List[np.ndarray] = []
    obj_labels: List[str] = []
    grip_labels: List[str] = []
    for item in entries:
        img_path = Path(item["path"])
        obj_label = item["label"]
        grip_label = OBJECT_TO_GRIP[obj_label]
        xs.append(load_image(img_path, image_size))
        obj_labels.append(obj_label)
        grip_labels.append(grip_label)
    return np.stack(xs, axis=0), np.array(obj_labels), np.array(grip_labels)


def plot_confusion(
    title_prefix: str,
    y_true: np.ndarray,
    y_pred: np.ndarray,
    class_names: List[str],
    output_dir: Path,
    split_name: str,
    title_suffix: str = "",
) -> None:
    cm = confusion_matrix(y_true, y_pred, labels=np.arange(len(class_names)))
    cm_norm = cm.astype(np.float32) / cm.sum(axis=1, keepdims=True)

    fig, ax = plt.subplots(figsize=(8, 6))
    sns.heatmap(
        cm_norm,
        annot=True,
        fmt=".2f",
        cmap="Blues",
        xticklabels=class_names,
        yticklabels=class_names,
        cbar_kws={"label": "Normalized frequency"},
        ax=ax,
    )
    ax.set_xlabel("Predicted label")
    ax.set_ylabel("True label")
    extra = f"\n{title_suffix}" if title_suffix else ""
    ax.set_title(f"{title_prefix} - {split_name} confusion matrix{extra}")
    plt.tight_layout()
    output_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_dir / f"{title_prefix}_{split_name}_confusion.png", dpi=200)
    plt.close(fig)


def plot_precision_recall(
    title_prefix: str,
    y_true: np.ndarray,
    y_pred: np.ndarray,
    class_names: List[str],
    output_dir: Path,
    split_name: str,
    title_suffix: str = "",
) -> None:
    precision, recall, _, _ = precision_recall_fscore_support(
        y_true, y_pred, labels=np.arange(len(class_names)), zero_division=0
    )
    indices = np.arange(len(class_names))
    width = 0.35

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.bar(indices - width / 2, precision, width, label="Precision")
    ax.bar(indices + width / 2, recall, width, label="Recall")
    ax.set_xticks(indices)
    ax.set_xticklabels(class_names, rotation=30, ha="right")
    ax.set_ylabel("Score")
    ax.set_ylim(0.0, 1.05)
    extra = f"\n{title_suffix}" if title_suffix else ""
    ax.set_title(f"{title_prefix} - {split_name.capitalize()} precision/recall{extra}")
    ax.legend(loc="lower right")
    plt.tight_layout()
    output_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_dir / f"{title_prefix}_{split_name}_precision_recall.png", dpi=200)
    plt.close(fig)


def train_and_report(
    model,
    x_train: np.ndarray,
    y_train: np.ndarray,
    x_val: np.ndarray,
    y_val: np.ndarray,
    x_test: np.ndarray,
    y_test: np.ndarray,
    class_names: List[str],
    title_prefix: str,
    plots_dir: Path,
    title_suffix: str,
) -> Dict[str, Any]:
    model.fit(x_train, y_train)

    def eval_split(split_name: str, x: np.ndarray, y_true: np.ndarray) -> Dict[str, Any]:
        y_pred = model.predict(x)
        acc = accuracy_score(y_true, y_pred)
        print(f"\n{title_prefix} - {split_name} accuracy: {acc:.4f}")
        print(
            classification_report(
                y_true, y_pred, target_names=class_names, digits=4
            )
        )
        plot_confusion(
            title_prefix,
            y_true,
            y_pred,
            class_names,
            plots_dir,
            split_name,
            title_suffix=title_suffix,
        )
        plot_precision_recall(
            title_prefix,
            y_true,
            y_pred,
            class_names,
            plots_dir,
            split_name,
            title_suffix=title_suffix,
        )
        return {"accuracy": acc}

    return {
        "val": eval_split("val", x_val, y_val),
        "test": eval_split("test", x_test, y_test),
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Two-head MLP baselines (object + grip) with 5 trials."
    )
    parser.add_argument(
        "--splits",
        type=str,
        default="data/splits.json",
        help="Path to data splits JSON file.",
    )
    parser.add_argument(
        "--image-size",
        type=int,
        nargs=2,
        default=(128, 128),
        metavar=("W", "H"),
        help="Image resize dimensions.",
    )
    parser.add_argument(
        "--plots-dir",
        type=str,
        default="plots_mlp",
        help="Base directory to save per-trial plots.",
    )
    args = parser.parse_args()

    splits_path = Path(args.splits)
    if not splits_path.exists():
        raise SystemExit(f"Splits file not found: {splits_path}")

    splits = load_splits(splits_path)
    image_size = tuple(args.image_size)
    base_plots_dir = Path(args.plots_dir)
    base_plots_dir.mkdir(parents=True, exist_ok=True)

    x_train, y_train_obj_str, y_train_grip_str = build_dataset(splits["train"], image_size)
    x_val, y_val_obj_str, y_val_grip_str = build_dataset(splits["val"], image_size)
    x_test, y_test_obj_str, y_test_grip_str = build_dataset(splits["test"], image_size)

    obj_encoder = LabelEncoder()
    grip_encoder = LabelEncoder()
    y_train_obj = obj_encoder.fit_transform(y_train_obj_str)
    y_val_obj = obj_encoder.transform(y_val_obj_str)
    y_test_obj = obj_encoder.transform(y_test_obj_str)
    obj_classes = list(obj_encoder.classes_)

    y_train_grip = grip_encoder.fit_transform(y_train_grip_str)
    y_val_grip = grip_encoder.transform(y_val_grip_str)
    y_test_grip = grip_encoder.transform(y_test_grip_str)
    grip_classes = list(grip_encoder.classes_)

    trials: List[Dict[str, Any]] = [
        {
            "name": "trial_1",
            "hidden": (256, 128),
            "activation": "relu",
            "lr": 1e-3,
        },
        {
            "name": "trial_2",
            "hidden": (512, 256),
            "activation": "relu",
            "lr": 1e-3,
        },
        {
            "name": "trial_3",
            "hidden": (256, 128, 64),
            "activation": "tanh",
            "lr": 1e-3,
        },
        {
            "name": "trial_4",
            "hidden": (512, 256, 128),
            "activation": "relu",
            "lr": 5e-4,
        },
        {
            "name": "trial_5",
            "hidden": (256, 256),
            "activation": "logistic",
            "lr": 7e-4,
        },
    ]

    summary_test_acc = []

    for trial in trials:
        trial_dir = base_plots_dir / trial["name"]
        trial_dir.mkdir(parents=True, exist_ok=True)
        suffix = (
            f"hidden={trial['hidden']}, act={trial['activation']}, lr={trial['lr']}"
        )
        print(f"\n=== Running {trial['name']} ===")
        print(suffix)

        common_params = dict(
            hidden_layer_sizes=trial["hidden"],
            activation=trial["activation"],
            learning_rate_init=trial["lr"],
            solver="adam",
            max_iter=500,
            random_state=42,
            early_stopping=True,
            validation_fraction=0.1,
            n_iter_no_change=10,
        )

        obj_model = MLPClassifier(**common_params)
        grip_model = MLPClassifier(**common_params)

        obj_results = train_and_report(
            obj_model,
            x_train,
            y_train_obj,
            x_val,
            y_val_obj,
            x_test,
            y_test_obj,
            obj_classes,
            f"{trial['name']}_MLP_object",
            trial_dir,
            suffix,
        )

        grip_results = train_and_report(
            grip_model,
            x_train,
            y_train_grip,
            x_val,
            y_val_grip,
            x_test,
            y_test_grip,
            grip_classes,
            f"{trial['name']}_MLP_grip",
            trial_dir,
            suffix,
        )

        summary_test_acc.append(
            (
                trial["name"],
                obj_results["test"]["accuracy"],
                grip_results["test"]["accuracy"],
            )
        )

        print(
            f"Completed {trial['name']}: "
            f"test obj acc={obj_results['test']['accuracy']:.4f}, "
            f"test grip acc={grip_results['test']['accuracy']:.4f}"
        )

    # Summary bar plot
    trial_names = [t[0] for t in summary_test_acc]
    obj_accs = [t[1] for t in summary_test_acc]
    grip_accs = [t[2] for t in summary_test_acc]
    x_idx = np.arange(len(trial_names))
    width = 0.35

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.bar(x_idx - width / 2, obj_accs, width, label="Object head")
    ax.bar(x_idx + width / 2, grip_accs, width, label="Grip head")
    ax.set_xticks(x_idx)
    ax.set_xticklabels(trial_names, rotation=25, ha="right")
    ax.set_ylabel("Test accuracy")
    ax.set_ylim(0.0, 1.05)
    ax.set_title("MLP trials - test accuracy (object vs grip)")
    ax.legend()
    plt.tight_layout()
    fig.savefig(base_plots_dir / "mlp_trials_test_accuracy.png", dpi=200)
    plt.close(fig)

    print(f"\nAll trials complete. Plots saved under '{base_plots_dir}'.")


if __name__ == "__main__":
    main()
