import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from PIL import Image
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import (
    accuracy_score,
    classification_report,
    confusion_matrix,
    precision_recall_fscore_support,
)
from sklearn.preprocessing import LabelEncoder
from sklearn.svm import SVC


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
) -> Tuple[np.ndarray, np.ndarray]:
    xs: List[np.ndarray] = []
    ys: List[str] = []
    for item in entries:
        img_path = Path(item["path"])
        label = item["label"]
        xs.append(load_image(img_path, image_size))
        ys.append(label)
    x_arr = np.stack(xs, axis=0)
    y_arr = np.array(ys)
    return x_arr, y_arr


def plot_confusion(
    model_name: str,
    y_true: np.ndarray,
    y_pred: np.ndarray,
    class_names: List[str],
    output_dir: Path,
    split_name: str,
    title_suffix: str,
) -> None:
    """Plot a normalized confusion matrix for a given model and split."""
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
    ax.set_title(f"{model_name} - {split_name} confusion matrix\n{title_suffix}")
    plt.tight_layout()

    output_dir.mkdir(parents=True, exist_ok=True)
    out_path = output_dir / f"{model_name.replace(' ', '_').replace('(', '').replace(')', '')}_{split_name}_confusion.png"
    fig.savefig(out_path, dpi=200)
    plt.close(fig)


def plot_precision_recall(
    model_name: str,
    precision: np.ndarray,
    recall: np.ndarray,
    class_names: List[str],
    output_dir: Path,
    split_name: str,
    title_suffix: str,
) -> None:
    """Plot per-class precision and recall as grouped bars."""
    indices = np.arange(len(class_names))
    width = 0.35

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.bar(indices - width / 2, precision, width, label="Precision")
    ax.bar(indices + width / 2, recall, width, label="Recall")

    ax.set_xticks(indices)
    ax.set_xticklabels(class_names, rotation=30, ha="right")
    ax.set_ylabel("Score")
    ax.set_ylim(0.0, 1.05)
    ax.set_title(f"{model_name} - {split_name} precision and recall\n{title_suffix}")
    ax.legend(loc="lower right")
    plt.tight_layout()

    output_dir.mkdir(parents=True, exist_ok=True)
    out_path = output_dir / f"{model_name.replace(' ', '_').replace('(', '').replace(')', '')}_{split_name}_precision_recall.png"
    fig.savefig(out_path, dpi=200)
    plt.close(fig)


def train_and_evaluate(
    name: str,
    model,
    x_train: np.ndarray,
    y_train: np.ndarray,
    x_val: np.ndarray,
    y_val: np.ndarray,
    x_test: np.ndarray,
    y_test: np.ndarray,
    target_names: List[str],
    plots_dir: Path,
    params: Dict[str, Any],
) -> Dict[str, Any]:
    print(f"\n=== {name} ===")
    model.fit(x_train, y_train)

    results: Dict[str, Any] = {}

    def report_split(split_name: str, x: np.ndarray, y_true: np.ndarray) -> Dict[str, Any]:
        y_pred = model.predict(x)
        acc = accuracy_score(y_true, y_pred)
        print(f"\n{name} - {split_name} accuracy: {acc:.4f}")
        print(
            classification_report(
                y_true,
                y_pred,
                target_names=target_names,
                digits=4,
            )
        )

        precision, recall, f1, _ = precision_recall_fscore_support(
            y_true, y_pred, labels=np.arange(len(target_names))
        )

        return {
            "accuracy": acc,
            "precision": precision,
            "recall": recall,
            "f1": f1,
            "y_true": y_true,
            "y_pred": y_pred,
        }

    title_suffix = ", ".join(f"{k}={v}" for k, v in params.items())

    results["val"] = report_split("val", x_val, y_val)
    results["test"] = report_split("test", x_test, y_test)

    # Plots for validation and test splits
    for split_name in ("val", "test"):
        split_res = results[split_name]
        plot_confusion(
            model_name=name,
            y_true=split_res["y_true"],
            y_pred=split_res["y_pred"],
            class_names=target_names,
            output_dir=plots_dir,
            split_name=split_name,
            title_suffix=title_suffix,
        )
        plot_precision_recall(
            model_name=name,
            precision=split_res["precision"],
            recall=split_res["recall"],
            class_names=target_names,
            output_dir=plots_dir,
            split_name=split_name,
            title_suffix=title_suffix,
        )

    return results


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Train logistic regression and SVM baselines on image dataset."
    )
    parser.add_argument(
        "--splits",
        type=str,
        default="data/splits.json",
        help="Path to JSON file with train/val/test indices.",
    )
    parser.add_argument(
        "--image-size",
        type=int,
        nargs=2,
        default=(128, 128),
        metavar=("WIDTH", "HEIGHT"),
        help="Image size to resize to before flattening.",
    )
    parser.add_argument(
        "--c-logreg",
        type=float,
        default=1.0,
        help="Inverse regularization strength C for logistic regression.",
    )
    parser.add_argument(
        "--c-svm",
        type=float,
        default=1.0,
        help="Inverse regularization strength C for SVM.",
    )
    parser.add_argument(
        "--gamma-svm",
        type=str,
        default="scale",
        help="Kernel coefficient gamma for RBF SVM (e.g., 'scale', 'auto').",
    )
    parser.add_argument(
        "--plots-dir",
        type=str,
        default="plots",
        help="Directory to save evaluation plots.",
    )
    args = parser.parse_args()

    splits_path = Path(args.splits)
    if not splits_path.exists():
        raise SystemExit(f"Splits file not found: {splits_path}")

    splits = load_splits(splits_path)
    required_keys = {"train", "val", "test"}
    if not required_keys.issubset(splits.keys()):
        raise SystemExit(f"Splits file must contain keys: {required_keys}")

    image_size = (args.image_size[0], args.image_size[1])
    plots_dir = Path(args.plots_dir)

    x_train, y_train_str = build_dataset(splits["train"], image_size)
    x_val, y_val_str = build_dataset(splits["val"], image_size)
    x_test, y_test_str = build_dataset(splits["test"], image_size)

    label_encoder = LabelEncoder()
    y_train = label_encoder.fit_transform(y_train_str)
    y_val = label_encoder.transform(y_val_str)
    y_test = label_encoder.transform(y_test_str)
    target_names = list(label_encoder.classes_)

    logreg = LogisticRegression(
        C=args.c_logreg,
        max_iter=1000,
        n_jobs=-1,
        multi_class="auto",
        solver="lbfgs",
    )

    svm_linear = SVC(
        C=args.c_svm,
        kernel="linear",
    )

    svm_rbf = SVC(
        C=args.c_svm,
        kernel="rbf",
        gamma=args.gamma_svm,
    )

    model_results: Dict[str, Any] = {}

    model_results["Logistic Regression"] = train_and_evaluate(
        "Logistic Regression",
        logreg,
        x_train,
        y_train,
        x_val,
        y_val,
        x_test,
        y_test,
        target_names,
        plots_dir,
        params={"C_logreg": args.c_logreg},
    )

    model_results["SVM (linear kernel)"] = train_and_evaluate(
        "SVM (linear kernel)",
        svm_linear,
        x_train,
        y_train,
        x_val,
        y_val,
        x_test,
        y_test,
        target_names,
        plots_dir,
        params={"C_svm": args.c_svm, "kernel": "linear"},
    )

    model_results["SVM (RBF kernel)"] = train_and_evaluate(
        "SVM (RBF kernel)",
        svm_rbf,
        x_train,
        y_train,
        x_val,
        y_val,
        x_test,
        y_test,
        target_names,
        plots_dir,
        params={"C_svm": args.c_svm, "kernel": "rbf", "gamma": args.gamma_svm},
    )

    # Summary bar plot of test accuracy for all models
    model_names = list(model_results.keys())
    test_accuracies = [
        model_results[m]["test"]["accuracy"] for m in model_names
    ]

    fig, ax = plt.subplots(figsize=(8, 5))
    indices = np.arange(len(model_names))
    ax.bar(indices, test_accuracies, color="steelblue")
    ax.set_xticks(indices)
    ax.set_xticklabels(model_names, rotation=20, ha="right")
    ax.set_ylabel("Test accuracy")
    ax.set_ylim(0.0, 1.05)
    ax.set_title("Comparison of test accuracy across baseline models")
    plt.tight_layout()

    plots_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(plots_dir / "baseline_test_accuracy_comparison.png", dpi=200)
    plt.close(fig)


if __name__ == "__main__":
    main()
