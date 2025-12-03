"""
Two-head CNN: shared backbone + object head + grip head.
- Uses fixed mapping object -> grip (apple→spherical, bottle→cylindrical, egg→tripod, grape→pinch, mug→hook)
- Runs 5 hardcoded trials with different CNN hyperparameters.
- Saves per-trial plots under plots/trial_1 ... trial_5:
  * confusion matrices (object + grip, val + test)
  * precision/recall bars (object + grip, val + test)
  * loss curves (train/val combined loss)
  * error curves (1-accuracy for object + grip, train/val)
"""

import argparse
import json
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import torch
import torch.nn as nn
import torch.optim as optim
from PIL import Image
from sklearn.metrics import (
    classification_report,
    confusion_matrix,
    precision_recall_fscore_support,
)
from sklearn.preprocessing import LabelEncoder
from torch.utils.data import DataLoader, Dataset
from torchvision import transforms

# Fixed mapping object -> grip
OBJECT_TO_GRIP = {
    "apple": "spherical",
    "bottle": "cylindrical",
    "egg": "tripod",
    "grape": "pinch",
    "mug": "hook",
}


class ImageDataset(Dataset):
    """Dataset returning (image, object_label, grip_label)."""

    def __init__(
        self,
        image_paths: List[str],
        object_labels: np.ndarray,
        grip_labels: np.ndarray,
        transform=None,
    ):
        self.image_paths = image_paths
        self.object_labels = torch.from_numpy(object_labels).long()
        self.grip_labels = torch.from_numpy(grip_labels).long()
        self.transform = transform

    def __len__(self) -> int:
        return len(self.image_paths)

    def __getitem__(self, idx: int):
        img_path = self.image_paths[idx]
        image = Image.open(img_path).convert("RGB")
        if self.transform:
            image = self.transform(image)
        return image, self.object_labels[idx], self.grip_labels[idx]


class MultiHeadCNN(nn.Module):
    """Shared CNN backbone with two heads (object + grip)."""

    def __init__(
        self,
        num_object_classes: int,
        num_grip_classes: int,
        image_size: int,
        conv_filters: List[int],
        kernel_size: int,
        activation: str,
        head_hidden: int = 256,
    ):
        super().__init__()
        act = nn.ReLU if activation == "relu" else nn.LeakyReLU

        layers: List[nn.Module] = []
        in_ch = 3
        for out_ch in conv_filters:
            layers.append(nn.Conv2d(in_ch, out_ch, kernel_size=kernel_size, padding=1))
            layers.append(act())
            layers.append(nn.MaxPool2d(2, 2))
            in_ch = out_ch
        self.conv_stack = nn.Sequential(*layers)

        # compute flatten dim dynamically
        with torch.no_grad():
            dummy = torch.zeros(1, 3, image_size, image_size)
            feat = self.conv_stack(dummy)
            flatten_dim = feat.numel()

        def make_head(num_classes: int) -> nn.Sequential:
            return nn.Sequential(
                nn.Flatten(),
                nn.Linear(flatten_dim, head_hidden),
                act(),
                nn.Linear(head_hidden, num_classes),
            )

        self.object_head = make_head(num_object_classes)
        self.grip_head = make_head(num_grip_classes)

    def forward(self, x: torch.Tensor):
        feats = self.conv_stack(x)
        obj_logits = self.object_head(feats)
        grip_logits = self.grip_head(feats)
        return obj_logits, grip_logits


def plot_confusion(
    model_name: str,
    y_true: np.ndarray,
    y_pred: np.ndarray,
    class_names: List[str],
    output_dir: Path,
    split_name: str,
    title_suffix: str = "",
) -> None:
    """Plot normalized confusion matrix (seaborn heatmap)."""
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
    title_extra = f"\n{title_suffix}" if title_suffix else ""
    ax.set_title(f"{model_name} - {split_name} confusion matrix{title_extra}")
    plt.tight_layout()

    output_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_dir / f"{model_name}_{split_name}_confusion.png", dpi=200)
    plt.close(fig)


def plot_precision_recall(
    model_name: str,
    y_true: np.ndarray,
    y_pred: np.ndarray,
    class_names: List[str],
    output_dir: Path,
    split_name: str,
    title_suffix: str = "",
) -> None:
    """Plot per-class precision and recall as grouped bars."""
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
    title_extra = f"\n{title_suffix}" if title_suffix else ""
    ax.set_title(f"{model_name} - {split_name.capitalize()} precision/recall{title_extra}")
    ax.legend(loc="lower right")
    plt.tight_layout()

    output_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_dir / f"{model_name}_{split_name}_precision_recall.png", dpi=200)
    plt.close(fig)


def evaluate(
    model: nn.Module,
    dataloader: DataLoader,
    device: torch.device,
    criterion: nn.Module,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float, float, float]:
    """Evaluate model; return labels/preds for both heads plus loss and accuracies."""
    model.eval()
    all_obj_labels: List[int] = []
    all_obj_preds: List[int] = []
    all_grip_labels: List[int] = []
    all_grip_preds: List[int] = []
    running_loss = 0.0
    total_obj = 0
    correct_obj = 0
    total_grip = 0
    correct_grip = 0
    with torch.no_grad():
        for inputs, obj_labels, grip_labels in dataloader:
            inputs = inputs.to(device)
            obj_labels = obj_labels.to(device)
            grip_labels = grip_labels.to(device)

            obj_logits, grip_logits = model(inputs)
            loss = criterion(obj_logits, obj_labels) + criterion(grip_logits, grip_labels)
            running_loss += loss.item()

            _, obj_preds = torch.max(obj_logits, 1)
            _, grip_preds = torch.max(grip_logits, 1)

            total_obj += obj_labels.size(0)
            correct_obj += (obj_preds == obj_labels).sum().item()
            total_grip += grip_labels.size(0)
            correct_grip += (grip_preds == grip_labels).sum().item()

            all_obj_labels.extend(obj_labels.cpu().numpy())
            all_obj_preds.extend(obj_preds.cpu().numpy())
            all_grip_labels.extend(grip_labels.cpu().numpy())
            all_grip_preds.extend(grip_preds.cpu().numpy())

    avg_loss = running_loss / len(dataloader)
    obj_acc = correct_obj / total_obj if total_obj else 0.0
    grip_acc = correct_grip / total_grip if total_grip else 0.0
    return (
        np.array(all_obj_labels),
        np.array(all_obj_preds),
        np.array(all_grip_labels),
        np.array(all_grip_preds),
        avg_loss,
        obj_acc,
        grip_acc,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Train two-head CNN (object + grip).")
    parser.add_argument(
        "--splits",
        type=str,
        default="data/splits.json",
        help="Path to JSON file with train/val/test splits.",
    )
    parser.add_argument(
        "--image-size",
        type=int,
        default=128,
        help="Image size (width and height).",
    )
    parser.add_argument(
        "--plots-dir", type=str, default="plots_cnn", help="Directory to save plots."
    )
    parser.add_argument("--epochs", type=int, default=20, help="Number of epochs.")
    parser.add_argument("--batch-size", type=int, default=32, help="Batch size.")
    args = parser.parse_args()

    # Predefined 5 trials
    trial_configs: List[Dict[str, object]] = [
        {
            "name": "trial_1",
            "conv_filters": [16, 32, 64],
            "kernel_size": 3,
            "activation": "relu",
            "lr": 1e-3,
        },
        {
            "name": "trial_2",
            "conv_filters": [32, 64, 128],
            "kernel_size": 3,
            "activation": "relu",
            "lr": 7e-4,
        },
        {
            "name": "trial_3",
            "conv_filters": [32, 64, 128],
            "kernel_size": 5,
            "activation": "leaky_relu",
            "lr": 7e-4,
        },
        {
            "name": "trial_4",
            "conv_filters": [16, 32, 64, 128],
            "kernel_size": 3,
            "activation": "relu",
            "lr": 5e-4,
        },
        {
            "name": "trial_5",
            "conv_filters": [32, 64, 128, 128],
            "kernel_size": 3,
            "activation": "leaky_relu",
            "lr": 5e-4,
        },
    ]

    base_plots_dir = Path(args.plots_dir)
    base_plots_dir.mkdir(parents=True, exist_ok=True)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # Load splits
    with open(args.splits, "r") as f:
        splits = json.load(f)

    # Encoders
    object_labels_train = [item["label"] for item in splits["train"]]
    object_encoder = LabelEncoder()
    object_encoder.fit(object_labels_train)
    object_classes = list(object_encoder.classes_)
    num_object_classes = len(object_classes)

    grip_labels_train = [OBJECT_TO_GRIP[label] for label in object_labels_train]
    grip_encoder = LabelEncoder()
    grip_encoder.fit(grip_labels_train)
    grip_classes = list(grip_encoder.classes_)
    num_grip_classes = len(grip_classes)

    def encode_split(split_key: str):
        obj_labels = [item["label"] for item in splits[split_key]]
        grip_labels = [OBJECT_TO_GRIP[label] for label in obj_labels]
        return (
            object_encoder.transform(obj_labels),
            grip_encoder.transform(grip_labels),
            [item["path"] for item in splits[split_key]],
        )

    y_train_obj, y_train_grip, X_train_paths = encode_split("train")
    y_val_obj, y_val_grip, X_val_paths = encode_split("val")
    y_test_obj, y_test_grip, X_test_paths = encode_split("test")

    transform = transforms.Compose(
        [
            transforms.Resize((args.image_size, args.image_size)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ]
    )

    train_dataset = ImageDataset(X_train_paths, y_train_obj, y_train_grip, transform)
    val_dataset = ImageDataset(X_val_paths, y_val_obj, y_val_grip, transform)
    test_dataset = ImageDataset(X_test_paths, y_test_obj, y_test_grip, transform)

    for cfg in trial_configs:
        trial_name = cfg["name"]
        trial_dir = base_plots_dir / trial_name
        trial_dir.mkdir(parents=True, exist_ok=True)

        print(f"\n=== Training two-head CNN ({trial_name}) ===")
        print(
            f"conv_filters={cfg['conv_filters']}, k={cfg['kernel_size']}, "
            f"activation={cfg['activation']}, lr={cfg['lr']}"
        )

        train_loader = DataLoader(
            train_dataset, batch_size=args.batch_size, shuffle=True, num_workers=2
        )
        val_loader = DataLoader(
            val_dataset, batch_size=args.batch_size, shuffle=False, num_workers=2
        )
        test_loader = DataLoader(
            test_dataset, batch_size=args.batch_size, shuffle=False, num_workers=2
        )

        model = MultiHeadCNN(
            num_object_classes=num_object_classes,
            num_grip_classes=num_grip_classes,
            image_size=args.image_size,
            conv_filters=cfg["conv_filters"],  # type: ignore[arg-type]
            kernel_size=cfg["kernel_size"],  # type: ignore[arg-type]
            activation=cfg["activation"],  # type: ignore[arg-type]
        ).to(device)

        criterion = nn.CrossEntropyLoss()
        optimizer = optim.Adam(model.parameters(), lr=cfg["lr"])  # type: ignore[arg-type]

        train_losses: List[float] = []
        val_losses: List[float] = []
        train_obj_errors: List[float] = []
        train_grip_errors: List[float] = []
        val_obj_errors: List[float] = []
        val_grip_errors: List[float] = []

        for epoch in range(args.epochs):
            model.train()
            running_loss = 0.0
            total_obj = 0
            correct_obj = 0
            total_grip = 0
            correct_grip = 0

            for inputs, obj_labels, grip_labels in train_loader:
                inputs = inputs.to(device)
                obj_labels = obj_labels.to(device)
                grip_labels = grip_labels.to(device)

                optimizer.zero_grad()
                obj_logits, grip_logits = model(inputs)
                loss_obj = criterion(obj_logits, obj_labels)
                loss_grip = criterion(grip_logits, grip_labels)
                loss = loss_obj + loss_grip
                loss.backward()
                optimizer.step()

                running_loss += loss.item()
                _, obj_preds = torch.max(obj_logits, 1)
                _, grip_preds = torch.max(grip_logits, 1)
                total_obj += obj_labels.size(0)
                correct_obj += (obj_preds == obj_labels).sum().item()
                total_grip += grip_labels.size(0)
                correct_grip += (grip_preds == grip_labels).sum().item()

            train_loss = running_loss / len(train_loader)
            train_obj_acc = correct_obj / total_obj if total_obj else 0.0
            train_grip_acc = correct_grip / total_grip if total_grip else 0.0

            (
                val_obj_labels,
                val_obj_preds,
                val_grip_labels,
                val_grip_preds,
                val_loss,
                val_obj_acc,
                val_grip_acc,
            ) = evaluate(model, val_loader, device, criterion)

            train_losses.append(train_loss)
            val_losses.append(val_loss)
            train_obj_errors.append(1.0 - train_obj_acc)
            train_grip_errors.append(1.0 - train_grip_acc)
            val_obj_errors.append(1.0 - val_obj_acc)
            val_grip_errors.append(1.0 - val_grip_acc)

            print(
                f"Epoch {epoch + 1}/{args.epochs} | "
                f"Train Loss: {train_loss:.4f} | "
                f"Train Obj Acc: {train_obj_acc:.4f} | Train Grip Acc: {train_grip_acc:.4f} | "
                f"Val Loss: {val_loss:.4f} | "
                f"Val Obj Acc: {val_obj_acc:.4f} | Val Grip Acc: {val_grip_acc:.4f}"
            )

        # Training curves
        epochs_range = np.arange(1, args.epochs + 1)
        plt.figure(figsize=(8, 5))
        plt.plot(epochs_range, train_losses, label="Train loss")
        plt.plot(epochs_range, val_losses, label="Val loss")
        plt.xlabel("Epoch")
        plt.ylabel("Loss")
        plt.title(f"{trial_name} - CNN two-head loss")
        plt.legend()
        plt.tight_layout()
        plt.savefig(trial_dir / "CNN_loss_curves.png", dpi=200)
        plt.close()

        plt.figure(figsize=(9, 5))
        plt.plot(epochs_range, train_obj_errors, label="Train error (object)")
        plt.plot(epochs_range, val_obj_errors, label="Val error (object)")
        plt.plot(epochs_range, train_grip_errors, label="Train error (grip)")
        plt.plot(epochs_range, val_grip_errors, label="Val error (grip)")
        plt.xlabel("Epoch")
        plt.ylabel("Error rate (1 - accuracy)")
        plt.title(f"{trial_name} - CNN two-head error rates")
        plt.legend()
        plt.tight_layout()
        plt.savefig(trial_dir / "CNN_error_curves.png", dpi=200)
        plt.close()

        cfg_suffix = (
            f"filters={cfg['conv_filters']}, k={cfg['kernel_size']}, "
            f"act={cfg['activation']}, lr={cfg['lr']}"
        )

        # Validation reports and plots
        print("\n--- Validation (object head) ---")
        print(
            classification_report(
                val_obj_labels,
                val_obj_preds,
                target_names=object_classes,
                digits=4,
            )
        )
        plot_confusion(
            f"{trial_name}_CNN_object",
            val_obj_labels,
            val_obj_preds,
            object_classes,
            trial_dir,
            "val",
            title_suffix=cfg_suffix,
        )
        plot_precision_recall(
            f"{trial_name}_CNN_object",
            val_obj_labels,
            val_obj_preds,
            object_classes,
            trial_dir,
            "val",
            title_suffix=cfg_suffix,
        )

        print("\n--- Validation (grip head) ---")
        print(
            classification_report(
                val_grip_labels,
                val_grip_preds,
                target_names=grip_classes,
                digits=4,
            )
        )
        plot_confusion(
            f"{trial_name}_CNN_grip",
            val_grip_labels,
            val_grip_preds,
            grip_classes,
            trial_dir,
            "val",
            title_suffix=cfg_suffix,
        )
        plot_precision_recall(
            f"{trial_name}_CNN_grip",
            val_grip_labels,
            val_grip_preds,
            grip_classes,
            trial_dir,
            "val",
            title_suffix=cfg_suffix,
        )

        # Test
        print("\n--- Test (both heads) ---")
        (
            test_obj_labels,
            test_obj_preds,
            test_grip_labels,
            test_grip_preds,
            _,
            _,
            _,
        ) = evaluate(model, test_loader, device, criterion)

        print("\n[Test] Object head")
        print(
            classification_report(
                test_obj_labels,
                test_obj_preds,
                target_names=object_classes,
                digits=4,
            )
        )
        plot_confusion(
            f"{trial_name}_CNN_object",
            test_obj_labels,
            test_obj_preds,
            object_classes,
            trial_dir,
            "test",
            title_suffix=cfg_suffix,
        )
        plot_precision_recall(
            f"{trial_name}_CNN_object",
            test_obj_labels,
            test_obj_preds,
            object_classes,
            trial_dir,
            "test",
            title_suffix=cfg_suffix,
        )

        print("\n[Test] Grip head")
        print(
            classification_report(
                test_grip_labels,
                test_grip_preds,
                target_names=grip_classes,
                digits=4,
            )
        )
        plot_confusion(
            f"{trial_name}_CNN_grip",
            test_grip_labels,
            test_grip_preds,
            grip_classes,
            trial_dir,
            "test",
            title_suffix=cfg_suffix,
        )
        plot_precision_recall(
            f"{trial_name}_CNN_grip",
            test_grip_labels,
            test_grip_preds,
            grip_classes,
            trial_dir,
            "test",
            title_suffix=cfg_suffix,
        )

        print(f"Completed {trial_name}, plots in {trial_dir}")

    print(f"\nAll trials complete. Plots saved under '{base_plots_dir}'.")


if __name__ == "__main__":
    main()
