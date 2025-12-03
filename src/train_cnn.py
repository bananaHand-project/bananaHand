"""
Trains and evaluates a Convolutional Neural Network (CNN) for image classification.

This script uses a pre-defined data split from a JSON file to train a CNN model
using PyTorch, evaluates its performance on validation and test sets, and saves
classification metrics and plots. The model is trained on the GPU if available.
"""

import argparse
import json
import os
from pathlib import Path
from typing import Any, Dict, List, Tuple

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


class ImageDataset(Dataset):
    """PyTorch Dataset for loading images from a list of file paths."""

    def __init__(self, image_paths: List[str], labels: np.ndarray, transform=None):
        self.image_paths = image_paths
        self.labels = torch.from_numpy(labels).long()
        self.transform = transform

    def __len__(self) -> int:
        return len(self.image_paths)

    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor]:
        img_path = self.image_paths[idx]
        image = Image.open(img_path).convert("RGB")
        if self.transform:
            image = self.transform(image)
        label = self.labels[idx]
        return image, label


class SimpleCNN(nn.Module):
    """A simple CNN for image classification."""

    def __init__(self, num_classes: int):
        super(SimpleCNN, self).__init__()
        self.conv_stack = nn.Sequential(
            nn.Conv2d(3, 16, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2, 2),
            nn.Conv2d(16, 32, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2, 2),
            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2, 2),
        )
        # After 3 max-pools of 2x2, a 128x128 image becomes 16x16
        # 16 * 16 * 64 channels
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(64 * 16 * 16, 512),
            nn.ReLU(),
            nn.Linear(512, num_classes),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.conv_stack(x)
        x = self.classifier(x)
        return x


def plot_confusion(
    model_name: str,
    y_true: np.ndarray,
    y_pred: np.ndarray,
    class_names: List[str],
    output_dir: Path,
    split_name: str,
) -> None:
    """Plot a normalized confusion matrix."""
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
    ax.set_title(f"{model_name} - {split_name} confusion matrix")
    plt.tight_layout()

    output_dir.mkdir(parents=True, exist_ok=True)
    out_path = output_dir / f"{model_name}_{split_name}_confusion.png"
    fig.savefig(out_path, dpi=200)
    plt.close(fig)


def plot_precision_recall(
    model_name: str,
    y_true: np.ndarray,
    y_pred: np.ndarray,
    class_names: List[str],
    output_dir: Path,
    split_name: str,
) -> None:
    """Plot per-class precision and recall as grouped bars."""
    precision, recall, _, _ = precision_recall_fscore_support(
        y_true,
        y_pred, labels=np.arange(len(class_names)), zero_division=0
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
    ax.set_title(f"{model_name} - {split_name.capitalize()} Precision and Recall")
    ax.legend(loc="lower right")
    plt.tight_layout()

    output_dir.mkdir(parents=True, exist_ok=True)
    out_path = output_dir / f"{model_name}_{split_name}_precision_recall.png"
    fig.savefig(out_path, dpi=200)
    plt.close(fig)


def evaluate(
    model: nn.Module,
    dataloader: DataLoader,
    device: torch.device,
    criterion: nn.Module = None,
) -> Tuple[np.ndarray, np.ndarray, float, float]:
    """Evaluate the model on a given dataset, returning labels, preds, loss, acc."""
    model.eval()
    all_preds: List[int] = []
    all_labels: List[int] = []
    running_loss = 0.0
    total = 0
    correct = 0
    with torch.no_grad():
        for inputs, labels in dataloader:
            inputs, labels = inputs.to(device), labels.to(device)
            outputs = model(inputs)
            _, preds = torch.max(outputs, 1)
            if criterion is not None:
                loss = criterion(outputs, labels)
                running_loss += loss.item()
            total += labels.size(0)
            correct += (preds == labels).sum().item()
            all_preds.extend(preds.cpu().numpy())
            all_labels.extend(labels.cpu().numpy())
    avg_loss = running_loss / len(dataloader) if criterion is not None else 0.0
    acc = correct / total if total > 0 else 0.0
    return np.array(all_labels), np.array(all_preds), avg_loss, acc

def main() -> None:
    """Main function to train and evaluate the CNN."""
    parser = argparse.ArgumentParser(description="Train a CNN on the image dataset.")
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
        "--plots-dir", type=str, default="plots", help="Directory to save plots."
    )
    parser.add_argument("--lr", type=float, default=0.001, help="Learning rate.")
    parser.add_argument("--epochs", type=int, default=25, help="Number of epochs.")
    parser.add_argument("--batch-size", type=int, default=32, help="Batch size.")
    args = parser.parse_args()

    # --- Setup ---
    plots_dir = Path(args.plots_dir)
    plots_dir.mkdir(parents=True, exist_ok=True)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # --- Load Data Splits ---
    with open(args.splits, "r") as f:
        splits = json.load(f)

    # --- Encode Labels ---
    all_labels_str = [item["label"] for item in splits["train"]]
    label_encoder = LabelEncoder()
    label_encoder.fit(all_labels_str)
    class_names = list(label_encoder.classes_)
    num_classes = len(class_names)

    y_train = label_encoder.transform([item["label"] for item in splits["train"]])
    y_val = label_encoder.transform([item["label"] for item in splits["val"]])
    y_test = label_encoder.transform([item["label"] for item in splits["test"]])

    X_train_paths = [item["path"] for item in splits["train"]]
    X_val_paths = [item["path"] for item in splits["val"]]
    X_test_paths = [item["path"] for item in splits["test"]]

    # --- Create Datasets and DataLoaders ---
    transform = transforms.Compose(
        [
            transforms.Resize((args.image_size, args.image_size)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ]
    )

    train_dataset = ImageDataset(X_train_paths, y_train, transform)
    val_dataset = ImageDataset(X_val_paths, y_val, transform)
    test_dataset = ImageDataset(X_test_paths, y_test, transform)

    train_loader = DataLoader(
        train_dataset, batch_size=args.batch_size, shuffle=True, num_workers=4
    )
    val_loader = DataLoader(val_dataset, batch_size=args.batch_size, num_workers=4)
    test_loader = DataLoader(test_dataset, batch_size=args.batch_size, num_workers=4)

    # --- Model, Loss, Optimizer ---
    model = SimpleCNN(num_classes=num_classes).to(device)
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=args.lr)

    # --- Training Loop ---
    print("\n=== Training CNN ===")
    train_losses: List[float] = []
    val_losses: List[float] = []
    train_errors: List[float] = []
    val_errors: List[float] = []

    for epoch in range(args.epochs):
        model.train()
        running_loss = 0.0
        total = 0
        correct = 0
        for inputs, labels in train_loader:
            inputs, labels = inputs.to(device), labels.to(device)

            optimizer.zero_grad()
            outputs = model(inputs)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            running_loss += loss.item()
            _, preds = torch.max(outputs, 1)
            total += labels.size(0)
            correct += (preds == labels).sum().item()

        train_loss = running_loss / len(train_loader)
        train_acc = correct / total if total > 0 else 0.0

        val_labels, val_preds, val_loss, val_acc = evaluate(
            model, val_loader, device, criterion
        )

        train_losses.append(train_loss)
        val_losses.append(val_loss)
        train_errors.append(1.0 - train_acc)
        val_errors.append(1.0 - val_acc)

        print(
            f"Epoch {epoch + 1}/{args.epochs} | "
            f"Train Loss: {train_loss:.4f} | Train Acc: {train_acc:.4f} | "
            f"Val Loss: {val_loss:.4f} | Val Acc: {val_acc:.4f}"
        )

    # --- Training Curves ---
    epochs_range = np.arange(1, args.epochs + 1)

    # Loss curves
    plt.figure(figsize=(8, 5))
    plt.plot(epochs_range, train_losses, label="Train loss")
    plt.plot(epochs_range, val_losses, label="Val loss")
    plt.xlabel("Epoch")
    plt.ylabel("Loss")
    plt.title("CNN training/validation loss")
    plt.legend()
    plt.tight_layout()
    plt.savefig(plots_dir / "CNN_loss_curves.png", dpi=200)
    plt.close()

    # Error rate curves (1 - accuracy)
    plt.figure(figsize=(8, 5))
    plt.plot(epochs_range, train_errors, label="Train error (1-acc)")
    plt.plot(epochs_range, val_errors, label="Val error (1-acc)")
    plt.xlabel("Epoch")
    plt.ylabel("Error rate")
    plt.title("CNN training/validation error rate")
    plt.legend()
    plt.tight_layout()
    plt.savefig(plots_dir / "CNN_error_curves.png", dpi=200)
    plt.close()

    # --- Final Evaluation ---
    print("\n--- Evaluating on Validation Set ---")
    val_labels, val_preds, _, _ = evaluate(model, val_loader, device, criterion)
    print(
        classification_report(
            val_labels, val_preds, target_names=class_names, digits=4
        )
    )
    plot_confusion("CNN", val_labels, val_preds, class_names, plots_dir, "val")
    plot_precision_recall(
        "CNN", val_labels, val_preds, class_names, plots_dir, "val"
    )

    print("\n--- Evaluating on Test Set ---")
    test_labels, test_preds, _, _ = evaluate(model, test_loader, device, criterion)
    print(
        classification_report(
            test_labels, test_preds, target_names=class_names, digits=4
        )
    )
    plot_confusion("CNN", test_labels, test_preds, class_names, plots_dir, "test")
    plot_precision_recall(
        "CNN", test_labels, test_preds, class_names, plots_dir, "test"
    )

    print(f"\nCNN evaluation complete. Plots saved to '{plots_dir}'.")


if __name__ == "__main__":
    main()
