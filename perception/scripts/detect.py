#!/usr/bin/env python3
"""
Simple YOLOv8 detection script.

Usage:
    python scripts/detect.py

This script processes all images in the input_images/ folder, runs YOLOv8 detection on each,
and saves annotated results to output_images/. The model `yolov8n.pt` will be downloaded automatically.
"""
import os
import cv2
from pathlib import Path
from ultralytics import YOLO


def get_config():
    """Return configuration for detection."""
    # Get the parent directory of the script (perception/ folder)
    script_dir = Path(__file__).parent.parent
    
    return {
        "input_dir": script_dir / "input_images",
        "output_dir": script_dir / "output_images",
        "model": script_dir / "yolov8n.pt",
        "conf": 0.25,
        "imgsz": 640,
    }


def main():
    cfg = get_config()
    input_dir = Path(cfg["input_dir"])
    output_dir = Path(cfg["output_dir"])

    # Create output directory if it doesn't exist
    output_dir.mkdir(parents=True, exist_ok=True)

    # Load model (will download weights if not present)
    print(f"Loading model {cfg['model']}...")
    model = YOLO(cfg["model"])

    # Find all image files in input directory
    image_extensions = {".jpg", ".jpeg", ".png", ".bmp", ".tiff"}
    image_files = [f for f in input_dir.iterdir() if f.suffix.lower() in image_extensions]

    if not image_files:
        print(f"No images found in {input_dir}")
        return

    print(f"Found {len(image_files)} image(s) in {input_dir}. Processing...")

    # Process each image
    for idx, image_path in enumerate(image_files, 1):
        print(f"  [{idx}/{len(image_files)}] Processing {image_path.name}...")
        
        # Run prediction
        results = model.predict(source=str(image_path), imgsz=cfg["imgsz"], conf=cfg["conf"], verbose=False)
        if len(results) == 0:
            print(f"    No results returned for {image_path.name}.")
            continue

        res = results[0]

        # res.plot() returns an annotated RGB image (numpy.ndarray)
        try:
            annotated = res.plot()
        except Exception as e:
            print(f"    Could not annotate {image_path.name}: {e}")
            continue

        # Convert RGB to BGR for OpenCV and save
        bgr = cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR)
        output_path = output_dir / image_path.name
        cv2.imwrite(str(output_path), bgr)
        print(f"    Saved to {output_path}")

    print(f"Done! All results saved to {output_dir}")


if __name__ == '__main__':
    main()
