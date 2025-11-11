# BananaHand

## YOLO-based object detection

This folder contains a YOLOv8-based object detection pipeline. The script processes all images in the `input_images/` folder, runs detection to identify objects, draws bounding boxes and labels, and saves annotated results to `output_images/`.

### Setup

1. Create the conda environment named `capstone`:

```bash
conda env create -f environment.yml
conda activate capstone
```

Or manually install dependencies:

```bash
conda create -n capstone python=3.10 -y
conda activate capstone
pip install -r requirements.txt
```

### Usage

1. Place images in the `input_images/` folder.
2. From the `perception/` directory, run:

```bash
python scripts/detect.py
```

3. Annotated images (with bounding boxes and class labels) will be saved to `output_images/`.

### Configuration

Edit the `get_config()` function in `scripts/detect.py` to adjust:
- `model`: YOLO model variant (default: `yolov8n.pt` — nano, fast; also try `yolov8s.pt`, `yolov8m.pt` for better accuracy)
- `conf`: confidence threshold (default: 0.25)
- `imgsz`: inference image size (default: 640)

### Notes

- The YOLOv8 model weights (`yolov8n.pt`) are downloaded automatically on first run.
- Supported image formats: `.jpg`, `.jpeg`, `.png`, `.bmp`, `.tiff`
- The script will create `output_images/` automatically if it doesn't exist.