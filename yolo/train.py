#!/usr/bin/env python3
"""
YOLO26n Detection Model Training Script

This script trains a YOLO26n (nano) detection model on the composite dataset
for detecting green and purple balls in FTC DECODE game.

Usage:
    python yolo/train.py [--epochs EPOCHS] [--batch BATCH_SIZE] [--imgsz IMAGE_SIZE]

Example:
    python yolo/train.py --epochs 100 --batch 16 --imgsz 640
"""

import argparse
import os
from pathlib import Path

from ultralytics import YOLO


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Train YOLO26n detection model on composite dataset"
    )
    parser.add_argument(
        "--model",
        type=str,
        default="yolo26n.pt",
        help="Base model to use for training (default: yolo26n.pt)",
    )
    parser.add_argument(
        "--data",
        type=str,
        default=None,
        help="Path to dataset.yaml (default: composite_dataset/dataset.yaml)",
    )
    parser.add_argument(
        "--epochs",
        type=int,
        default=300,
        help="Number of training epochs (default: 100)",
    )
    parser.add_argument(
        "--batch",
        type=int,
        default=32,
        help="Batch size for training (default: 16)",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="Image size for training (default: 640)",
    )
    parser.add_argument(
        "--device",
        type=str,
        default=None,
        help="Device to use (e.g., '0' for GPU, 'cpu' for CPU, default: auto)",
    )
    parser.add_argument(
        "--project",
        type=str,
        default="runs/detect",
        help="Project directory for saving results (default: runs/detect, relative to repo root)",
    )
    parser.add_argument(
        "--name",
        type=str,
        default="yolo26n_composite",
        help="Experiment name (default: yolo26n_composite)",
    )
    parser.add_argument(
        "--patience",
        type=int,
        default=50,
        help="Early stopping patience (epochs without improvement, default: 50)",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=8,
        help="Number of data loader workers (default: 8)",
    )
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Resume training from last checkpoint",
    )
    return parser.parse_args()


def main():
    """Main training function."""
    args = parse_args()

    # Determine project root
    script_dir = Path(__file__).parent
    project_root = script_dir.parent

    # force the project directory to reside under the repo root unless an
    # absolute path was given by the user.  this prevents training from
    # scattering results in whatever cwd the script is executed from.
    proj_path = Path(args.project)
    if not proj_path.is_absolute():
        args.project = str(project_root / proj_path)
    else:
        args.project = str(proj_path)

    # Set default data path if not provided (relative to repo root)
    if args.data is None:
        args.data = str(project_root / "composite_dataset" / "data.yaml")

    # Resolve data path to absolute if necessary.  This ensures that a
    # relative path is interpreted relative to the repository root rather
    # than the current working directory, which matches how we handle
    # project paths above.
    data_path = Path(args.data)
    if not data_path.is_absolute():
        args.data = str(project_root / data_path)

    # Check if dataset exists
    if not os.path.exists(args.data):
        raise FileNotFoundError(f"Dataset config not found: {args.data}")

    # Check if model file exists (if not using pretrained from Ultralytics)
    model_path = project_root / args.model
    if model_path.exists():
        model_source = str(model_path)
        print(f"Using local model: {model_source}")
    else:
        model_source = args.model
        print(f"Using model from Ultralytics: {model_source}")

    # Load the model
    print(f"\n{'=' * 60}")
    print("YOLO26n Detection Model Training")
    print(f"{'=' * 60}")
    print(f"  Model:       {model_source}")
    print(f"  Dataset:     {args.data}")
    print(f"  Epochs:      {args.epochs}")
    print(f"  Batch size:  {args.batch}")
    print(f"  Image size:  {args.imgsz}")
    print(f"  Device:      {args.device or 'auto'}")
    print(f"  Project:     {args.project}")
    print(f"  Name:        {args.name}")
    print(f"{'=' * 60}\n")

    # Load YOLO model
    model = YOLO(model_source)

    # Train the model
    results = model.train(
        data=args.data,
        epochs=args.epochs,
        batch=args.batch,
        imgsz=args.imgsz,
        device=args.device,
        project=args.project,
        name=args.name,
        patience=args.patience,
        workers=args.workers,
        resume=args.resume,
    )

    # Print training summary
    print(f"\n{'=' * 60}")
    print("Training Complete!")
    print(f"{'=' * 60}")
    print(f"  Results saved to: {args.project}/{args.name}")
    print(f"  Best model: {args.project}/{args.name}/weights/best.pt")
    print(f"  Last model: {args.project}/{args.name}/weights/last.pt")
    print(f"{'=' * 60}\n")

    # Validate the model
    print("Running validation on best model...")
    best_model = YOLO(f"{args.project}/{args.name}/weights/best.pt")
    val_results = best_model.val(data=args.data)

    print(f"\nValidation Results:")
    print(f"  mAP@50:    {val_results.box.map50:.4f}")
    print(f"  mAP@50-95: {val_results.box.map:.4f}")
    print(f"  Precision: {val_results.box.mp:.4f}")
    print(f"  Recall:    {val_results.box.mr:.4f}")

    return results


if __name__ == "__main__":
    main()
