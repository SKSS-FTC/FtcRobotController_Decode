#!/usr/bin/env python3
"""
Simple YOLO26 INT8 TFLite Export Script (KISS Principle)

Exports YOLO26 models to INT8 quantized TFLite format using local COCO dataset
for calibration. Works with Ultralytics >= 8.4.14.

Usage:
    python export_int8.py --model yolo26n.pt --output ../TeamCode/src/main/assets/
"""

import argparse
import sys
from pathlib import Path

def export_int8(
    model_path_str: str,
    output_dir_str: str,
    imgsz: int = 640,
    data_path_override: str | None = None,
    batch: int = 1,
    fraction: float = 0.02,
    device: str = "0",
    export_fp16: bool = False,
):
    """
    Export YOLO26 model to (quantized) TFLite.

    Can perform INT8 calibration using a COCO-style dataset and/or
    produce an FP16 model in the same run.
    
    Args:
        model_path_str: Path to YOLO26 detection .pt model file
        output_dir_str: Directory to save the exported .tflite file
        imgsz: Input image size (default: 640)
        data_path_override: Optional dataset YAML path for INT8 calibration
        batch: Calibration/export batch size (default: 1)
        fraction: Fraction of calibration set to use (default: 0.02)
        device: Device for export (default: 0)
        export_fp16: If True, also export a float16 TFLite model
    """
    try:
        from ultralytics import YOLO
    except ImportError:
        print("Error: ultralytics not installed. Run: pip install ultralytics>=8.4.14")
        sys.exit(1)
    
    model_path = Path(model_path_str)
    output_dir = Path(output_dir_str)
    
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Load model (Ultralytics auto-downloads if not present)
    if model_path.exists():
        print(f"Loading local model: {model_path}")
        model = YOLO(str(model_path))
    else:
        # Model will be auto-downloaded by Ultralytics
        model_name = model_path.stem  # e.g., "yolo26n"
        print(f"Model not found locally. Auto-downloading: {model_name}")
        print(f"  (Ultralytics will download from https://github.com/ultralytics/assets)")
        model = YOLO(model_name)

    base_name = model_path.stem.replace("-seg", "").replace("_seg", "")
    
    # Export to INT8 TFLite
    # Using COCO val2017 for calibration (need data.yaml)
    # Determine modes to export
    modes = []
    if export_fp16:
        modes.append("FP16")
    modes.append("INT8")

    print(
        f"Exporting to {' + '.join(modes)} TFLite "
        f"(imgsz={imgsz}, batch={batch}, fraction={fraction}, device={device}, end2end=True)..."
    )
    if "INT8" in modes:
        print("Note: INT8 quantization requires a calibration dataset.")
        print("Use --data to override calibration dataset YAML if needed.")
    
    try:
        # prepare dataset path once if INT8 is requested
        data_path = None
        if "INT8" in modes:
            if data_path_override:
                data_path = data_path_override
            else:
                coco_data_yaml = Path("data/coco/coco.yaml")
                if coco_data_yaml.exists():
                    data_path = str(coco_data_yaml)
                else:
                    # Fallback to Ultralytics sample dataset config
                    data_path = "coco8.yaml"

        # Carry out exports independently so both can be produced
        if "INT8" in modes:
            model.export(
                format="tflite",
                int8=True,
                data=data_path,  # Uses local COCO dataset for calibration
                imgsz=imgsz,
                batch=batch,
                fraction=fraction,
                device=device,
                end2end=True,
            )

            # Find and copy INT8 output
            export_path = model_path.parent / f"{model_path.stem}_int8.tflite"
            if not export_path.exists():
                # Try alternative naming
                export_path = model_path.parent / f"{model_path.stem}_saved_model" / f"{model_path.stem}_int8.tflite"

            if export_path.exists():
                output_name = f"{base_name}_int8.tflite"
                output_path = output_dir / output_name
                import shutil
                shutil.copy2(export_path, output_path)
                print(f"✓ INT8 export successful!")
                print(f"  Source: {export_path}")
                print(f"  Saved to: {output_path}")
                print(f"  Size: {output_path.stat().st_size / (1024*1024):.2f} MB")
            else:
                print(f"⚠ INT8 export may have succeeded but couldn't find output file")
                print(f"  Looked for: {export_path}")

        if export_fp16:
            model.export(
                format="tflite",
                half=True,
                imgsz=imgsz,
                device=device,
                end2end=True,
            )

            # Locate fp16 file
            fp16_path = model_path.parent / f"{model_path.stem}_fp16.tflite"
            if not fp16_path.exists():
                fp16_path = model_path.parent / f"{model_path.stem}_saved_model" / f"{model_path.stem}_fp16.tflite"

            if fp16_path.exists():
                output_name = f"{base_name}_fp16.tflite"
                output_path = output_dir / output_name
                import shutil
                shutil.copy2(fp16_path, output_path)
                print(f"✓ FP16 export successful!")
                print(f"  Source: {fp16_path}")
                print(f"  Saved to: {output_path}")
                print(f"  Size: {output_path.stat().st_size / (1024*1024):.2f} MB")
            else:
                print(f"⚠ FP16 export may have succeeded but couldn't find output file")
                print(f"  Looked for: {fp16_path}")
    except Exception as e:
        print(f"✗ Export failed: {e}")
        print("\nTroubleshooting:")
        print("  1. Ensure COCO dataset is downloaded and labels are available")
        print("  2. Check ultralytics version: pip show ultralytics (need >= 8.4.14)")
        print("  3. Reduce calibration pressure if needed: --batch 1 --fraction 0.01")
        print("  4. For segmentation models, use export_int8_seg.py")
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(
        description="Export YOLO26 detection models to INT8 TFLite",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Export detection model
  python export_int8.py --model yolo26n.pt
  
  # Export to specific directory
  python export_int8.py --model yolo26n.pt --output ./models/

  # Export both INT8 and FP16
  python export_int8.py --model yolo26n.pt --fp16
        """
    )
    
    parser.add_argument(
        "--model", "-m",
        required=True,
        help="Path to YOLO26 detection .pt model (e.g., yolo26n.pt)"
    )
    
    parser.add_argument(
        "--output", "-o",
        default="../TeamCode/src/main/assets/",
        help="Output directory for .tflite file (default: ../TeamCode/src/main/assets/)"
    )
    
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="Input image size (default: 640)"
    )

    parser.add_argument(
        "--data",
        default=None,
        help="Calibration dataset YAML path (default: auto-detect data/coco/coco.yaml, else coco8.yaml)"
    )

    parser.add_argument(
        "--batch",
        type=int,
        default=1,
        help="Calibration/export batch size (default: 1)"
    )

    parser.add_argument(
        "--fraction",
        type=float,
        default=0.02,
        help="Fraction of calibration set to use in INT8 export (default: 0.02)"
    )

    parser.add_argument(
        "--device",
        default="0",
        help="Export device (default: 0, e.g. 0 or cpu)"
    )

    parser.add_argument(
        "--fp16",
        action="store_true",
        help="Also export a float16 TFLite model alongside INT8"
    )

    args = parser.parse_args()
    
    export_int8(
        args.model,
        args.output,
        args.imgsz,
        args.data,
        args.batch,
        args.fraction,
        args.device,
        args.fp16,
    )


if __name__ == "__main__":
    main()
