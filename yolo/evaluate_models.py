#!/usr/bin/env python3
"""Utility to evaluate YOLO26 models (FP32/PT, FP16-TFLite, INT8-TFLite)

This script leverages the Ultralytics `YOLO` class which supports
loading TFLite models directly. It runs validation on the specified
"data" dataset using the CPU, reporting the usual metrics (mAP, etc.).

Usage examples:

    # evaluate baseline pt only
    python evaluate_models.py --model runs/detect/.../weights/best.pt \
        --data ./composite_dataset/data.yaml

    # evaluate all available formats (fp32, fp16, int8)
    python evaluate_models.py --model runs/detect/.../weights/best.pt \
        --data ./composite_dataset/data.yaml --fp16 --int8

    # explicitly specify tflite files
    python evaluate_models.py --model my.pt --data coco.yaml \
        --fp16-file my_fp16.tflite --int8-file my_int8.tflite

"""

import argparse
import sys
from pathlib import Path

# use non-GUI backend for matplotlib so script can run headless
import matplotlib
matplotlib.use('Agg')


import csv
import matplotlib.pyplot as plt


def _get_tflite_interpreter():
    """Return a tflite Interpreter class, trying tflite_runtime then tensorflow."""
    try:
        import tflite_runtime.interpreter as tflite
        return tflite.Interpreter
    except ImportError:
        pass
    try:
        import tensorflow as tf
        return tf.lite.Interpreter
    except ImportError:
        pass
    return None


def probe_tflite(path: Path):
    """Try to allocate a TFLite model and return (ok: bool, error_msg: str | None).

    This is a cheap pre-flight check that catches XNNPACK delegate failures
    (e.g. INT8 weights mixed with FLOAT32 activations) before launching the
    expensive Ultralytics YOLO.val() pipeline.
    """
    Interpreter = _get_tflite_interpreter()
    if Interpreter is None:
        # Cannot verify – let Ultralytics try and fail loudly on its own.
        return True, None
    try:
        interp = Interpreter(model_path=str(path))
        interp.allocate_tensors()
        return True, None
    except Exception as exc:
        return False, str(exc)


def evaluate(model_path: Path, data: str, imgsz: int, device: str):
    """Run validation, prints and returns results summary."""
    from ultralytics import YOLO

    print(f"Evaluating: {model_path} on device={device}")
    m = YOLO(str(model_path))
    results = m.val(data=data, imgsz=imgsz, device=device)
    # results is a list of Results objects, we just display a short summary
    try:
        summary = results.box.map  # mAP@0.5
    except Exception:
        summary = results
    return results


def main():
    parser = argparse.ArgumentParser(
        description="Evaluate YOLO26 models including TFLite exports",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python evaluate_models.py --model yolo26n.pt --data coco.yaml
  python evaluate_models.py --model yolo26n.pt --data coco.yaml --fp16 --int8
""",
    )

    parser.add_argument("--model", "-m", required=True,
                        help="Path to base YOLO .pt model")
    parser.add_argument("--data", "-d", required=True,
                        help="Dataset YAML for val set")
    parser.add_argument("--imgsz", type=int, default=640,
                        help="Image size used for validation")
    parser.add_argument("--device", default="cpu",
                        help="Device to run inference on (cpu)" )
    parser.add_argument("--assets-dir", default=None,
                        help="Directory to search for exported tflite files (e.g. TeamCode/src/main/assets)")

    parser.add_argument("--fp16", action="store_true",
                        help="Also evaluate fp16 tflite model if it exists")
    parser.add_argument("--int8", action="store_true",
                        help="Also evaluate int8 tflite model if it exists")
    parser.add_argument("--fp16-file", default=None,
                        help="Explicit fp16 tflite file path")
    parser.add_argument("--int8-file", default=None,
                        help="Explicit int8 tflite file path")

    args = parser.parse_args()

    model_path = Path(args.model)
    if not model_path.exists():
        print(f"Error: model file not found: {model_path}")
        sys.exit(1)

    # results accumulators for CSV
    records = []

    def _saved_model_fallback(candidate: Path) -> Path | None:
        """Look for the same filename inside any *_saved_model/ sibling directory."""
        workspace = Path('.')
        for sm_dir in workspace.glob('*_saved_model'):
            alt = sm_dir / candidate.name
            if alt.exists():
                return alt
        return None

    def find_file(candidate: Path) -> Path:
        # if exists return it, otherwise try assets dir if provided
        if candidate.exists():
            return candidate
        if args.assets_dir:
            assets = Path(args.assets_dir)
            # direct filename match
            alt = assets / candidate.name
            if alt.exists():
                print(f"Found {candidate.name} in assets dir {args.assets_dir}")
                # Pre-flight probe: verify the TFLite model can be loaded.
                if candidate.suffix.lower() == '.tflite':
                    ok, err = probe_tflite(alt)
                    if not ok:
                        print(f"  WARNING: {alt.name} failed compatibility probe: {err}")
                        fb = _saved_model_fallback(candidate)
                        if fb:
                            ok2, err2 = probe_tflite(fb)
                            if ok2:
                                print(f"  Using fallback {fb} instead.")
                                return fb
                            else:
                                print(f"  Fallback {fb} also failed probe: {err2}")
                        return alt  # return anyway; caller will handle the failure
                return alt
            # heuristic search by precision or base name
            base = model_path.stem.replace("-seg", "").replace("_seg", "")
            patterns = []
            # precision from candidate name
            if 'fp16' in candidate.name.lower() or 'float16' in candidate.name.lower():
                patterns.append(f"*fp16*.tflite")
                patterns.append(f"*float16*.tflite")
            if 'int8' in candidate.name.lower():
                patterns.append(f"*int8*.tflite")
            # also match base name prefix
            patterns.append(f"{base}*.tflite")
            for pat in patterns:
                for p in assets.glob(pat):
                    print(f"Found {p.name} (pattern {pat}) in assets dir")
                    if candidate.suffix.lower() == '.tflite':
                        ok, err = probe_tflite(p)
                        if not ok:
                            print(f"  WARNING: {p.name} failed compatibility probe: {err}")
                            fb = _saved_model_fallback(p)
                            if fb:
                                ok2, _ = probe_tflite(fb)
                                if ok2:
                                    print(f"  Using fallback {fb} instead.")
                                    return fb
                            continue  # skip broken file, try next pattern match
                    return p
        return candidate

    def run_and_record(path: Path, precision: str):
        real_path = find_file(path)
        if not real_path.exists():
            print(f"{precision} file not found: {real_path}")
            records.append({
                'model': path.name,
                'precision': precision,
                'status': 'not_found',
                'preprocess_ms': None, 'inference_ms': None, 'postprocess_ms': None,
                'mAP50': None, 'mAP50-95': None,
            })
            return

        # Pre-flight probe for TFLite files
        if real_path.suffix.lower() == '.tflite':
            ok, err = probe_tflite(real_path)
            if not ok:
                _XNNPACK_HINT = (
                    "This usually means the model uses mixed INT8/FLOAT32 quantization "
                    "that the XNNPACK delegate cannot handle. "
                    "Try the *_integer_quant.tflite variant (float32 I/O, INT8 weights) "
                    "or the *_float16.tflite variant instead."
                )
                print(
                    f"SKIP {precision} model {real_path}: TFLite allocate_tensors() failed.\n"
                    f"  Error: {err}\n"
                    f"  Hint:  {_XNNPACK_HINT}"
                )
                records.append({
                    'model': real_path.name,
                    'precision': precision,
                    'status': f'tflite_load_error: {err}',
                    'preprocess_ms': None, 'inference_ms': None, 'postprocess_ms': None,
                    'mAP50': None, 'mAP50-95': None,
                })
                return

        try:
            res = evaluate(real_path, args.data, args.imgsz, args.device)
        except Exception as e:
            print(f"Failed to evaluate {precision} model {real_path}: {e}")
            records.append({
                'model': real_path.name,
                'precision': precision,
                'status': f'eval_error: {e}',
                'preprocess_ms': None, 'inference_ms': None, 'postprocess_ms': None,
                'mAP50': None, 'mAP50-95': None,
            })
            return
        # take speed info
        speed = res.speed if hasattr(res, 'speed') else {}
        records.append({
            'model': real_path.name,
            'precision': precision,
            'status': 'ok',
            'preprocess_ms': speed.get('preprocess', None),
            'inference_ms': speed.get('inference', None),
            'postprocess_ms': speed.get('postprocess', None),
            'mAP50': res.results_dict.get('metrics/mAP50(B)', None) if hasattr(res, 'results_dict') else None,
            'mAP50-95': res.results_dict.get('metrics/mAP50-95(B)', None) if hasattr(res, 'results_dict') else None,
        })

    # evaluate raw pt (fp32)
    run_and_record(model_path, 'FP32')

    base = model_path.stem.replace("-seg", "").replace("_seg", "")
    parent = model_path.parent

    if args.fp16 or args.fp16_file:
        fp16_path = Path(args.fp16_file) if args.fp16_file else parent / f"{base}_fp16.tflite"
        run_and_record(fp16_path, 'FP16')

    if args.int8 or args.int8_file:
        int8_path = Path(args.int8_file) if args.int8_file else parent / f"{base}_int8.tflite"
        run_and_record(int8_path, 'INT8')

    # write CSV
    csv_path = Path('eval_results.csv')
    fieldnames = ['model', 'precision', 'status',
                  'preprocess_ms', 'inference_ms', 'postprocess_ms',
                  'mAP50', 'mAP50-95']
    if records:
        with csv_path.open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(records)
        print(f"Saved evaluation CSV to {csv_path}")
    else:
        print("No evaluation records to save (all models were skipped or failed).")

    # make boxplot for inference speed
    try:
        import pandas as pd
        df = pd.DataFrame(records)
        fig, ax = plt.subplots()
        df.boxplot(column='inference_ms', by='precision', ax=ax)
        ax.set_title('Inference speed (ms)')
        ax.set_ylabel('ms per image')
        fig.savefig('inference_speed_boxplot.png')
        print("Saved boxplot to inference_speed_boxplot.png")
    except ImportError:
        print("pandas/matplotlib not available, skipping boxplot")


if __name__ == "__main__":
    try:
        from ultralytics import YOLO
    except ImportError:
        print("Error: ultralytics not installed. Run: pip install ultralytics>=8.4.14")
        sys.exit(1)
    main()
