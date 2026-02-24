"""
Checkerboard camera calibration script.

Captures frames from a live camera (or reads images from a folder) and uses
OpenCV's checkerboard corner detector to compute camera intrinsics.

Square size: 20 mm x 20 mm  (default, change with --square-size)
Board inner corners: 7 x 9   (default, change with --cols / --rows)
  → Matches a standard 7-column 9-row physical checkerboard.

Output: camera_calibration.yaml  (same format used by detect_apriltag.py)

Usage – live camera:
    python calibrate_camera.py

Usage – from a folder of images:
    python calibrate_camera.py --images ./calib_images

Key controls (live mode):
    SPACE  – capture current frame
    c      – run calibration immediately (requires ≥ 10 frames)
    q / ESC – quit
"""

import argparse
import glob
import os
import sys

import cv2
# disable OpenCL to avoid GPU-related "!f.fail()" errors
cv2.ocl.setUseOpenCL(False)
import numpy as np
import yaml
import time

# ---------------------------------------------------------------------------
# Default board geometry
# ---------------------------------------------------------------------------
# The checkerboard has inner corner counts; by default we use a 7×9
# pattern (i.e. 8×10 squares) because that's what the user specified.
DEFAULT_COLS = 7         # inner corners per row   (squares - 1)
DEFAULT_ROWS = 9         # inner corners per column (squares - 1)
DEFAULT_SQUARE_MM = 20.0 # physical square side in millimetres
MIN_FRAMES = 100          # minimum accepted frames for calibration

OUTPUT_YAML = os.path.join(os.path.dirname(__file__), "..", "camera_calibration.yaml")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description="Checkerboard camera calibration")
    p.add_argument("--camera",      type=int,   default=0,
                   help="Camera index (default 0)")
    p.add_argument("--images",      type=str,   default=None,
                   help="Path to folder containing calibration images "
                        "(*.jpg / *.png).  Skips live capture.")
    p.add_argument("--auto", action="store_true",
                   help="Enable automatic capture when corners are detected")
    p.add_argument("--auto-interval", type=float, default=1.0,
                   help="Minimum seconds between automatic captures (default 1.0)")
    p.add_argument("--debug", action="store_true",
                   help="Show preprocessing steps and detection methods in real-time")
    p.add_argument("--cols",        type=int,   default=DEFAULT_COLS,
                   help=f"Inner corners per row   (default {DEFAULT_COLS})")
    p.add_argument("--rows",        type=int,   default=DEFAULT_ROWS,
                   help=f"Inner corners per column (default {DEFAULT_ROWS})")
    p.add_argument("--square-size", type=float, default=DEFAULT_SQUARE_MM,
                   help=f"Square size in mm (default {DEFAULT_SQUARE_MM})")
    p.add_argument("--output",      type=str,   default=OUTPUT_YAML,
                   help="Output YAML file (default: ../camera_calibration.yaml)")
    return p.parse_args()


def build_object_points(cols: int, rows: int, square_mm: float) -> np.ndarray:
    """Return 3-D world coordinates for one checkerboard view (Z = 0)."""
    objp = np.zeros((rows * cols, 3), dtype=np.float64)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_mm          # scale to millimetres
    return objp


def find_corners(gray: np.ndarray, cols: int, rows: int, debug: bool = False):
    """Detect sub-pixel checkerboard corners with robust fallback strategies.
    
    Attempts multiple detection methods in order:
    1. Standard findChessboardCorners with adaptive thresholding
    2. Enhanced with CLAHE (Contrast Limited Adaptive Histogram Equalization)
    3. Enhanced with bilateral filtering
    4. findChessboardCornersSB (symmetric board, more robust but slower)
    
    Returns (found, corners, method_name)
    """
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    board_size = (cols, rows)
    
    # Strategy 1: Standard detection with adaptive thresholding
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH
             | cv2.CALIB_CB_NORMALIZE_IMAGE
             | cv2.CALIB_CB_FAST_CHECK)
    found, corners = cv2.findChessboardCorners(gray, board_size, flags)
    if found:
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        return found, corners, "standard"
    
    # Strategy 2: CLAHE enhancement (improves contrast in challenging lighting)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)
    if debug:
        cv2.imshow("Debug: CLAHE Enhanced", enhanced)
    found, corners = cv2.findChessboardCorners(enhanced, board_size, flags)
    if found:
        corners = cv2.cornerSubPix(enhanced, corners, (11, 11), (-1, -1), criteria)
        return found, corners, "CLAHE"
    
    # Strategy 3: Bilateral filtering (reduce noise while preserving edges)
    filtered = cv2.bilateralFilter(gray, 9, 75, 75)
    if debug:
        cv2.imshow("Debug: Bilateral Filtered", filtered)
    found, corners = cv2.findChessboardCorners(filtered, board_size, flags)
    if found:
        corners = cv2.cornerSubPix(filtered, corners, (11, 11), (-1, -1), criteria)
        return found, corners, "bilateral"
    
    # Strategy 4: findChessboardCornersSB (symmetric board detector - more robust)
    # This is slower but works better with challenging conditions
    try:
        found, corners = cv2.findChessboardCornersSB(gray, board_size, 
                                                      cv2.CALIB_CB_EXHAUSTIVE 
                                                      | cv2.CALIB_CB_ACCURACY)
        if found:
            # SB already gives sub-pixel accuracy, but refine further
            corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
            return found, corners, "SB"
    except Exception:
        pass  # findChessboardCornersSB might not be available in older OpenCV
    
    # Strategy 5: SB with CLAHE enhancement
    try:
        found, corners = cv2.findChessboardCornersSB(enhanced, board_size,
                                                      cv2.CALIB_CB_EXHAUSTIVE 
                                                      | cv2.CALIB_CB_ACCURACY)
        if found:
            corners = cv2.cornerSubPix(enhanced, corners, (5, 5), (-1, -1), criteria)
            return found, corners, "SB+CLAHE"
    except Exception:
        pass
    
    return False, None, "none"


def calibrate(obj_pts, img_pts, img_size):
    """Run cv2.calibrateCamera and return (rms, K, dist).

    OpenCV expects a list of vectors of Point3f (float32) for object points
    and Point2f for image points.  Ensure the arrays are correctly shaped
    and typed to avoid the "Unsupported format" error noticed in v4.11.
    """
    # convert to required dtype/shape
    obj32 = [np.asarray(pts, dtype=np.float32).reshape(-1, 1, 3)
             for pts in obj_pts]
    img32 = [np.asarray(pts, dtype=np.float32).reshape(-1, 1, 2)
             for pts in img_pts]

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj32, img32, img_size, None, None
    )
    return rms, K, dist


def save_yaml(path: str, K: np.ndarray, dist: np.ndarray):
    """Save intrinsics to YAML in the same format as camera_calibration.yaml."""
    data = {
        "camera_matrix": K.tolist(),
        "dist_coeff":    dist.tolist(),
    }
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=False)
    print(f"\nCalibration saved → {os.path.abspath(path)}")


def print_result(rms, K, dist):
    print(f"\n{'='*55}")
    print(f"  RMS reprojection error : {rms:.4f} px")
    print(f"  fx = {K[0,0]:.2f}   fy = {K[1,1]:.2f}")
    print(f"  cx = {K[0,2]:.2f}   cy = {K[1,2]:.2f}")
    print(f"  dist: {dist.flatten().tolist()}")
    print(f"{'='*55}")


# ---------------------------------------------------------------------------
# From-folder mode
# ---------------------------------------------------------------------------

def calibrate_from_folder(folder: str, cols: int, rows: int,
                           square_mm: float, output: str):
    patterns = [os.path.join(folder, ext)
                for ext in ("*.jpg", "*.jpeg", "*.png", "*.bmp")]
    paths = []
    for pat in patterns:
        paths.extend(glob.glob(pat))
    paths.sort()

    if not paths:
        print(f"No images found in '{folder}'")
        sys.exit(1)

    print(f"Found {len(paths)} image(s) in '{folder}'")

    objp = build_object_points(cols, rows, square_mm)
    obj_pts, img_pts = [], []
    img_size = None

    for path in paths:
        img = cv2.imread(path)
        if img is None:
            print(f"  [skip] cannot read {path}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img_size is None:
            img_size = (gray.shape[1], gray.shape[0])

        found, corners, method = find_corners(gray, cols, rows)
        status = f"OK ({method})" if found else "no corners"
        print(f"  {os.path.basename(path):40s} → {status}")

        if found:
            obj_pts.append(objp)
            img_pts.append(corners)
            cv2.drawChessboardCorners(img, (cols, rows), corners, found)

        cv2.imshow("Calibration – folder", img)
        cv2.waitKey(300)

    cv2.destroyAllWindows()

    if len(obj_pts) < MIN_FRAMES:
        print(f"\nOnly {len(obj_pts)} usable frames (need ≥ {MIN_FRAMES}). "
              "Calibration aborted.")
        sys.exit(1)

    print(f"\nCalibrating with {len(obj_pts)} frame(s)…")
    rms, K, dist = calibrate(obj_pts, img_pts, img_size)
    print_result(rms, K, dist)
    save_yaml(output, K, dist)


# ---------------------------------------------------------------------------
# Live-camera mode
# ---------------------------------------------------------------------------

def calibrate_live(camera_idx: int, cols: int, rows: int,
                   square_mm: float, output: str,
                   auto: bool = False, auto_interval: float = 1.0,
                   debug: bool = False):
    cap = cv2.VideoCapture(camera_idx)
    if not cap.isOpened():
        print(f"Cannot open camera {camera_idx}")
        sys.exit(1)

    objp = build_object_points(cols, rows, square_mm)
    obj_pts, img_pts = [], []
    img_size = None
    last_capture = 0.0

    print("\nLive calibration controls:")
    print("  SPACE  – capture frame (needs detected corners)")
    print("  c      – calibrate now (needs ≥ 10 frames)")
    print("  q/ESC  – quit\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame read error")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if img_size is None:
            img_size = (gray.shape[1], gray.shape[0])

        found, corners, method = find_corners(gray, cols, rows, debug)

        display = frame.copy()
        if found:
            cv2.drawChessboardCorners(display, (cols, rows), corners, found)

        # HUD
        info = (f"Board {cols}x{rows}  sq={square_mm:.0f}mm  "
            f"captured={len(obj_pts)}/{MIN_FRAMES}+")
        # HUD color: green if found, orange otherwise
        color = (0, 255, 0) if found else (0, 80, 255)
        if auto:
            info += f"  AUTO(interval={auto_interval:.1f}s)"
        cv2.putText(display, info, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
        detect_txt = f"Detected! ({method})" if found else "No corners"
        cv2.putText(display, detect_txt, (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

        cv2.imshow("Checkerboard calibration – SPACE=capture  c=calibrate  q=quit",
                   display)

        key = cv2.waitKey(1) & 0xFF

        if key in (ord('q'), 27):          # q or ESC
            break

        if key == ord(' ') and found:      # SPACE – capture
            obj_pts.append(objp)
            img_pts.append(corners)
            last_capture = time.time()
            print(f"  Captured frame #{len(obj_pts)} (manual)")

        # Automatic capture when enabled (debounced by auto_interval)
        if auto and found:
            now = time.time()
            if now - last_capture >= float(auto_interval):
                obj_pts.append(objp)
                img_pts.append(corners)
                last_capture = now
                print(f"  Captured frame #{len(obj_pts)} (auto)")

        if key == ord('c'):                # c – calibrate now
            if len(obj_pts) < MIN_FRAMES:
                print(f"  Need ≥ {MIN_FRAMES} frames (have {len(obj_pts)})")
            else:
                break

        # If auto mode and we have enough frames, auto-run calibration
        if auto and len(obj_pts) >= MIN_FRAMES:
            print(f"\nAuto mode collected {len(obj_pts)} frames — running calibration")
            break

    cap.release()
    cv2.destroyAllWindows()

    if len(obj_pts) < MIN_FRAMES:
        print(f"\nNot enough frames ({len(obj_pts)} < {MIN_FRAMES}). "
              "Calibration aborted.")
        sys.exit(0)

    print(f"\nCalibrating with {len(obj_pts)} frame(s)…")
    rms, K, dist = calibrate(obj_pts, img_pts, img_size)
    print_result(rms, K, dist)
    save_yaml(output, K, dist)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    args = parse_args()

    print(f"Board : {args.cols} × {args.rows} inner corners  "
          f"(square = {args.square_size:.0f} mm)")

    if args.images:
        calibrate_from_folder(args.images, args.cols, args.rows,
                               args.square_size, args.output)
    else:
        calibrate_live(args.camera, args.cols, args.rows,
                       args.square_size, args.output,
                       auto=args.auto, auto_interval=args.auto_interval,
                       debug=args.debug)


if __name__ == "__main__":
    main()
