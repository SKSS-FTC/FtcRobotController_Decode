"""
AprilTag detection with pose matrix output using pupil-apriltags + OpenCV.
Loads camera intrinsics from camera_calibration.yaml.

Usage:
    python detect_apriltag.py [--tag-size 0.165] [--family tag36h11] [--camera 1]
"""

import argparse
import os
import cv2
import numpy as np
import yaml
from pupil_apriltags import Detector

CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), "..", "camera_calibration.yaml")


def load_calibration(path: str):
    with open(path) as f:
        data = yaml.safe_load(f)
    K = np.array(data["camera_matrix"], dtype=np.float64)
    dist = np.array(data["dist_coeff"], dtype=np.float64).flatten()
    return K, dist


def draw_detection(frame, det, pose_R, pose_t, fx, fy, cx_k, cy_k, axis_len=0.1):
    """Draw tag corners and 3D colour-coded axis arrows projected onto the image.

    X → red, Y → green, Z → blue  (tag-frame axes drawn from the tag origin).
    ``axis_len`` is the arrow length in metres (default 0.1 m).
    """
    corners = det.corners.astype(int)

    # Draw outline
    for i in range(4):
        cv2.line(frame, tuple(corners[i]), tuple(corners[(i + 1) % 4]), (0, 255, 0), 2)

    # Center dot
    icx, icy = int(det.center[0]), int(det.center[1])
    cv2.circle(frame, (icx, icy), 5, (0, 0, 255), -1)

    # Tag ID label
    cv2.putText(frame, f"ID {det.tag_id}", (icx - 20, icy - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    # --- 3D axis arrows ---
    # pose_R, pose_t describe the tag→camera transform.
    # Project a point p (in tag frame) to image:
    #   p_cam = R @ p + t
    #   u = fx * p_cam[0]/p_cam[2] + cx_k
    def project(pt_tag):
        p = pose_R @ np.array(pt_tag, dtype=np.float64) + pose_t.flatten()
        if p[2] <= 0:
            return None
        u = int(fx * p[0] / p[2] + cx_k)
        v = int(fy * p[1] / p[2] + cy_k)
        return (u, v)

    origin = project([0, 0, 0])
    axes = [
        ([axis_len, 0,        0       ], (0,   0,   255), "X"),  # red
        ([0,        axis_len, 0       ], (0,   255, 0  ), "Y"),  # green
        ([0,        0,        axis_len], (255, 0,   0  ), "Z"),  # blue
    ]

    if origin is not None:
        for pt_tag, colour, label in axes:
            tip = project(pt_tag)
            if tip is not None:
                cv2.arrowedLine(frame, origin, tip, colour, 2, tipLength=0.2)
                cv2.putText(frame, label, tip,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)


def main():
    parser = argparse.ArgumentParser(description="AprilTag detection with pose matrix")
    parser.add_argument("--tag-size", type=float, default=0.20,
                        help="Physical tag side length in metres (default: 0.166)")
    parser.add_argument("--family", default="tag36h11",
                        help="AprilTag family (default: tag36h11)")
    parser.add_argument("--camera", type=int, default=0,
                        help="Camera index (default: 0)")
    parser.add_argument("--z-up", action="store_true",
                        help="Reframe tag axes: Z up, X left, Y forward (into wall)")
    args = parser.parse_args()

    # --- Camera intrinsics ---
    K, _ = load_calibration(CALIBRATION_FILE)
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    print(f"Camera matrix:\n{K}")

    # --- Detector ---
    detector = Detector(
        families=args.family,
        nthreads=4,
        quad_decimate=2.0,
        quad_sigma=0.0,
        decode_sharpening=0.25,
    )

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {args.camera}")

    print("Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Undistort frame
        # frame = cv2.undistort(frame, K, dist)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detections = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(fx, fy, cx, cy),
            tag_size=args.tag_size,
        )

        for det in detections:
            pose_R = det.pose_R          # 3x3 rotation matrix
            pose_t = det.pose_t          # 3x1 translation vector  (metres)
            pose = np.eye(4, dtype=np.float64)  # homogeneous transform (tag->camera)
            pose[:3, :3] = pose_R
            pose[:3, 3] = pose_t.flatten()

            # apply a 90° rotation about the X axis (tag pitched forward)
            # x90 = np.array([[1,  0,       0,      0],
            #                  [0,  np.cos(np.pi/-2), -np.sin(np.pi/-2), 0],
            #                  [0,  np.sin(np.pi/-2),  np.cos(np.pi/-2), 0],
            #                  [0,  0,       0,      1]], dtype=np.float64)
            x90 = np.array([[1,  0,       0,      0],
                             [0,  np.cos(0), -np.sin(0), 0],
                             [0,  np.sin(0),  np.cos(0), 0],
                             [0,  0,       0,      1]], dtype=np.float64)
            pose = pose @ x90
            # pose = np.linalg.inv(pose)  # now tag->camera becomes camera->tag
            pose_R = pose[:3, :3]
            pose_t = pose[:3, 3]
                

            # Console output
            print(f"\n--- Tag ID: {det.tag_id}  family: {det.tag_family.decode()} ---")
            print(f"Rotation matrix R:\n{pose_R}")
            print(f"Translation vector t (m):\n{pose_t.flatten()}")
            # compute camera->tag homogeneous transform
            R_ct = pose_R.T
            t_ct = -R_ct @ pose_t.flatten()
            H_ct = np.eye(4, dtype=np.float64)
            H_ct[:3, :3] = R_ct
            H_ct[:3, 3] = t_ct

            draw_detection(frame, det, pose_R, pose_t, fx, fy, cx, cy,
                           axis_len=args.tag_size * 0.6)

        # Tag count overlay
        cv2.putText(frame, f"Tags: {len(detections)}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.imshow("AprilTag Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
