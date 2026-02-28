"""
AprilTag TF Visualizer - Camera as Base Frame  /  Map Frame

Two visualization modes:

1. Camera frame (default)  – tag pose expressed relative to the camera origin.
2. Map frame  (--map-mode) – camera position inferred from a known tag position in
   the field map.  The tag is placed at a fixed coordinate; the camera location is
   back-computed from the tag→camera transform.

Usage:
    python visualize_apriltag_tf.py [--camera 0] [--tag-size 0.165]
    python visualize_apriltag_tf.py --map-mode [--tag-yaw 54]
"""

import argparse
import os
import cv2
import numpy as np
import yaml
from pupil_apriltags import Detector
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), "..", "camera_calibration.yaml")

# ===========================================================================
# Map-frame constants  (from Constants.java + user specification)
# Map origin: bottom-left corner of field
# Map axes:   X = right (east), Y = north (forward), Z = up
# ===========================================================================
HALF_FIELD = 1.829   # metres

# Map center in map frame
MAP_CENTER = np.array([HALF_FIELD, HALF_FIELD, 0.0])   # (1.829, 1.829, 0)

# Tag positions = center-frame offset + HALF_FIELD
# Tag 24 RED:  center-frame ( 1.482,  1.413, 0.7493)  ->  upper-right
# Tag 20 BLUE: center-frame (-1.482,  1.413, 0.7493)  ->  upper-left
RED_TAG_POS_MAP  = np.array([ 1.482 + HALF_FIELD,  1.413 + HALF_FIELD, 0.7493])  # (3.311, 3.242, 0.7493)
BLUE_TAG_POS_MAP = np.array([-1.482 + HALF_FIELD,  1.413 + HALF_FIELD, 0.7493])  # (0.347, 3.242, 0.7493)

# Yaw convention: ψ such that face normal (tag-Z, toward camera) = (sin ψ, −cos ψ, 0)
# Tag 24 RED  faces toward map center from upper-right  → yaw = -54°
# Tag 20 BLUE faces toward map center from upper-left   → yaw = +54°
RED_TAG_YAW_DEG  = -54.0
BLUE_TAG_YAW_DEG =  54.0

# Convenient dict: tag_id → (position, yaw)
TAG_INFO = {
    24: (RED_TAG_POS_MAP,  RED_TAG_YAW_DEG,  'RED'),
    20: (BLUE_TAG_POS_MAP, BLUE_TAG_YAW_DEG, 'BLUE'),
}

# Default CLI tag (detected tag)
DEFAULT_DETECT_TAG_ID = 24
# ===========================================================================


def to_viz(pt):
    """Convert camera-frame coordinates to visualization frame.

    Camera frame: X-right, Y-down,    Z-forward
    Viz frame:    X-right, Y-forward, Z-up

    Mapping: viz_X = cam_X,  viz_Y = cam_Z,  viz_Z = -cam_Y
    """
    pt = np.asarray(pt, dtype=float).flatten()
    return np.array([pt[0], pt[2], -pt[1]])


# ---------------------------------------------------------------------------
# Map-frame back-projection
# ---------------------------------------------------------------------------

def build_R_map_tag(tag_yaw_deg: float) -> np.ndarray:
    """
    Rotation matrix from tag frame to map frame.

    pupil-apriltags / OpenCV tag frame:
      tag-X – right across the face
      tag-Y – DOWN on the face
      tag-Z – INTO the wall (away from camera side)
               Camera origin in tag frame = -R^T @ t, which for a frontal tag
               (t ≈ (0,0,d)) gives cam_in_tag ≈ (0, 0, -d), i.e. in the -tag-Z
               direction.  So tag-Z must point INTO the wall so that the camera
               ends up inside the field.

    FTC map frame: X-right (east), Y-north (forward), Z-up.

    Reference orientation (ψ=0): tag on south wall facing north.
      tag-X = +map-X,  tag-Y = -map-Z,  tag-Z = +map-Y (into wall)

    After rotating ψ around map-Z:
      col-0 (tag-X) = ( cos ψ,  sin ψ,  0)
      col-1 (tag-Y) = (     0,      0, -1)
      col-2 (tag-Z) = (-sin ψ,  cos ψ,  0)   ← INTO the wall

    Verification:
      ψ = -54° → tag-Z = (+0.809, +0.588, 0)  [RED  tag → into upper-right wall]
      ψ = +54° → tag-Z = (-0.809, +0.588, 0)  [BLUE tag → into upper-left  wall]
      Camera (front of tag) is in −tag-Z direction → points toward field interior ✓
    """
    psi = np.radians(tag_yaw_deg)
    c, s = np.cos(psi), np.sin(psi)
    return np.array([
        [ c,  0, -s],
        [ s,  0,  c],
        [ 0, -1,  0],
    ])


def compute_camera_in_map_frame(
        pose_R: np.ndarray,
        pose_t: np.ndarray,
        tag_pos_map: np.ndarray,
        tag_yaw_deg: float,
) -> tuple:
    """
    Back-compute camera position and orientation in the map frame.

    The tag is at a known fixed location/orientation in the map.
    The detection gives us the tag's pose *in the camera frame*.

    Math:
        cam_in_tag_frame  = -R.T  @ t          (invert tag→camera transform)
        cam_pos_map       = R_map_tag @ cam_in_tag + tag_pos_map
        R_map_cam         = R_map_tag @ R.T     (R.T = R_cam_tag → R_tag_cam)

    Returns:
        cam_pos_map  – (3,) camera origin in map frame (meters)
        R_map_cam    – (3,3) rotation: camera frame → map frame
    """
    pose_t_flat = pose_t.flatten()
    R_tag_cam   = pose_R.T                        # camera frame expressed in tag frame
    cam_in_tag  = -R_tag_cam @ pose_t_flat        # camera origin in tag frame

    R_mt = build_R_map_tag(tag_yaw_deg)
    cam_pos_map = R_mt @ cam_in_tag + tag_pos_map
    R_map_cam   = R_mt @ R_tag_cam
    return cam_pos_map, R_map_cam


def draw_map_origin(ax, scale=0.2):
    """Draw XYZ axes for the map frame at (0,0,0)."""
    origin = np.zeros(3)
    ax.quiver(*origin, scale, 0, 0, color='r', linewidth=2, arrow_length_ratio=0.25)
    ax.text(scale * 1.15, 0, 0, 'Map-X\n(east)', color='r', fontsize=9, fontweight='bold')

    ax.quiver(*origin, 0, scale, 0, color='g', linewidth=2, arrow_length_ratio=0.25)
    ax.text(0, scale * 1.15, 0, 'Map-Y\n(north)', color='g', fontsize=9, fontweight='bold')

    ax.quiver(*origin, 0, 0, scale, color='b', linewidth=2, arrow_length_ratio=0.25)
    ax.text(0, 0, scale * 1.15, 'Map-Z\n(up)', color='b', fontsize=9, fontweight='bold')

    ax.scatter(*origin, color='k', s=120, marker='o')
    ax.text(0.02, 0.02, 0.02, 'MAP\nORIGIN', fontsize=8, fontweight='bold', color='k')


def draw_map_center(ax, scale=0.2):
    """Draw the map center frame at (HALF_FIELD, HALF_FIELD, 0)."""
    o = MAP_CENTER
    ax.quiver(*o, scale, 0, 0, color='r', linewidth=2, arrow_length_ratio=0.25, linestyle='dashed')
    ax.quiver(*o, 0, scale, 0, color='g', linewidth=2, arrow_length_ratio=0.25, linestyle='dashed')
    ax.quiver(*o, 0, 0, scale, color='b', linewidth=2, arrow_length_ratio=0.25, linestyle='dashed')
    ax.scatter(*o, color='orange', s=200, marker='*', zorder=5)
    ax.text(o[0] + 0.05, o[1] + 0.05, o[2] + 0.05,
            f'MAP CENTER\n({o[0]:.3f}, {o[1]:.3f}, {o[2]:.3f})',
            fontsize=9, fontweight='bold', color='darkorange',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='moccasin',
                      edgecolor='darkorange', alpha=0.9))


def draw_camera_in_map(ax, cam_pos_map: np.ndarray, R_map_cam: np.ndarray, scale=0.15):
    """Draw camera axes and frustum symbol at the computed map-frame position."""
    o = cam_pos_map
    # Camera X/Y/Z columns of R_map_cam expressed in map frame
    cx = R_map_cam[:, 0] * scale
    cy = R_map_cam[:, 1] * scale
    cz = R_map_cam[:, 2] * scale

    ax.quiver(*o, *cx, color='r', linewidth=2, arrow_length_ratio=0.3, alpha=0.8)
    ax.quiver(*o, *cy, color='g', linewidth=2, arrow_length_ratio=0.3, alpha=0.8)
    ax.quiver(*o, *cz, color='b', linewidth=2, arrow_length_ratio=0.3, alpha=0.8)

    ax.scatter(*o, color='k', s=150, marker='^')
    ax.text(o[0] + 0.04, o[1], o[2] + 0.04,
            f'CAMERA\n({o[0]:.3f}, {o[1]:.3f}, {o[2]:.3f})',
            fontsize=9, fontweight='bold', color='k',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='lightyellow',
                      edgecolor='black', alpha=0.9))


def draw_tag_in_map(ax, tag_pos_map: np.ndarray, tag_yaw_deg: float,
                    tag_id: int, label: str = '', tag_size=0.165, scale=0.12):
    """Draw the AprilTag at its known map-frame position with ID label."""
    R_mt = build_R_map_tag(tag_yaw_deg)
    colour = 'royalblue' if label == 'BLUE' else 'tomato'
    face_colour = 'lightblue' if label == 'BLUE' else 'lightsalmon'

    # Tag local face corners (in tag frame: XY plane, z=0)
    h = tag_size / 2
    local_corners = np.array([
        [-h, -h, 0],
        [ h, -h, 0],
        [ h,  h, 0],
        [-h,  h, 0],
    ])
    corners_map = (R_mt @ local_corners.T).T + tag_pos_map

    for i in range(4):
        p1, p2 = corners_map[i], corners_map[(i + 1) % 4]
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]],
                color=colour, linewidth=2, alpha=0.9)
    ax.add_collection3d(Poly3DCollection([corners_map], facecolors=face_colour, alpha=0.4))

    # Tag axes (X red, Y green, Z=normal blue)
    for col, c, lbl in [(0, 'r', 'tX'), (1, 'g', 'tY'), (2, 'b', 'tZ')]:
        v = R_mt[:, col] * scale
        ax.quiver(*tag_pos_map, *v, color=c, linewidth=1.5,
                  arrow_length_ratio=0.3, alpha=0.8)
        ax.text(*(tag_pos_map + v * 1.3), lbl, color=c, fontsize=8)

    ax.scatter(*tag_pos_map, color=colour, s=120, marker='s', zorder=5)
    ax.text(tag_pos_map[0] + 0.04, tag_pos_map[1], tag_pos_map[2] + 0.08,
            f'Tag {tag_id} ({label})\n({tag_pos_map[0]:.3f}, {tag_pos_map[1]:.3f}, {tag_pos_map[2]:.3f})\nyaw={tag_yaw_deg}°',
            fontsize=8, fontweight='bold', color=colour,
            bbox=dict(boxstyle='round,pad=0.3', facecolor=face_colour,
                      edgecolor=colour, alpha=0.9))


def create_map_frame_visualization(
        detections_data,
        camera_params,
        detect_tag_id: int,
        tag_size: float,
):
    """
    Visualize everything in the map frame.

    Both tags (20 BLUE, 24 RED) are drawn at their fixed map positions.
    The map origin (0,0,0) and map center (1.829,1.829,0) are shown.
    The camera position is back-computed from whichever tag was detected.
    """
    fig = plt.figure(figsize=(15, 11))
    ax  = fig.add_subplot(111, projection='3d')

    # Map origin and center frames
    draw_map_origin(ax, scale=0.20)
    draw_map_center(ax, scale=0.20)

    # Both tags at fixed map positions
    for tid, (tpos, tyaw, tlabel) in TAG_INFO.items():
        draw_tag_in_map(ax, tpos, tyaw, tag_id=tid, label=tlabel,
                        tag_size=tag_size, scale=0.15)

    # Back-compute and draw camera for detections of the selected tag
    if detect_tag_id not in TAG_INFO:
        print(f"[Warning] tag ID {detect_tag_id} not in TAG_INFO; skipping camera computation")
        cam_positions = []
    else:
        tag_pos_map, tag_yaw_deg, _ = TAG_INFO[detect_tag_id]
        cam_positions = []
        for (tag_id, pose_R, pose_t) in detections_data:
            if tag_id != detect_tag_id:
                continue
            cam_pos_map, R_map_cam = compute_camera_in_map_frame(
                pose_R, pose_t, tag_pos_map, tag_yaw_deg)
            cam_positions.append(cam_pos_map)
            draw_camera_in_map(ax, cam_pos_map, R_map_cam, scale=0.18)
            print(f"\n[Map frame] Tag {tag_id}")
            print(f"  Tag pos  : {tag_pos_map}")
            print(f"  Camera   : ({cam_pos_map[0]:.4f}, {cam_pos_map[1]:.4f}, {cam_pos_map[2]:.4f}) m")

    # Axis labels
    ax.set_xlabel('Map-X / east (m)', fontsize=11, fontweight='bold')
    ax.set_ylabel('Map-Y / north (m)', fontsize=11, fontweight='bold')
    ax.set_zlabel('Map-Z / up (m)', fontsize=11, fontweight='bold')
    ax.set_title(
        'AprilTag Map-Frame Visualization\n'
        f'Map origin (0,0,0)  |  Map center ({HALF_FIELD},{HALF_FIELD},0)  |  detecting Tag {detect_tag_id}',
        fontsize=12, fontweight='bold', pad=20)

    # Axis limits to cover full field + camera
    all_pts = list(TAG_INFO.values())  # (pos, yaw, label) tuples
    all_pts_arr = [p for p, _, __ in all_pts] + [np.zeros(3), MAP_CENTER] + cam_positions
    all_pts_arr = np.array(all_pts_arr)
    margin = 0.4
    ax.set_xlim([all_pts_arr[:, 0].min() - margin, all_pts_arr[:, 0].max() + margin])
    ax.set_ylim([all_pts_arr[:, 1].min() - margin, all_pts_arr[:, 1].max() + margin])
    ax.set_zlim([0, all_pts_arr[:, 2].max() + margin])

    # Info box
    info = (
        f"Map-frame mode\n"
        f"Tag 24 RED : {RED_TAG_POS_MAP}  yaw={RED_TAG_YAW_DEG}°\n"
        f"Tag 20 BLUE: {BLUE_TAG_POS_MAP}  yaw={BLUE_TAG_YAW_DEG}°\n"
        f"Map center : {MAP_CENTER}\n"
        f"fx={camera_params[0]:.1f}  fy={camera_params[1]:.1f}\n"
        f"{len(detections_data)} detection(s)"
    )
    ax.text2D(0.02, 0.98, info, transform=ax.transAxes, fontsize=8,
              verticalalignment='top', fontfamily='monospace',
              bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    ax.view_init(elev=30, azim=-110)
    plt.tight_layout()
    return fig, ax


# ---------------------------------------------------------------------------
# Helpers (camera-frame visualization, unchanged)
# ---------------------------------------------------------------------------

def load_calibration(path: str):
    """Load camera intrinsics from YAML file."""
    with open(path) as f:
        data = yaml.safe_load(f)
    K = np.array(data["camera_matrix"], dtype=np.float64)
    dist = np.array(data["dist_coeff"], dtype=np.float64).flatten()
    return K, dist


def convert_to_ftc_pose(pose_R: np.ndarray, pose_t: np.ndarray) -> tuple:
    """
    Convert pupil-apriltags pose to FTC SDK AprilTagPoseFtc format.
    
    FTC uses: AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES
    Position mapping: ftcPose.x = rawPose.x, ftcPose.y = rawPose.z, ftcPose.z = -rawPose.y
    Angle mapping: yaw = -firstAngle, pitch = secondAngle, roll = thirdAngle
    
    Returns: (x, y, z, yaw, roll, pitch) in meters and degrees
    """
    # Position remapping to match FTC SDK
    # ftcPose.x = rawPose.x, ftcPose.y = rawPose.z, ftcPose.z = -rawPose.y
    x = pose_t[0][0]   # rawPose.x
    y = pose_t[2][0]   # rawPose.z
    z = -pose_t[1][0]  # -rawPose.y
    
    # Convert rotation matrix to scipy Rotation object
    rotation = R.from_matrix(pose_R)
    
    # Get Euler angles in YXZ order (intrinsic rotations)
    # scipy uses uppercase for intrinsic rotations
    # For YXZ intrinsic: first rotation is Y (yaw), then X (pitch), then Z (roll)
    euler_angles = rotation.as_euler('YXZ', degrees=True)
    
    # Map to FTC SDK angles:
    # yaw = -firstAngle (rotation about Y)
    # pitch = secondAngle (rotation about X)
    # roll = thirdAngle (rotation about Z)
    yaw = -euler_angles[0]
    pitch = euler_angles[1]
    roll = euler_angles[2]
    
    return x, y, z, yaw, roll, pitch

def draw_camera_frame(ax, scale=0.1):
    """Draw the camera frame at origin with RGB axes (in visualization frame)."""
    # Camera axes in camera frame, remapped to viz frame.
    # Camera: X-right, Y-down, Z-forward
    # Viz:    X-right, Y-forward, Z-up
    origin = np.array([0, 0, 0])

    # cam-X (right) → viz (+x, 0, 0)
    xv = to_viz([scale, 0, 0])
    ax.quiver(*origin, *xv, color='r', linewidth=2, arrow_length_ratio=0.3)
    ax.text(*(origin + xv * 1.2), 'cam-X', color='r', fontsize=9, fontweight='bold')

    # cam-Y (down) → viz (0, 0, -scale)
    yv = to_viz([0, scale, 0])
    ax.quiver(*origin, *yv, color='g', linewidth=2, arrow_length_ratio=0.3)
    ax.text(*(origin + yv * 1.2), 'cam-Y↓', color='g', fontsize=9, fontweight='bold')

    # cam-Z (forward) → viz (0, +scale, 0)
    zv = to_viz([0, 0, scale])
    ax.quiver(*origin, *zv, color='b', linewidth=2, arrow_length_ratio=0.3)
    ax.text(*(origin + zv * 1.2), 'cam-Z→', color='b', fontsize=9, fontweight='bold')

    # Mark camera origin
    ax.scatter(*origin, color='k', s=100, marker='o')
    ax.text(0.02, 0.02, 0.02, 'CAMERA', fontsize=9, fontweight='bold', color='k')


def draw_tag_frame(ax, pose_R: np.ndarray, pose_t: np.ndarray, tag_id: int, scale=0.05):
    """
    Draw AprilTag coordinate frame in 3D space (visualization frame).

    All coordinates are remapped from camera frame (X-right, Y-down, Z-forward)
    to visualization frame (X-right, Y-forward, Z-up) via to_viz().

    Args:
        ax: matplotlib 3D axis
        pose_R: 3x3 rotation matrix (tag→camera transform from pupil-apriltags)
        pose_t: 3x1 translation vector (tag position in camera frame)
        tag_id: AprilTag ID for labeling
        scale: axis length for visualization
    """
    # Tag origin in visualization frame
    origin_viz = to_viz(pose_t.flatten())

    # Tag axes (columns of R) expressed in camera frame → remap to viz frame
    x_axis_viz = to_viz(pose_R[:, 0] * scale)
    y_axis_viz = to_viz(pose_R[:, 1] * scale)
    z_axis_viz = to_viz(pose_R[:, 2] * scale)

    # Draw X axis (red)
    ax.quiver(*origin_viz, *x_axis_viz, color='r', linewidth=2, arrow_length_ratio=0.3, alpha=0.8)
    ax.text(*(origin_viz + x_axis_viz * 1.2), f'X{tag_id}', color='r', fontsize=8)

    # Draw Y axis (green)
    ax.quiver(*origin_viz, *y_axis_viz, color='g', linewidth=2, arrow_length_ratio=0.3, alpha=0.8)
    ax.text(*(origin_viz + y_axis_viz * 1.2), f'Y{tag_id}', color='g', fontsize=8)

    # Draw Z axis (blue) — tag normal pointing toward camera
    ax.quiver(*origin_viz, *z_axis_viz, color='b', linewidth=2, arrow_length_ratio=0.3, alpha=0.8)
    ax.text(*(origin_viz + z_axis_viz * 1.2), f'Z{tag_id}', color='b', fontsize=8)

    # Draw tag face as a square in the tag's XY plane (z=0 in tag frame)
    tag_size = scale * 2
    local_corners = np.array([
        [-tag_size/2, -tag_size/2, 0],
        [ tag_size/2, -tag_size/2, 0],
        [ tag_size/2,  tag_size/2, 0],
        [-tag_size/2,  tag_size/2, 0]
    ])

    # Transform corners: camera frame = R @ local + t, then remap to viz frame
    corners_cam = (pose_R @ local_corners.T).T + pose_t.flatten()
    corners_viz = np.array([to_viz(c) for c in corners_cam])

    # Draw tag outline
    for i in range(4):
        p1 = corners_viz[i]
        p2 = corners_viz[(i + 1) % 4]
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 'c-', linewidth=2, alpha=0.7)

    # Fill tag surface
    ax.add_collection3d(Poly3DCollection([corners_viz], facecolors='cyan', alpha=0.3))

    # Label tag center
    ax.scatter(*origin_viz, color='c', s=80, marker='s')
    ax.text(origin_viz[0], origin_viz[1], origin_viz[2] + scale * 0.6,
            f'Tag {tag_id}', fontsize=9, fontweight='bold', color='c', ha='center')


def draw_detection_overlay(frame, det, pose_R, pose_t, fx, fy, cx, cy, axis_len=0.1):
    """
    Draw 2D overlay of tag detection on camera frame with axes.
    X → red, Y → green, Z → blue
    """
    corners = det.corners.astype(int)
    
    # Draw tag outline
    for i in range(4):
        cv2.line(frame, tuple(corners[i]), tuple(corners[(i + 1) % 4]), (0, 255, 0), 2)
    
    # Center dot
    icx, icy = int(det.center[0]), int(det.center[1])
    cv2.circle(frame, (icx, icy), 5, (0, 0, 255), -1)
    
    # Tag ID label
    cv2.putText(frame, f"ID {det.tag_id}", (icx - 20, icy - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    
    # Project 3D axes onto image
    def project(pt_tag):
        p = pose_R @ np.array(pt_tag, dtype=np.float64) + pose_t.flatten()
        if p[2] <= 0:
            return None
        u = int(fx * p[0] / p[2] + cx)
        v = int(fy * p[1] / p[2] + cy)
        return (u, v)
    
    origin = project([0, 0, 0])
    axes = [
        ([axis_len, 0, 0], (0, 0, 255), "X"),      # red
        ([0, axis_len, 0], (0, 255, 0), "Y"),      # green
        ([0, 0, axis_len], (255, 0, 0), "Z"),      # blue
    ]
    
    if origin is not None:
        for pt_tag, colour, label in axes:
            tip = project(pt_tag)
            if tip is not None:
                cv2.arrowedLine(frame, origin, tip, colour, 2, tipLength=0.2)
                cv2.putText(frame, label, tip,
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)


def print_ftc_format_pose(tag_id: int, pose_R: np.ndarray, pose_t: np.ndarray):
    """
    Print pose in FTC SDK format matching the Java code:
    
    ftcPose.x = rawPose.x
    ftcPose.y = rawPose.z
    ftcPose.z = -rawPose.y
    yaw = -rot.firstAngle
    pitch = rot.secondAngle
    roll = rot.thirdAngle
    """
    x, y, z, yaw, roll, pitch = convert_to_ftc_pose(pose_R, pose_t)
    print(f"\n--- Tag {tag_id} ---")
    print(f"// FTC SDK Pose Format")
    print(f"double poseX = {x:.6f};")
    print(f"double poseY = {y:.6f};")
    print(f"double poseZ = {z:.6f};")
    print(f"double yaw = {yaw:.2f};")
    print(f"double roll = {roll:.2f};")
    print(f"double pitch = {pitch:.2f};")

    """
    Print pose in FTC SDK format matching the Java code:
    
    ftcPose.x = rawPose.x
    ftcPose.y = rawPose.z
    ftcPose.z = -rawPose.y
    yaw = -rot.firstAngle
    pitch = rot.secondAngle
    roll = rot.thirdAngle
    """

def create_3d_visualization(detections_data, camera_params):
    """
    Create a 3D matplotlib visualization with camera as base frame.
    Shows FTC pose format (poseX, poseY, poseZ, poseAX, poseAY, poseAZ) for each tag.
    
    Args:
        detections_data: List of (tag_id, pose_R, pose_t) tuples
        camera_params: (fx, fy, cx, cy) camera intrinsics
    """
    fig = plt.figure(figsize=(16, 10))
    ax = fig.add_subplot(111, projection='3d')
    # Draw camera frame at origin
    draw_camera_frame(ax, scale=0.15)
    
    # Draw each detected tag with its coordinate frame
    for i, (tag_id, pose_R, pose_t) in enumerate(detections_data):
        # Draw the tag's coordinate frame
        draw_tag_frame(ax, pose_R, pose_t, tag_id, scale=0.08)
        
        # Add FTC pose annotation
        add_ftc_pose_annotation(ax, pose_R, pose_t, tag_id, offset=i*0.08)
    # Set axis properties
    # Viz frame: X=right, Y=depth(forward), Z=up
    ax.set_xlabel('X – right (m)', fontsize=11, fontweight='bold')
    ax.set_ylabel('Depth / cam-Z (m)', fontsize=11, fontweight='bold')
    ax.set_zlabel('Up / −cam-Y (m)', fontsize=11, fontweight='bold')

    ax.set_title('AprilTag TF Visualization - FTC Pose Format\n'
                 '(Camera at origin, Y = depth, Z = up)',
                 fontsize=14, fontweight='bold', pad=20)

    # Set equal aspect ratio and reasonable limits
    max_range = 1.0
    if detections_data:
        max_dist = max([np.linalg.norm(pose_t) for _, _, pose_t in detections_data])
        max_range = max(0.5, max_dist * 1.5)

    ax.set_xlim([-max_range * 0.5, max_range * 0.5])
    ax.set_ylim([0, max_range])          # depth ≥ 0
    ax.set_zlim([-max_range * 0.5, max_range * 0.5])
    
    # Add legend/info text box with FTC format explanation
    info_text = "FTC SDK Format:\n"
    info_text += "poseX, poseY, poseZ = position (m)\n"
    info_text += "poseAX, poseAY, poseAZ = angles (deg)\n\n"
    info_text += f"Camera Intrinsics:\nfx={camera_params[0]:.2f}, fy={camera_params[1]:.2f}\n"
    info_text += f"cx={camera_params[2]:.2f}, cy={camera_params[3]:.2f}\n\n"
    info_text += f"Detected {len(detections_data)} tag(s)"
    ax.text2D(0.02, 0.98, info_text, transform=ax.transAxes, fontsize=9,
              verticalalignment='top', fontfamily='monospace',
              bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))
    
    # View from a comfortable angle: slightly above and to the side
    ax.view_init(elev=15, azim=-45)
    
    plt.tight_layout()
    return fig, ax

def add_ftc_pose_annotation(ax, pose_R: np.ndarray, pose_t: np.ndarray, tag_id: int, offset: float = 0):
    """
    Add FTC pose annotation text near the tag in the 3D visualization.
    
    Displays FTC SDK pose: x, y, z (position in meters) and yaw, roll, pitch (angles in degrees)
    
    Args:
        ax: matplotlib 3D axis
        pose_R: 3x3 rotation matrix (tag→camera transform)
        pose_t: 3x1 translation vector (tag position in camera frame)
        tag_id: AprilTag ID for labeling
        offset: vertical offset multiplier to separate multiple tags
    """
    # Get FTC pose values using the conversion function
    x, y, z, yaw, roll, pitch = convert_to_ftc_pose(pose_R, pose_t)
    
    # Tag origin remapped to visualization frame
    origin_viz = to_viz(pose_t.flatten())

    # Create annotation text with FTC format values
    annotation_text = f"Tag {tag_id}\n"
    annotation_text += f"x={x:.3f}m\n"
    annotation_text += f"y={y:.3f}m\n"
    annotation_text += f"z={z:.3f}m\n"
    annotation_text += f"yaw={yaw:.1f}°\n"
    annotation_text += f"roll={roll:.1f}°\n"
    annotation_text += f"pitch={pitch:.1f}°"

    # Position text slightly to the right and above the tag in viz frame
    text_pos = origin_viz + np.array([0.08, 0.0, 0.05 + offset])

    ax.text(*text_pos, annotation_text, fontsize=8, fontweight='bold',
            color='darkblue', verticalalignment='bottom',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='lightyellow',
                      edgecolor='darkblue', alpha=0.9))

    # Draw a small dot at the tag center
    ax.scatter(*origin_viz, color='darkblue', s=50, marker='o')


def main():
    parser = argparse.ArgumentParser(
        description="Visualize AprilTag TF – camera frame or map frame"
    )
    parser.add_argument("--tag-size", type=float, default=0.165,
                        help="Physical tag side length in metres (default: 0.165)")
    parser.add_argument("--family", default="tag36h11",
                        help="AprilTag family (default: tag36h11)")
    parser.add_argument("--camera", type=int, default=0,
                        help="Camera index (default: 0)")
    parser.add_argument("--snapshot", action="store_true",
                        help="Take a single snapshot and visualize instead of continuous stream")
    # Map-frame mode
    parser.add_argument("--map-mode", action="store_true",
                        help="Show camera + both tags in field map frame")
    parser.add_argument(
        "--detect-tag", type=int, default=DEFAULT_DETECT_TAG_ID,
        choices=list(TAG_INFO.keys()),
        help=(f"Tag ID to detect and use for camera localisation "
              f"(default: {DEFAULT_DETECT_TAG_ID}, choices: {list(TAG_INFO.keys())})"),
    )
    args = parser.parse_args()

    if args.map_mode:
        info = TAG_INFO[args.detect_tag]
        print(f"[Map mode]  detecting tag {args.detect_tag} ({info[2]})")
        print(f"            pos = {info[0]}  |  yaw = {info[1]}°")

    # Load camera calibration
    K, _ = load_calibration(CALIBRATION_FILE)
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    
    print(f"Camera matrix:\n{K}")
    print(f"fx={fx}, fy={fy}, cx={cx}, cy={cy}")

    # Initialize detector
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

    print("\n" + "="*50)
    print("Controls:")
    print("  'q' - Quit")
    if args.map_mode:
        print("  's' - Show MAP-FRAME visualization  (camera position in field)")
    else:
        print("  's' - Show camera-frame 3D visualization")
    print("  'c' - Capture single frame (in snapshot mode)")
    print("="*50 + "\n")

    plt.ion()  # Interactive mode for matplotlib
    fig_3d = None

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        detections = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(fx, fy, cx, cy),
            tag_size=args.tag_size,
        )

        detections_data = []
        
        for det in detections:
            pose_R = det.pose_R          # 3x3 rotation matrix (tag→camera)
            pose_t = det.pose_t          # 3x1 translation vector (tag position in camera frame)
            
            # Store detection data for 3D visualization
            detections_data.append((det.tag_id, pose_R.copy(), pose_t.copy()))
            
            # Print in FTC format
            print_ftc_format_pose(det.tag_id, pose_R, pose_t)
            
            # Draw 2D overlay on frame
            draw_detection_overlay(frame, det, pose_R, pose_t, fx, fy, cx, cy,
                                   axis_len=args.tag_size * 0.5)

        # Tag count overlay
        cv2.putText(frame, f"Tags: {len(detections)}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # Instructions overlay
        cv2.putText(frame, "'q' Quit  's' 3D Viz  'c' Capture", (10, frame.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        cv2.imshow("AprilTag Detection - Camera Base Frame", frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            break
        elif key == ord('s') and detections_data:
            # Create/update 3D visualization
            if fig_3d is not None:
                plt.close(fig_3d)
            if args.map_mode:
                fig_3d, ax_3d = create_map_frame_visualization(
                    detections_data, (fx, fy, cx, cy),
                    args.detect_tag, args.tag_size)
            else:
                fig_3d, ax_3d = create_3d_visualization(detections_data, (fx, fy, cx, cy))
            plt.show()
            plt.pause(0.1)
        elif key == ord('c') and args.snapshot:
            # Single capture mode - save image and exit
            if detections_data:
                cv2.imwrite("apriltag_capture.png", frame)
                print(f"Saved apriltag_capture.png")

                if args.map_mode:
                    fig_3d, ax_3d = create_map_frame_visualization(
                        detections_data, (fx, fy, cx, cy),
                        args.detect_tag, args.tag_size)
                else:
                    fig_3d, ax_3d = create_3d_visualization(detections_data, (fx, fy, cx, cy))
                plt.savefig("apriltag_3d_viz.png", dpi=150, bbox_inches='tight')
                print(f"Saved apriltag_3d_viz.png")

                plt.show(block=True)
            break

    cap.release()
    cv2.destroyAllWindows()
    
    if fig_3d is not None:
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    main()
