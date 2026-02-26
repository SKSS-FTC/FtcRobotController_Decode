package org.firstinspires.ftc.teamcode.subsystem;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

import Jama.Matrix;

/**
 * AprilTag processor for extracting AprilTag detections and transformations.
 */
public class AprilTagReader {
    private AprilTagProcessor aprilTagProcessor = null;

    /**
     * Set AprilTag processor (typically configured with VisionPortal in team code).
     *
     * @param processor AprilTagProcessor instance
     */
    public void setProcessor(AprilTagProcessor processor) {
        this.aprilTagProcessor = processor;
    }

    /**
     * Get all AprilTag detections from the processor.
     *
     * @return List of AprilTag detections
     */
    public List<AprilTagDetection> getDetections() {
        if (aprilTagProcessor == null) {
            return new ArrayList<>();
        }
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        return detections != null ? detections : new ArrayList<>();
    }

    /**
     * Get detection for a specific AprilTag ID.
     *
     * @param tagId AprilTag ID to filter
     * @return First detection matching the tag ID, or null if not found
     */
    public AprilTagDetection getDetection(int tagId) {
        List<AprilTagDetection> detections = getDetectionsForTag(tagId);
        return detections.isEmpty() ? null : detections.get(0);
    }

    /**
     * Get detections for a specific AprilTag ID.
     *
     * @param tagId AprilTag ID to filter
     * @return List of detections matching the tag ID
     */
    public List<AprilTagDetection> getDetectionsForTag(int tagId) {
        List<AprilTagDetection> filtered = new ArrayList<>();
        for (AprilTagDetection detection : getDetections()) {
            if (detection.id == tagId) {
                filtered.add(detection);
            }
        }
        return filtered;
    }

    /**
     * Get detections for multiple AprilTag IDs.
     *
     * @param tagIds Array of AprilTag IDs to filter
     * @return List of detections matching any of the tag IDs
     */
    public List<AprilTagDetection> getDetectionsForTags(int[] tagIds) {
        List<AprilTagDetection> filtered = new ArrayList<>();
        for (AprilTagDetection detection : getDetections()) {
            for (int tagId : tagIds) {
                if (detection.id == tagId) {
                    filtered.add(detection);
                    break;
                }
            }
        }
        return filtered;
    }

    /**
     * Get camera-to-tag transformation matrix for a specific AprilTag ID.
     *
     * @param tagId AprilTag ID
     * @return 4x4 homogeneous transformation matrix [R t; 0 1] (meters, radians), or identity if not found
     */
    public Matrix getCameraToTagMatrix(int tagId) {
        AprilTagDetection detection = getDetection(tagId);
        return detectionToCameraToTagMatrix(detection);
    }

    /**
     * Get tag-to-camera transformation matrix for a specific AprilTag ID.
     *
     * @param tagId AprilTag ID
     * @return 4x4 homogeneous transformation matrix [R t; 0 1] (meters, radians)
     */
    public Matrix getTagToCameraMatrix(int tagId) {
        Matrix cameraToTag = getCameraToTagMatrix(tagId);
        return Transformation.computeInverse(cameraToTag);
    }

    /**
     * Get camera-to-tag transformation matrices for multiple AprilTag IDs.
     *
     * @param tagIds Array of AprilTag IDs
     * @return List of 4x4 homogeneous transformation matrices
     */
    public List<Matrix> getCameraToTagMatrices(int[] tagIds) {
        List<Matrix> matrices = new ArrayList<>();
        List<AprilTagDetection> detections = getDetectionsForTags(tagIds);
        for (AprilTagDetection detection : detections) {
            matrices.add(detectionToCameraToTagMatrix(detection));
        }
        return matrices;
    }

    /**
     * Convert AprilTagDetection to camera-to-tag transformation matrix (4x4 homogeneous).
     *
     * @param detection AprilTag detection with ftcPose (units: inches/degrees)
     * @return 4x4 homogeneous transformation matrix [R t; 0 1] (meters, radians)
     */
    public Matrix detectionToCameraToTagMatrix(AprilTagDetection detection) {
        if (detection == null || detection.ftcPose == null) {
            return Transformation.createIdentityMatrix();
        }

        // ftcPose provides: X, Y, Z (inches), pitch, roll, yaw (degrees)
        // Convert to meters and radians
        double x = detection.ftcPose.x * 0.0254;
        double y = detection.ftcPose.y * 0.0254;
        double z = detection.ftcPose.z * 0.0254;
        double pitch = Math.toRadians(detection.ftcPose.pitch);
        double roll = Math.toRadians(detection.ftcPose.roll);
        double yaw = Math.toRadians(detection.ftcPose.yaw);

        if (!isFinite(x) || !isFinite(y) || !isFinite(z)
                || !isFinite(pitch) || !isFinite(roll) || !isFinite(yaw)) {
            return Transformation.createIdentityMatrix();
        }

        Matrix R = rotationMatrixFromRpy(roll, pitch, yaw);
        Matrix H = new Matrix(4, 4);
        H.setMatrix(0, 2, 0, 2, R);
        H.set(0, 3, x);
        H.set(1, 3, y);
        H.set(2, 3, z);
        H.set(3, 0, 0);
        H.set(3, 1, 0);
        H.set(3, 2, 0);
        H.set(3, 3, 1);
        return H;
    }

    private boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    /**
     * Create rotation matrix from roll, pitch, yaw (ZYX Euler angles).
     *
     * @param roll  Rotation around x-axis (radians)
     * @param pitch Rotation around y-axis (radians)
     * @param yaw   Rotation around z-axis (radians)
     * @return 3x3 rotation matrix
     */
    private Matrix rotationMatrixFromRpy(double roll, double pitch, double yaw) {
        double cr = Math.cos(roll);
        double sr = Math.sin(roll);
        double cp = Math.cos(pitch);
        double sp = Math.sin(pitch);
        double cy = Math.cos(yaw);
        double sy = Math.sin(yaw);

        double[][] r = new double[3][3];
        // R = Rz(yaw) * Ry(pitch) * Rx(roll)
        r[0][0] = cy * cp;
        r[0][1] = cy * sp * sr - sy * cr;
        r[0][2] = cy * sp * cr + sy * sr;

        r[1][0] = sy * cp;
        r[1][1] = sy * sp * sr + cy * cr;
        r[1][2] = sy * sp * cr - cy * sr;

        r[2][0] = -sp;
        r[2][1] = cp * sr;
        r[2][2] = cp * cr;

        return new Matrix(r);
    }

    /**
     * Get the underlying AprilTag processor (for advanced usage).
     *
     * @return AprilTagProcessor instance
     */
    public AprilTagProcessor getProcessor() {
        return aprilTagProcessor;
    }

    // -------------------------------------------------------------------------
    // Drawing helpers – ported from detect_apriltag.py  draw_detection()
    // -------------------------------------------------------------------------

    /**
     * Draw tag outline, centre dot, ID label and colour-coded 3-D axis arrows
     * on an Android {@link Canvas}.
     *
     * <p>Axis colours match the Python reference: X → red, Y → green, Z → blue.
     * Arrows are drawn in the <em>tag</em> coordinate frame and projected with
     * the pinhole model.
     *
     * <p>Call this from a {@code VisionProcessor.onDrawFrame()} implementation
     * (or similar) after scaling image-pixel coordinates to canvas coordinates
     * via {@code scaleBmpPxToCanvasPx}.
     *
     * @param canvas   Android canvas to draw on
     * @param detection AprilTag detection (must have a valid {@code ftcPose})
     * @param fx       Focal length x (pixels, in canvas/bitmap space)
     * @param fy       Focal length y (pixels, in canvas/bitmap space)
     * @param cx       Principal point x (pixels)
     * @param cy       Principal point y (pixels)
     * @param axisLen  Axis arrow length in metres (e.g. {@code tagSize * 0.6})
     */
    public void drawDetection(Canvas canvas, AprilTagDetection detection,
                              float fx, float fy, float cx, float cy,
                              float axisLen) {
        if (detection == null) return;

        // --- tag → camera transform (R, t) for 3-D projection ---
        // detectionToCameraToTagMatrix returns H_cameraToTag; invert to get H_tagToCamera.
        Matrix H_cameraToTag = detectionToCameraToTagMatrix(detection);
        Matrix H_tagToCamera = Transformation.computeInverse(H_cameraToTag);

        double[][] R = H_tagToCamera.getMatrix(0, 2, 0, 2).getArray();
        double tx = H_tagToCamera.get(0, 3);
        double ty = H_tagToCamera.get(1, 3);
        double tz = H_tagToCamera.get(2, 3);

        // --- Paint objects ---
        Paint paintGreen = makePaint(Color.GREEN, 4f);
        Paint paintRed   = new Paint(Paint.ANTI_ALIAS_FLAG);
        paintRed.setColor(Color.RED);
        paintRed.setStyle(Paint.Style.FILL);

        // --- Tag corners (outline) ---
        Point[] corners = detection.corners;
        if (corners != null && corners.length == 4) {
            for (int i = 0; i < 4; i++) {
                Point c0 = corners[i];
                Point c1 = corners[(i + 1) % 4];
                canvas.drawLine((float) c0.x, (float) c0.y,
                                (float) c1.x, (float) c1.y, paintGreen);
            }
        }

        // --- Centre dot ---
        Point center = detection.center;
        if (center != null) {
            canvas.drawCircle((float) center.x, (float) center.y, 8f, paintRed);
        }

        // --- Tag ID label ---
        Paint textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        textPaint.setColor(Color.YELLOW);
        textPaint.setStyle(Paint.Style.FILL);
        textPaint.setTextSize(36f);
        float labelX = center != null ? (float) center.x - 20f : 0f;
        float labelY = center != null ? (float) center.y - 14f : 36f;
        canvas.drawText("ID " + detection.id, labelX, labelY, textPaint);

        // --- 3-D axis arrows (X=red, Y=green, Z=blue) ---
        // Arrows start from detection.center (2D pixel coords) for stability.
        // Direction is taken from the 3D projection so perspective is preserved:
        //   tip_2d = center + (proj(axisPoint) - proj(origin))
        float[] proj3dOrigin = projectPoint(new double[]{0, 0, 0}, R, tx, ty, tz, fx, fy, cx, cy);
        // Fall back to projected origin if center is unavailable
        float ox = (center != null) ? (float) center.x : (proj3dOrigin != null ? proj3dOrigin[0] : 0f);
        float oy = (center != null) ? (float) center.y : (proj3dOrigin != null ? proj3dOrigin[1] : 0f);
        if (proj3dOrigin == null) return;

        double[][]  axisPts    = { {axisLen,0,0},  {0,axisLen,0},  {0,0,axisLen} };
        int[]       axisColors = { Color.RED, Color.GREEN, Color.BLUE };
        String[]    axisLabels = { "X",        "Y",         "Z"        };

        for (int i = 0; i < 3; i++) {
            float[] proj3dTip = projectPoint(axisPts[i], R, tx, ty, tz, fx, fy, cx, cy);
            if (proj3dTip == null) continue;
            // Translate direction to start from the 2D center
            float tipX = ox + (proj3dTip[0] - proj3dOrigin[0]);
            float tipY = oy + (proj3dTip[1] - proj3dOrigin[1]);
            Paint p = makePaint(axisColors[i], 4f);
            drawArrowedLine(canvas, ox, oy, tipX, tipY, 0.2f, p);
            Paint labelPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
            labelPaint.setColor(axisColors[i]);
            labelPaint.setStyle(Paint.Style.FILL);
            labelPaint.setTextSize(30f);
            canvas.drawText(axisLabels[i], tipX, tipY, labelPaint);
        }
    }

    /**
     * Project a point from the tag frame into image (canvas) coordinates.
     *
     * @param pt  3-D point in tag frame [x, y, z]
     * @param R   3×3 rotation matrix (tag → camera) as {@code double[3][3]}
     * @param tx  Camera translation x
     * @param ty  Camera translation y
     * @param tz  Camera translation z
     * @return {@code float[]{u, v}} in pixels, or {@code null} if behind camera
     */
    private float[] projectPoint(double[] pt, double[][] R,
                                 double tx, double ty, double tz,
                                 float fx, float fy, float cx, float cy) {
        double px = R[0][0]*pt[0] + R[0][1]*pt[1] + R[0][2]*pt[2] + tx;
        double py = R[1][0]*pt[0] + R[1][1]*pt[1] + R[1][2]*pt[2] + ty;
        double pz = R[2][0]*pt[0] + R[2][1]*pt[1] + R[2][2]*pt[2] + tz;
        if (pz <= 0) return null;
        float u = (float) (fx * px / pz + cx);
        float v = (float) (fy * py / pz + cy);
        return new float[]{u, v};
    }

    /**
     * Draw a line with a simple arrowhead at the tip, like OpenCV's
     * {@code cv2.arrowedLine}.
     *
     * @param tipFraction  Arrowhead size as a fraction of total line length (0–1)
     */
    private void drawArrowedLine(Canvas canvas,
                                 float x1, float y1, float x2, float y2,
                                 float tipFraction, Paint paint) {
        canvas.drawLine(x1, y1, x2, y2, paint);

        float dx = x2 - x1;
        float dy = y2 - y1;
        float len = (float) Math.hypot(dx, dy);
        if (len < 1f) return;

        float tipLen = len * tipFraction;
        float nx = dx / len;
        float ny = dy / len;

        // Back-direction unit vector rotated ±25°
        double angle = Math.toRadians(25);
        float cos = (float) Math.cos(angle);
        float sin = (float) Math.sin(angle);

        // Rotate (-nx, -ny) by +angle
        float ax1 = -nx * cos - (-ny) * sin;
        float ay1 = -nx * sin + (-ny) * cos;
        // Rotate (-nx, -ny) by -angle
        float ax2 = -nx * cos + (-ny) * sin;
        float ay2 =  nx * sin + (-ny) * cos;

        canvas.drawLine(x2, y2, x2 + ax1 * tipLen, y2 + ay1 * tipLen, paint);
        canvas.drawLine(x2, y2, x2 + ax2 * tipLen, y2 + ay2 * tipLen, paint);
    }

    /** Convenience factory for stroke-style {@link Paint}. */
    private Paint makePaint(int color, float strokeWidth) {
        Paint p = new Paint(Paint.ANTI_ALIAS_FLAG);
        p.setColor(color);
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(strokeWidth);
        return p;
    }
}
