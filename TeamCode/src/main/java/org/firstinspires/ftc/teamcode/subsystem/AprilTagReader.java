package org.firstinspires.ftc.teamcode.subsystem;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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
        try {
            return Transformation.computeInverse(cameraToTag);
        } catch (RuntimeException ignored) {
            return Transformation.createIdentityMatrix();
        }
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

        // Create rotation matrix from roll-pitch-yaw
        Matrix rotationMatrix = rotationMatrixFromRpy(roll, pitch, yaw);

        double[] translation = new double[]{x, y, z};

        return Transformation.createTransformationMatrix(rotationMatrix.getArrayCopy(), translation);
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
}
