package org.firstinspires.ftc.teamcode.subsystem;

import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import Jama.Matrix;

public class Transformation {

    private static final Map<Integer, Matrix> TAG_TO_MAP_TRANSFORMS = new HashMap<>();

    public static Matrix H_CAMERA_TO_SHOOTER;
    public static Matrix H_SHOOTER_TO_BASE;

    private static void ensureInitialized() {
        if (H_CAMERA_TO_SHOOTER == null || H_SHOOTER_TO_BASE == null) {
            initialize();
        }
    }

    public static void initialize() {
        double a = RobotState.ShooterAngleRobot;
        double y = Constants.cameraAngleOfElevation * Math.PI;
        H_CAMERA_TO_SHOOTER = createTransformationMatrix(
                new double[][]{
                        // a around z, b around y, y around x
                        //cos(b) = cos(0) = 1, sin(b) = sin(0) = 0
                        {Math.cos(a),-1*Math.sin(a)*Math.cos(y),Math.sin(a)*Math.sin(y)},
                        {Math.sin(a),Math.cos(a)*Math.cos(y),-1*Math.cos(a)*Math.sin(y)},
                        {0,Math.sin(y),Math.cos(y)}
                },
                new double[]{0.081,-0.10323,0.184}
        );;
        H_SHOOTER_TO_BASE = createTransformationMatrix(
                new double[][]{
                        {1,0,0},
                        {0,1,0},
                        {0,0,1}
                },
                new double[]{0,-0.08,0.24}
        );
        TAG_TO_MAP_TRANSFORMS.clear();
    }

    public static void registerAprilTag(int tagId, Matrix tagToMap) {
        ensureInitialized();
        if (tagToMap == null) {
            throw new IllegalArgumentException("tagToMap must not be null");
        }
        if (tagToMap.getRowDimension() != 4 || tagToMap.getColumnDimension() != 4) {
            throw new IllegalArgumentException("tagToMap must be a 4x4 homogeneous transformation matrix");
        }
        TAG_TO_MAP_TRANSFORMS.put(tagId, tagToMap);
    }

    public static Matrix getTagToMapTransform(int tagId) {
        return TAG_TO_MAP_TRANSFORMS.get(tagId);
    }

    public static boolean isTagRegistered(int tagId) {
        return TAG_TO_MAP_TRANSFORMS.containsKey(tagId);
    }

    public static RobotPose getRobotPoseInMap(int tagId, Matrix H_cameraToTag) {
        ensureInitialized();
        if (!isTagRegistered(tagId)) {
            return null;
        }

        if (H_cameraToTag == null || H_cameraToTag.getRowDimension() != 4 || H_cameraToTag.getColumnDimension() != 4) {
            return null;
        }

        Matrix H_tagToMap = getTagToMapTransform(tagId);

        // ── Notation ────────────────────────────────────────────
        // Each "X_TO_Y" variable stores the pose-of-X-in-Y, i.e. ^Y T_X,
        // which transforms points FROM frame X INTO frame Y.
        //
        // Actual matrices:
        //   H_cameraToTag        (AprilTagReader)  = ^cam T_tag
        //   H_tagToMap            (Constants)       = ^map T_tag
        //   H_CAMERA_TO_SHOOTER                     = ^shooter T_cam
        //   H_SHOOTER_TO_BASE                       = ^base T_shooter
        //
        // Goal: ^map T_base  (robot base pose in map frame).
        //
        // Chain:  ^map T_base = ^map T_tag  ×  ^tag T_cam  ×  ^cam T_base
        //
        //   ^tag T_cam  = (^cam T_tag)^-1              = H_cameraToTag^-1
        //   ^base T_cam = ^base T_shooter × ^shooter T_cam
        //               = H_SHOOTER_TO_BASE × H_CAMERA_TO_SHOOTER
        //   ^cam T_base = (^base T_cam)^-1
        // ────────────────────────────────────────────────────────

        // tag ← camera
        Matrix H_tagFromCam = H_cameraToTag.inverse();

        // base ← camera  (body chain, correct multiplication order)
        // Matrix H_baseFromCam = H_SHOOTER_TO_BASE.times(H_CAMERA_TO_SHOOTER);

        // Base from cam is zero for testing
        Matrix H_baseFromCam = createIdentityMatrix();


        // camera ← base
        Matrix H_camFromBase = H_baseFromCam.inverse();

        // map ← base  (full chain)
        Matrix H_mapFromBase = H_tagToMap.times(H_tagFromCam).times(H_camFromBase);

        return extractRotationAndTranslation(H_mapFromBase);
    }

    public static RobotPose getRobotPoseInMapFromMultipleTags(List<TagDetection> tagDetections) {
        if (tagDetections == null || tagDetections.isEmpty()) {
            return null;
        }

        List<RobotPose> validPoses = new ArrayList<>();

        for (TagDetection detection : tagDetections) {
            RobotPose pose = getRobotPoseInMap(detection.tagId, detection.cameraToTag);
            if (pose != null) {
                validPoses.add(pose);
            }
        }

        if (validPoses.isEmpty()) {
            return null;
        }

        return averagePoses(validPoses);
    }

    private static RobotPose averagePoses(List<RobotPose> poses) {
        int n = poses.size();

        double[] avgTranslation = new double[3];
        for (int i = 0; i < 3; i++) {
            double sum = 0;
            for (RobotPose pose : poses) {
                sum += pose.translation[i];
            }
            avgTranslation[i] = sum / n;
        }

        double[][] avgRotation = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                double sum = 0;
                for (RobotPose pose : poses) {
                    sum += pose.rotationMatrix[i][j];
                }
                avgRotation[i][j] = sum / n;
            }
        }

        return new RobotPose(avgRotation, avgTranslation);
    }

    public static Matrix computeInverse(Matrix H) {
        if (H == null) {
            throw new IllegalArgumentException("Matrix must not be null");
        }
        if (H.getRowDimension() != 4 || H.getColumnDimension() != 4) {
            throw new IllegalArgumentException("Matrix must be 4x4 homogeneous transformation matrix");
        }

        return H.inverse();
    }

    public static RobotPose extractRotationAndTranslation(Matrix H) {
        if (H == null) {
            throw new IllegalArgumentException("Matrix must not be null");
        }
        if (H.getRowDimension() != 4 || H.getColumnDimension() != 4) {
            throw new IllegalArgumentException("Matrix must be 4x4 homogeneous transformation matrix");
        }

        double[][] rotationMatrix = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                rotationMatrix[i][j] = H.get(i, j);
            }
        }

        double[] translation = new double[3];
        for (int i = 0; i < 3; i++) {
            translation[i] = H.get(i, 3);
        }

        return new RobotPose(rotationMatrix, translation);
    }

    /**
     * Build the R_map_tag rotation matrix for a wall-mounted AprilTag.
     *
     * <p>Matches the Python {@code build_R_map_tag(yaw_deg)} used in
     * {@code visualize_apriltag_tf.py}.
     *
     * <p>OpenCV / pupil-apriltags tag frame:
     * <ul>
     *   <li>tag-X – right across the face</li>
     *   <li>tag-Y – <em>down</em> on the face</li>
     *   <li>tag-Z – <em>into the wall</em> (away from the camera)</li>
     * </ul>
     *
     * <p>FTC map frame: X-right (east), Y-north (forward), Z-up.
     *
     * <p>Reference orientation (yaw=0): tag on the south wall, facing north.
     * After rotating {@code yawRad} around map-Z:
     * <pre>
     *   col-0 (tag-X) = ( cos(ψ),  sin(ψ),  0)
     *   col-1 (tag-Y) = (      0,       0, -1)
     *   col-2 (tag-Z) = (-sin(ψ),  cos(ψ),  0)   ← points INTO the wall
     * </pre>
     *
     * <p>Verification:
     * <ul>
     *   <li>ψ = -54° → tag-Z ≈ (+0.809, +0.588, 0)  [RED  tag – upper-right wall]</li>
     *   <li>ψ = +54° → tag-Z ≈ (-0.809, +0.588, 0)  [BLUE tag – upper-left  wall]</li>
     * </ul>
     * Camera (front of tag) is in the −tag-Z direction → inside the field ✓
     *
     * @param yawRad yaw angle in <b>radians</b> (positive = CCW when viewed from above)
     * @return 3×3 rotation matrix as {@code double[3][3]}
     */
    public static double[][] buildRMapTag(double yawRad) {
        double c = Math.cos(yawRad);
        double s = Math.sin(yawRad);
        return new double[][] {
            { c,  0, -s },
            { s,  0,  c },
            { 0, -1,  0 },
        };
    }

    public static Matrix createIdentityMatrix() {
        return Matrix.identity(4, 4);
    }

    public static Matrix createTransformationMatrix(double[][] rotationMatrix, double[] translation) {
        if (rotationMatrix.length != 3 || rotationMatrix[0].length != 3) {
            throw new IllegalArgumentException("Rotation matrix must be 3x3");
        }
        if (translation.length != 3) {
            throw new IllegalArgumentException("Translation vector must be length 3");
        }

        Matrix H = new Matrix(4, 4);

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                H.set(i, j, rotationMatrix[i][j]);
            }
        }

        for (int i = 0; i < 3; i++) {
            H.set(i, 3, translation[i]);
        }

        H.set(3, 0, 0);
        H.set(3, 1, 0);
        H.set(3, 2, 0);
        H.set(3, 3, 1);

        return H;
    }

    public static class RobotPose {
        public final double[][] rotationMatrix;
        public final double[] translation;

        public RobotPose(double[][] rotationMatrix, double[] translation) {
            this.rotationMatrix = rotationMatrix;
            this.translation = translation;
        }

        public double[] getPosition() {
            return translation.clone();
        }

        public double[][] getOrientationMatrix() {
            return rotationMatrix;
        }

        /**
         * Extract ZYX Euler angles (radians) from the rotation matrix.
         * Convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
         * @return double[] {roll, pitch, yaw} in radians
         */
        public double[] getRPY() {
            double[][] R = rotationMatrix;
            double yaw   = Math.atan2(R[1][0], R[0][0]);
            double pitch = Math.atan2(-R[2][0], Math.sqrt(R[2][1] * R[2][1] + R[2][2] * R[2][2]));
            double roll  = Math.atan2(R[2][1], R[2][2]);
            return new double[]{roll, pitch, yaw};
        }
    }

    public static class TagDetection {
        public final int tagId;
        public final Matrix cameraToTag;

        public TagDetection(int tagId, Matrix cameraToTag) {
            this.tagId = tagId;
            this.cameraToTag = cameraToTag;
        }
    }
}
