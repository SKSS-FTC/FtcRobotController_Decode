package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.geometry.Pose;

import Jama.Matrix;

    /**
     * Shared constants for FTC Decoding robot control.
     * Map reference frame: Bottom left corner of field (0,0,0)
     */
public class Constants {
    private Constants() {}

    // ============================================
    public static final double FIELD_SIZE_METERS = 3.658;
    public static final double HALF_FIELD_METERS = 1.829;
    public static final double TAG_HEIGHT_METERS = 0.7493;

    // ============================================
    // AprilTag IDs
    // ============================================
    public static final int BLUE_GOAL_TAG_ID = 20;
    public static final int RED_GOAL_TAG_ID = 24;

    // ============================================
    // Map Center to Map Corner Transformation
    // ============================================
    public static final double[] FIELD_CENTER_TO_CORNER_TRANSLATION = {-HALF_FIELD_METERS, -HALF_FIELD_METERS, 0.0};

    // ============================================
    // AprilTag Positions (in meters from field center)
    // Measured values from real DECODE field (~1mm precision)
    // ============================================
    public static final Matrix BLUE_GOAL_TAG_POSITION_CENTER_FRAME = new Matrix(new double[]{-1.482, -1.413, 0.7493}, 3);
    public static final Matrix RED_GOAL_TAG_POSITION_CENTER_FRAME = new Matrix(new double[]{-1.482, 1.413, 0.7493}, 3);

    // ============================================
    // AprilTag Rotations (radians, measured from real DECODE field)
    // ============================================
    public static final double BLUE_GOAL_TAG_ROTATION_YAW = Math.toRadians(54);
    public static final double RED_GOAL_TAG_ROTATION_YAW = Math.toRadians(-54);

    // ============================================
    // AprilTag Rotation Matrices (3x3)
    // ============================================

    /**
     * AprilTag rotation matrices - rotations around Z axis
     */
    public static final double[][] BLUE_GOAL_TAG_ROTATION_MATRIX = {
        {Math.cos(BLUE_GOAL_TAG_ROTATION_YAW), -Math.sin(BLUE_GOAL_TAG_ROTATION_YAW), 0.0},
        {Math.sin(BLUE_GOAL_TAG_ROTATION_YAW),  Math.cos(BLUE_GOAL_TAG_ROTATION_YAW), 0.0},
        {0.0,                                        0.0,                              1.0}
    };

    public static final Matrix ROTATION_MATRIX_TAG_20;
    static {
        ROTATION_MATRIX_TAG_20 = new Matrix(new double[][]{
            {Math.cos(BLUE_GOAL_TAG_ROTATION_YAW), -Math.sin(BLUE_GOAL_TAG_ROTATION_YAW), 0},
            {Math.sin(BLUE_GOAL_TAG_ROTATION_YAW),  Math.cos(BLUE_GOAL_TAG_ROTATION_YAW), 0},
            {0,                                 0,                                        1}
        });
    }

    public static final Matrix ROTATION_MATRIX_TAG_24;
    static {
        ROTATION_MATRIX_TAG_24 = new Matrix(new double[][]{
            {Math.cos(RED_GOAL_TAG_ROTATION_YAW), -Math.sin(RED_GOAL_TAG_ROTATION_YAW), 0},
            {Math.sin(RED_GOAL_TAG_ROTATION_YAW),  Math.cos(RED_GOAL_TAG_ROTATION_YAW), 0},
            {0,                                 0,                                        1}
        });
    }
    public static final double[][] RED_GOAL_TAG_ROTATION_MATRIX = {
        {Math.cos(RED_GOAL_TAG_ROTATION_YAW), -Math.sin(RED_GOAL_TAG_ROTATION_YAW), 0.0},
        {Math.sin(RED_GOAL_TAG_ROTATION_YAW),  Math.cos(RED_GOAL_TAG_ROTATION_YAW), 0.0},
        {0.0,                                       0.0,                              1.0}
    };

    // ============================================
    // AprilTag Transformation Matrices (4x4 homogeneous)
    // ============================================

    /**
     * Transformation matrices from field center to map corner
     * Map origin: bottom left corner of field
     */
    public static final Matrix FIELD_CENTER_TO_MAP_CORNER;
    static {
        FIELD_CENTER_TO_MAP_CORNER = Transformation.createTransformationMatrix(
            new double[][]{
                {1, 0, 0},
                {0, 1, 0},
                {0, 0, 1}
            },
            FIELD_CENTER_TO_CORNER_TRANSLATION
        );
    }

    /**
     * AprilTag transformation matrices - converts AprilTag frame to map corner frame
     */
    public static final Matrix TAG_20_TO_MAP = computeTagToMapTransform(
        ROTATION_MATRIX_TAG_20, 
        BLUE_GOAL_TAG_POSITION_CENTER_FRAME
    );

    public static final Matrix TAG_24_TO_MAP = computeTagToMapTransform(
        ROTATION_MATRIX_TAG_24,
        RED_GOAL_TAG_POSITION_CENTER_FRAME
    );

    public static Matrix getTag20ToMapTransform() {
        return TAG_20_TO_MAP;
    }

    public static Matrix getTag24ToMapTransform() {
        return TAG_24_TO_MAP;
    }

    private static Matrix computeTagToMapTransform(Matrix rotation, Matrix position) {
        double[] positionArray = new double[3];
        for (int i = 0; i < 3; i++) {
            positionArray[i] = position.get(i, 0);
        }
        Matrix tagToCenter = Transformation.createTransformationMatrix(rotation.getArrayCopy(), positionArray);
        return FIELD_CENTER_TO_MAP_CORNER.times(tagToCenter);
    }

    /**
     * AprilTag positions in map corner frame (from field center using .times())
     */
    public static final Matrix BLUE_GOAL_TAG_POSITION_MAP_FRAME = FIELD_CENTER_TO_MAP_CORNER.times(BLUE_GOAL_TAG_POSITION_CENTER_FRAME);
    public static final Matrix RED_GOAL_TAG_POSITION_MAP_FRAME = FIELD_CENTER_TO_MAP_CORNER.times(RED_GOAL_TAG_POSITION_CENTER_FRAME);

    /**
     * Shooting target positions in map corner frame (4x1 matrices)
     */
    public static final Matrix BLUE_SHOOTING_TARGET_POSITION_MAP = new Matrix(new double[]{17*0.0254, 138*0.0254, 0.984, 0}, 4);
    public static final Matrix RED_SHOOTING_TARGET_POSITION_MAP = new Matrix(new double[]{132*0.0254, 138*0.0254, 0.984, 0}, 4);

    public static Pose currentPose = new Pose(0,0,0);
    public static int shooterEncoder = 10000;
}
