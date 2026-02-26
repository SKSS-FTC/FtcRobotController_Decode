package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;

import Jama.Matrix;

    /**
     * Shared constants for FTC Decoding robot control.
     * Map reference frame: Bottom left corner of field (0,0,0)
     */
@Config
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
    public static final Matrix FIELD_CENTER_TO_MAP_CORNER = Transformation.createTransformationMatrix(
            new double[][]{{1,0,0},{0,1,0},{0,0,1}},
            FIELD_CENTER_TO_CORNER_TRANSLATION
    );

    // ============================================
    // AprilTag Positions (in meters from field center)
    // Measured values from real DECODE field (~1mm precision)
    // ============================================
    // Tag 20 (BLUE): upper-left  -> field-center X=-1.482 (left wall), Y=+1.413 (upper half)
    // Tag 24 (RED):  upper-right -> field-center X=+1.482 (right wall),  Y=+1.413 (upper half)
    public static final Matrix BLUE_GOAL_TAG_POSITION_CENTER_FRAME = new Matrix(new double[]{-1.482, 1.413, 0.7493}, 3);
    public static final Matrix RED_GOAL_TAG_POSITION_CENTER_FRAME = new Matrix(new double[]{1.482, 1.413, 0.7493}, 3);

    // ============================================
    // AprilTag Rotations (radians, measured from real DECODE field)
    // ============================================
    public static final double BLUE_GOAL_TAG_ROTATION_YAW = Math.toRadians(54);
    public static final double RED_GOAL_TAG_ROTATION_YAW = Math.toRadians(-54);

    // ============================================
    // AprilTag Transformations (4x4 homogeneous transformation matrices from field center to tag frame)
    // ============================================
    // Tag rotation = Rz(yaw) * Rx(90deg): face normal points horizontally toward field center.
    public static final Matrix BLUE_GOAL_TAG_CENTER_FRAME;
    static {
        double y = BLUE_GOAL_TAG_ROTATION_YAW;
        BLUE_GOAL_TAG_CENTER_FRAME = Transformation.createTransformationMatrix(
            new double[][]{
                { Math.cos(y), 0,  Math.sin(y)},
                { Math.sin(y), 0, -Math.cos(y)},
                { 0,           1,  0          }
            },
            new double[]{
                BLUE_GOAL_TAG_POSITION_CENTER_FRAME.get(0, 0),
                BLUE_GOAL_TAG_POSITION_CENTER_FRAME.get(1, 0),
                BLUE_GOAL_TAG_POSITION_CENTER_FRAME.get(2, 0)
            }
        );
    }

    public static final Matrix RED_GOAL_TAG_CENTER_FRAME;
    static {
        double y = RED_GOAL_TAG_ROTATION_YAW;
        RED_GOAL_TAG_CENTER_FRAME = Transformation.createTransformationMatrix(
            new double[][]{
                { Math.cos(y), 0,  Math.sin(y)},
                { Math.sin(y), 0, -Math.cos(y)},
                { 0,           1,  0          }
            },
            new double[]{
                RED_GOAL_TAG_POSITION_CENTER_FRAME.get(0, 0),
                RED_GOAL_TAG_POSITION_CENTER_FRAME.get(1, 0),
                RED_GOAL_TAG_POSITION_CENTER_FRAME.get(2, 0)
            }
        );
    }

    /**
     * AprilTag transformation matrices in MAP CORNER frame (4x4)
     * T_corner_tag = inv(T_center_corner) * T_center_tag
     */
    public static final Matrix BLUE_GOAL_TAG_MAP_FRAME = BLUE_GOAL_TAG_CENTER_FRAME.inverse().times(FIELD_CENTER_TO_MAP_CORNER);
    public static final Matrix RED_GOAL_TAG_MAP_FRAME  =  RED_GOAL_TAG_CENTER_FRAME.inverse().times(FIELD_CENTER_TO_MAP_CORNER);

    /**
     * Shooting target positions in map corner frame (4x1 matrices)
     */
    public static final Matrix BLUE_SHOOTING_TARGET_POSITION_MAP = new Matrix(new double[]{17*0.0254, 138*0.0254, 0.984, 1.0}, 4);
    public static final Matrix RED_SHOOTING_TARGET_POSITION_MAP = new Matrix(new double[]{132*0.0254, 138*0.0254, 0.984, 1.0}, 4);

    // ============================================
    // Shooter PID/SMC Control Constants
    // ============================================
    public static double SHOOTER_PID_KP = 0.000002;
    public static double SHOOTER_PID_KI = 0.0;
    public static double SHOOTER_PID_KD = 0.00001;
    public static double SHOOTER_SMC_LAMBDA = 0.001;//0.000000001
    public static double SHOOTER_SMC_ETA = 0.01;//0.0001
    public static boolean SHOOTER_PID = true;
    public static double shooterTargetAngularVelocity = 10;
    public static double shooterMinCheckSecond = 0.05;

    // ============================================
    // Camera Calibration Constants
    // ============================================
    public static double CAMERA_FOCAL_LENGTH = 500.0;
    public static double CALIBRATION_ACTUAL_DISTANCE = 0.0;
    public static double KNOWN_OBJECT_WIDTH = 0.1524;
    public static int sliderDriveEncoderValue = 100;
    public static int sliderParkEncoderValue = 100;
    public static double leftSliderServoClose = 0;
    public static double rightSliderServoClose = 0;
    public static double leftSliderServoOpen = 1;
    public static double rightSliderServoOpen = 1;
    public static double cameraAngleOfElevation = (double) 18 /180 *Math.PI;
    public static double angleTunerAngle = 0.04;
}
