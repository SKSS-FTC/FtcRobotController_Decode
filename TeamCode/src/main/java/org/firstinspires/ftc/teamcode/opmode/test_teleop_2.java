package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Localizer;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Slider;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp(name = "test_teleop_2",group = "TeleOP")
public class test_teleop_2 extends LinearOpMode {
    private DcMotor leftUp, rightUp, leftDown, rightDown;
    private double relativeTargetAngle, joystickMagnitude, outputX, outputY, outputR, currentHeading;

    private IMU imu;
    private double ERROR;
    private boolean fieldDrive;
    private YawPitchRollAngles orientation;
    private Localizer localizer;
    private Pose STARTING_POSE = new Pose(0,0,0);

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        leftUp = hardwareMap.get(DcMotor.class, "leftUp");
        rightUp = hardwareMap.get(DcMotor.class, "rightUp");
        leftDown = hardwareMap.get(DcMotor.class, "leftDown");
        rightDown = hardwareMap.get(DcMotor.class, "rightDown");

        fieldDrive = true;

        leftUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized new version");
        telemetry.update(); // Display the "Initialized`" message
        rightDown.setDirection(DcMotorSimple.Direction.FORWARD);
        rightUp.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDown.setDirection(DcMotorSimple.Direction.REVERSE);
        leftUp.setDirection(DcMotorSimple.Direction.REVERSE);

        localizer = new Localizer.Builder()
                .webcamName("Webcam 1")
                .startingPose(STARTING_POSE)
                .build();
        localizer.init(hardwareMap);

        waitForStart();
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

        while (opModeIsActive()) {
            localizer.update();
            if (gamepad1.dpad_up) {
                imu.resetYaw();
            }

            if (gamepad1.triangle) {
                fieldDrive = true;
            } else if (gamepad1.cross) {
                fieldDrive = false;
            }

            orientation = imu.getRobotYawPitchRollAngles();
            currentHeading = orientation.getYaw(AngleUnit.DEGREES) * -1;
            if (currentHeading < 0) {
                currentHeading += 360;
            }
            relativeTargetAngle = getStickAngle(gamepad1.left_stick_x, -1 * gamepad1.left_stick_y) - currentHeading;
            joystickMagnitude = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));

            if (fieldDrive) {
                outputX = joystickMagnitude * Math.sin(relativeTargetAngle / 180 * Math.PI);
                outputY = -1 * joystickMagnitude * Math.cos(relativeTargetAngle / 180 * Math.PI);
                outputR = gamepad1.right_stick_x;
            } else {
                outputX = gamepad1.left_stick_x;
                outputY = gamepad1.left_stick_y;
                outputR = gamepad1.right_stick_x;
            }


            leftUp.setPower(0.4 * (-outputX + outputY - outputR));
            rightUp.setPower(0.4 * (-outputX - outputY - outputR));
            leftDown.setPower(0.4 * (outputX + outputY - outputR));
            rightDown.setPower(0.4 * (outputX - outputY - outputR));

            telemetry.addData("output x", outputX);
            telemetry.addData("output y", outputY);
            telemetry.addData("output r", outputR);
            telemetry.addData("magnitude", joystickMagnitude);
            telemetry.addData("stick x", gamepad1.left_stick_x);
            telemetry.addData("stick y", gamepad1.left_stick_y);
            telemetry.addData("stick angle", getStickAngle( gamepad1.left_stick_x, -1 * gamepad1.left_stick_y));
            telemetry.addData("yaw", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("pose",localizer.getPose());
            telemetry.update();
        }
    }


    private double getStickAngle(double x, double y) {
        if (y == 0) {
            if (x == 0) {
                return 0;
            } else if (x < 0) {
                return 270;
            } else {
                return 90;
            }
        }
        if (x == 0) {
            if (y > 0) {
                return 0;
            } else if (y < 0) {
                return 180;
            }
        }
        return angelCalculation(x, y, Math.abs(Math.atan(y/x) / Math.PI * 180));
    }

    private double angelCalculation(double x,double y,double inputAngle){
        if (x >0){
            if (y >0){
                //quadrant 1
                return 90 - inputAngle;
            }else{
                //quadrant 4
                return 90 + inputAngle;
            }
        }else{
            if (y >0){
                //quadrant 2
                return 270 + inputAngle;
            }else{
                return 270 - inputAngle;
                //quadrant 3
            }
        }
    }
}
//package org.firstinspires.ftc.teamcode.examples;
//
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.IMU;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.teamcode.subsystem.Intake;
//import org.firstinspires.ftc.teamcode.subsystem.Shooter;
//
//@TeleOp(name = "absoluteDriveTest1",group = "TeleOP")
//public class imuFieldOrientedDrive extends LinearOpMode {
//
//    private DcMotor leftUp, rightUp, leftDown, rightDown;
//    private double relativeTargetAngle, joystickMagnitude, outputX, outputY, outputR, currentHeading, intakeL, shootR;
//
//    private IMU imu;
//    private double ERROR;
//    private boolean fieldDrive;
//    private YawPitchRollAngles orientation;
//    private Shooter shooter;
//    private Intake intake;
//
//    @Override
//    public void runOpMode() {
//        shooter = new Shooter(hardwareMap, Shooter.Alliance.RED);
/// /        shooter = new Shooter(hardwareMap, Shooter.Alliance.RED);
//        intake = new Intake(hardwareMap);
//        imu = hardwareMap.get(IMU.class, "imu");
//        leftUp = hardwareMap.get(DcMotor.class, "leftUp");
//        rightUp = hardwareMap.get(DcMotor.class, "rightUp");
//        leftDown = hardwareMap.get(DcMotor.class, "leftDown");
//        rightDown = hardwareMap.get(DcMotor.class, "rightDown");
//
//        fieldDrive = true;
//
//        leftUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        telemetry.addData("Status", "Initialized new version");
//        telemetry.update(); // Display the "Initialized`" message
//        rightDown.setDirection(DcMotorSimple.Direction.FORWARD);
//        rightUp.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftDown.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftUp.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        waitForStart();
//        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
//        imu.resetYaw();
//
//
//        while (opModeIsActive()) {
//            shooter.shooterAiming = false;
////            driveTrain.setFieldDrive(false);
////            driveTrain.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
//            // Get detections directly - VisionPortal handles frame processing automatically
//
//            if (gamepad1.triangle) {
//                shooter.shooterRotate = true;
//            } else if (gamepad1.cross) {
//                shooter.shooterRotate = false;
//            }
//
//            if (gamepad1.dpad_up) {
//                imu.resetYaw();
//            }
//
////            if (gamepad1.triangle) {
////                fieldDrive = true;
////            } else if (gamepad1.cross) {
////                fieldDrive = false;
////            }
//
//
//            orientation = imu.getRobotYawPitchRollAngles();
//            currentHeading = orientation.getYaw(AngleUnit.DEGREES) * -1;
//            if (currentHeading < 0) {
//                currentHeading += 360;
//            }
//            relativeTargetAngle = getStickAngle(gamepad1.left_stick_x, -1 * gamepad1.left_stick_y) - currentHeading;
//            joystickMagnitude = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
//
//            if (fieldDrive) {
//                outputX = joystickMagnitude * Math.sin(relativeTargetAngle / 180 * Math.PI);
//                outputY = -1 * joystickMagnitude * Math.cos(relativeTargetAngle / 180 * Math.PI);
//                outputR = gamepad1.right_stick_x;
//            } else {
//                outputX = gamepad1.left_stick_x;
//                outputY = gamepad1.left_stick_y;
//                outputR = gamepad1.right_stick_x;
//            }
//            if (gamepad1.dpad_up) {
//                intake.reserve();
//            } else if (gamepad1.left_trigger > 0) {
//                intake.intake();
//                shooter.closeGate();
//            } else if (gamepad1.right_trigger > 0.5) {
//                shooter.openGate();
//                intake.intake();
//                intake.shoot();
//
//            } else {
//                intake.stop();
//            }
//
//
//            leftUp.setPower(0.4 * (-outputX + outputY - outputR));
//            rightUp.setPower(0.4 * (-outputX - outputY - outputR));
//            leftDown.setPower(0.4 * (outputX + outputY - outputR));
//            rightDown.setPower(0.4 * (outputX - outputY - outputR));
//
//            intake.update();
//            shooter.update(new Pose(0,0),0);
//
//            telemetry.addData("output x", outputX);
//            telemetry.addData("output y", outputY);
//            telemetry.addData("output r", outputR);
//            telemetry.addData("Intake", intakeL);
//            telemetry.addData("shoot", shootR);
//            telemetry.addData("magnitude", joystickMagnitude);
//            telemetry.addData("stick x", gamepad1.left_stick_x);
//            telemetry.addData("stick y", gamepad1.left_stick_y);
//            telemetry.addData("stick angle", getStickAngle( gamepad1.left_stick_x, -1 * gamepad1.left_stick_y));
//            telemetry.addData("yaw", orientation.getYaw(AngleUnit.DEGREES));
//            telemetry.update();
//        }
//    }
//
//
//    private double getStickAngle(double x, double y) {
//        if (y == 0) {
//            if (x == 0) {
//                return 0;
//            } else if (x < 0) {
//                return 270;
//            } else {
//                return 90;
//            }
//        }
//        if (x == 0) {
//            if (y > 0) {
//                return 0;
//            } else if (y < 0) {
//                return 180;
//            }
//        }
//        return angelCalculation(x, y, Math.abs(Math.atan(y/x) / Math.PI * 180));
//    }
//
//    private double angelCalculation(double x,double y,double inputAngle){
//        if (x >0){
//            if (y >0){
//                //quadrant 1
//                return 90 - inputAngle;
//            }else{
//                //quadrant 4
//                return 90 + inputAngle;
//            }
//        }else{
//            if (y >0){
//                //quadrant 2
//                return 270 + inputAngle;
//            }else{
//                return 270 - inputAngle;
//                //quadrant 3
//            }
//        }
//    }
//}
