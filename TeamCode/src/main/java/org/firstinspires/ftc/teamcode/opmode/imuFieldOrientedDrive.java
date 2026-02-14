package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "absoluteDriveTest1",group = "TeleOP")
public class imuFieldOrientedDrive extends LinearOpMode {

    private DcMotor leftUp, rightUp, leftDown, rightDown, intake, shoot;
    private double relativeTargetAngle, joystickMagnitude, outputX, outputY, outputR, currentHeading, intakeL, shootR;

    private IMU imu;
    private double ERROR;
    private boolean fieldDrive;
    private YawPitchRollAngles orientation;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        leftUp = hardwareMap.get(DcMotor.class, "leftUp");
        rightUp = hardwareMap.get(DcMotor.class, "rightUp");
        leftDown = hardwareMap.get(DcMotor.class, "leftDown");
        rightDown = hardwareMap.get(DcMotor.class, "rightDown");
        intake = hardwareMap.get(DcMotor.class,"intake");
        shoot = hardwareMap.get(DcMotor.class,"shoot");

        fieldDrive = true;

        leftUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update(); // Display the "Initialized`" message
        shoot.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDown.setDirection(DcMotorSimple.Direction.FORWARD);
        rightUp.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDown.setDirection(DcMotorSimple.Direction.REVERSE);
        leftUp.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

        while (opModeIsActive()) {
            if (gamepad1.left_trigger > 0.7) {
                shoot.setPower(1);
            } else {
                shoot.setPower(0);
            }
            if (gamepad1.right_trigger > 0.7) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }
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
            relativeTargetAngle = currentHeading - getStickAngle(-1 * gamepad1.left_stick_x, gamepad1.left_stick_y);
            joystickMagnitude = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));

            if (fieldDrive) {
                outputX = joystickMagnitude * Math.sin(relativeTargetAngle / 180 * Math.PI);
                outputY = joystickMagnitude * Math.cos(relativeTargetAngle / 180 * Math.PI);
                outputR = gamepad1.right_stick_x;
            } else {
                outputX = gamepad1.left_stick_x;
                outputY = gamepad1.left_stick_y;
                outputR = gamepad1.right_stick_x;
            }

            intakeL = gamepad1.left_trigger;
            shootR = gamepad1.right_trigger;

            leftUp.setPower(0.4 * (-outputX + outputY - outputR));
            rightUp.setPower(0.4 * (-outputX - outputY - outputR));
            leftDown.setPower(0.4 * (outputX + outputY - outputR));
            rightDown.setPower(0.4 * (outputX - outputY - outputR));
            telemetry.addData("output x", outputX);
            telemetry.addData("output y", outputY);
            telemetry.addData("output r", outputR);
            telemetry.addData("Intake", intakeL);
            telemetry.addData("shoot", shootR);
            telemetry.addData("magnitude", joystickMagnitude);
            telemetry.addData("stick x", gamepad1.left_stick_x);
            telemetry.addData("stick y", -1 *gamepad1.left_stick_y);
            telemetry.addData("stick angle", getStickAngle( gamepad1.left_stick_x, -1 *gamepad1.left_stick_y));
            telemetry.addData("yaw", orientation.getYaw(AngleUnit.DEGREES));
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
        return angelCalculation(x, y, Math.abs(Math.atan(gamepad1.left_stick_y / gamepad1.left_stick_x)) / Math.PI * 180);
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
                //quadrant 3
                if (inputAngle == 90){
                    return 0;
                }else {
                    return 270 - inputAngle;
                }
            }
        }
    }
}
