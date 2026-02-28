package org.firstinspires.ftc.teamcode.subsystem;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class DriveTrain {
    public DcMotor leftUp, rightUp, leftDown, rightDown;
    public double x, y, r, relativeTargetAngle, joystickMagnitude, currentHeading;
    private boolean fieldDrive;
    private IMU imu;
    private YawPitchRollAngles orientation;
    public DriveTrain(HardwareMap hardwareMap){
        leftUp = hardwareMap.get(DcMotor.class, "leftUp");
        rightUp = hardwareMap.get(DcMotor.class, "rightUp");
        leftDown = hardwareMap.get(DcMotor.class, "leftDown");
        rightDown = hardwareMap.get(DcMotor.class, "rightDown");
        imu = hardwareMap.get(IMU.class,"imu");
        fieldDrive = true;
        leftUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftUp.setDirection(DcMotor.Direction.FORWARD);
        rightUp.setDirection(DcMotor.Direction.REVERSE);
        leftDown.setDirection(DcMotor.Direction.FORWARD);
        rightDown.setDirection(DcMotor.Direction.FORWARD);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.DOWN)));
        imu.resetYaw();
    }
    public void update(double gamepad1_left_X,double gamepad1_left_Y,double gamepad1_right_X){
        orientation = imu.getRobotYawPitchRollAngles();
        currentHeading = orientation.getYaw(AngleUnit.DEGREES) * -1;
        if (currentHeading < 0) {
            currentHeading += 360;
        }
        relativeTargetAngle = currentHeading - getStickAngle(gamepad1_left_X, -1 *gamepad1_left_Y);
        joystickMagnitude = Math.sqrt(Math.pow(gamepad1_left_X, 2) + Math.pow(gamepad1_left_Y, 2));

        if (fieldDrive) {
            x = joystickMagnitude * Math.sin(relativeTargetAngle / 180 * Math.PI);
            y = joystickMagnitude * Math.cos(relativeTargetAngle / 180 * Math.PI);
            r = gamepad1_right_X;
        } else {
            x = gamepad1_left_X;
            y = gamepad1_left_Y;
            r = gamepad1_right_X;
        }
        leftUp.setPower(0.4 * (-x + y - r));
        rightUp.setPower(0.4 * (-x - y - r));
        leftDown.setPower(0.4 * (x + y - r));
        rightDown.setPower(0.4 * (x - y - r));
    }
    public double getStickAngle(double x, double y) {
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
        return angelCalculation(x, y, Math.abs(Math.atan( y/ x)) / Math.PI * 180);
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
    public void resetIMU(){
        imu.resetYaw();
    }
    public void setFieldDrive(boolean input){
        fieldDrive = input;
    }
}

