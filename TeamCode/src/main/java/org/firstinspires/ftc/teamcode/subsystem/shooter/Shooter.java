package org.firstinspires.ftc.teamcode.subsystem.shooter;

import static org.firstinspires.ftc.teamcode.subsystem.Constants.BLUE_SHOOTING_TARGET_POSITION_MAP;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.RED_SHOOTING_TARGET_POSITION_MAP;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.SHOOTER_PID;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.SHOOTER_PID_KP;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.SHOOTER_PID_KI;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.SHOOTER_PID_KD;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.SHOOTER_SMC_LAMBDA;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.SHOOTER_SMC_ETA;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.shooterMinCheckSecond;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.shooterTargetAngularVelocity;
import static org.firstinspires.ftc.teamcode.subsystem.RobotState.shooterEncoder;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import Jama.Matrix;

public class Shooter {
    private static final int TICKS_PER_REVOLUTION = 28;

    public enum Alliance {
        BLUE,
        RED
    }

    private DcMotorEx rotate, shooter;
    private Servo angleTuner;
    private Pose currentPose, relativeShootingVector;
    private double absoluteShooterHeading = 0, relativeShooterHeading = 0;
    public boolean shooterAiming = false;
    private Alliance alliance;

    private double pidIntegral = 0.0;
    private double pidLastError = 0.0;

    private int lastEncoderPosition = 0;
    private double lastTime = 0.0;
    private double lastPowerUpdateTime = 0.0;
    private double lastAngularVelocity = 0.0;
    private ElapsedTime timer;

    public Shooter(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;

        rotate = hardwareMap.get(DcMotorEx.class, "rotate");
        shooter = hardwareMap.get(DcMotorEx.class, "shoot");
        angleTuner = hardwareMap.get(Servo.class, "angleTuner");

        rotate.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        angleTuner.setDirection(Servo.Direction.FORWARD);

        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rotate.setTargetPosition(0);
        rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        timer = new ElapsedTime();
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public void update(Pose inputPose, double robotHeading) {
        currentPose = inputPose;
        if (shooterAiming) {
            calculateShooterAngle(robotHeading);
            setShooterAngle(relativeShooterHeading);
            setAngleTuner(currentPose.getX(), currentPose.getY());
        } else {
            shooter.setPower(0);
            rotate.setPower(0);
        }
    }

    private void setShooterAngle(double shooterTargetHeading) {
        rotate.setTargetPosition((int) (shooterTargetHeading / 360 * shooterEncoder));
        rotate.setPower(1);
    }

    private void calculateShooterAngle(double robotHeading) {
        Matrix targetPosition = getTargetPosition();
        relativeShootingVector = new Pose(
                currentPose.getX() - targetPosition.get(0, 0),
                currentPose.getY() - targetPosition.get(1, 0));

        if (alliance == Alliance.BLUE) {
            absoluteShooterHeading = 360 - Math.tan(Math.abs(relativeShootingVector.getX() / relativeShootingVector.getY()) * Math.PI / 180);
        } else {
            absoluteShooterHeading = Math.tan(Math.abs(relativeShootingVector.getX() / relativeShootingVector.getY()) * Math.PI / 180);
        }

        relativeShooterHeading = absoluteShooterHeading - robotHeading;
        while (relativeShooterHeading > 360) {
            relativeShooterHeading -= 360;
        }
        while (relativeShooterHeading < 0) {
            relativeShooterHeading += 360;
        }
    }

    private Matrix getTargetPosition() {
        return alliance == Alliance.BLUE ? BLUE_SHOOTING_TARGET_POSITION_MAP : RED_SHOOTING_TARGET_POSITION_MAP;
    }

    private void setAngleTuner(double x, double y) {
        double distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        setShooterPower(distance);
        setShootingAngle(distance);
    }

    private void setShooterPower(double distance) {
        double currentTime = timer.milliseconds();
        double deltaTime = (currentTime - lastPowerUpdateTime) / 1000.0;

        if (deltaTime > shooterMinCheckSecond) {
            double currentAngularVelocity = shooter.getVelocity();
            double output;
            if (SHOOTER_PID) {
                output = getShooterPIDFPower(shooterTargetAngularVelocity, currentAngularVelocity);
            } else {
                output = getShooterSMCPower(shooterTargetAngularVelocity, currentAngularVelocity);
            }
            output += shooterTargetAngularVelocity /2 /Math.PI * 28;
            shooter.setVelocity(output);
            lastPowerUpdateTime = currentTime;
        }
    }

    private void setShootingAngle(double distance) {
        double output = distance * 0.01;
        angleTuner.setPosition(output);
    }

    private double getShooterPIDFPower(double targetAngularVelocity, double currentAngularVelocity) {
        double error = targetAngularVelocity - currentAngularVelocity;
        pidIntegral += error;
        double derivative = error - pidLastError;
        pidLastError = error;
        double output = SHOOTER_PID_KP * error + SHOOTER_PID_KI * pidIntegral + SHOOTER_PID_KD * derivative;
        return Math.max(-1.0, Math.min(1.0, output));
    }

    private double getShooterSMCPower(double targetAngularVelocity, double currentAngularVelocity) {
        double error = targetAngularVelocity - currentAngularVelocity;
        double slidingSurface = error + SHOOTER_SMC_LAMBDA * error;
        double output = SHOOTER_SMC_ETA * Math.signum(slidingSurface);
        return Math.max(-1.0, Math.min(1.0, output));
    }

    public double getAngularVelocity() {
        int currentEncoderPosition = shooter.getCurrentPosition();
        double currentTime = timer.milliseconds();
        double deltaTime = (currentTime - lastTime) / 1000.0;
        double angularVelocity = 0.0;
        if (deltaTime > shooterMinCheckSecond) {
            double ticksPerSecond = (currentEncoderPosition - lastEncoderPosition) / deltaTime;
            angularVelocity = (ticksPerSecond / TICKS_PER_REVOLUTION) * 2 * Math.PI;
            lastEncoderPosition = currentEncoderPosition;
            lastTime = currentTime;
            lastAngularVelocity = angularVelocity;
        }
        return lastAngularVelocity;
    }
    public double getShootVelocity{return (shooter.getVelocity() /2 /Math.PI *28);}
}
