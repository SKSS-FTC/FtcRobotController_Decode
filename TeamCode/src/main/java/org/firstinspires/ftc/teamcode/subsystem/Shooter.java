package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.Constants.BLUE_SHOOTING_TARGET_POSITION_MAP;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.RED_SHOOTING_TARGET_POSITION_MAP;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.SHOOTER_PID;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.SHOOTER_PID_KP;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.SHOOTER_PID_KI;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.SHOOTER_PID_KD;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.SHOOTER_SMC_LAMBDA;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.SHOOTER_SMC_ETA;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.angleTunerAngle;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.gateClose;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.gateOpen;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.shooterMinCheckSecond;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.shooterTargetAngularVelocity;
import static org.firstinspires.ftc.teamcode.subsystem.RobotState.shooterEncoder;

import android.content.Context;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import Jama.Matrix;

public class Shooter {
    private static final int TICKS_PER_REVOLUTION = 28;

    public enum Alliance {
        BLUE,
        RED
    }

    private DcMotorEx rotate, shooter;
    private Servo angleTuner,gate;
    private Pose currentPose, relativeShootingVector;
    private double absoluteShooterHeading = 0, relativeShooterHeading = 0;
    public boolean shooterAiming = false;
    public boolean shooterRotate = false;
    public boolean farShot = false;
    public boolean gateClosed = true;
    private Alliance alliance;
    private double lastTime = 0.0;
    private double lastPowerUpdateTime = 0.0;
    private double lastAngularVelocity = 0.0;
    private ElapsedTime timer;

    private boolean isLogging = false;
    private List<DataPoint> dataPoints = new ArrayList<>();
    private HardwareMap hardwareMap;
    private double logStartTime = 0.0;

    private static class DataPoint {
        final double time;
        final double targetAngularVelocity;
        final double currentAngularVelocity;
        final double error;

        DataPoint(double time, double targetAngularVelocity, double currentAngularVelocity, double error) {
            this.time = time;
            this.targetAngularVelocity = targetAngularVelocity;
            this.currentAngularVelocity = currentAngularVelocity;
            this.error = error;
        }
    }

    public Shooter(HardwareMap hardwareMap, Alliance alliance) {
        this.hardwareMap = hardwareMap;
        this.alliance = alliance;

        rotate = hardwareMap.get(DcMotorEx.class, "rotate");
        shooter = hardwareMap.get(DcMotorEx.class, "shoot");
        angleTuner = hardwareMap.get(Servo.class, "angleTuner");
        gate = hardwareMap.get(Servo.class,"shooterGate");


        rotate.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        angleTuner.setDirection(Servo.Direction.FORWARD);
        gate.setDirection(Servo.Direction.FORWARD);

        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rotate.setTargetPosition(0);
        rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gate.setPosition(gateClose);

        timer = new ElapsedTime();
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public void openGate(){
        gate.setPosition(gateOpen);
        gateClosed = false;
    }

    public void closeGate(){
        gate.setPosition(gateClose);
        gateClosed = true;
    }
    public boolean getGateClosed(){
        return gateClosed;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public void update(Pose inputPose, double robotHeading) {
        currentPose = inputPose;
        if (shooterAiming) {
            calculateShooterAngle(robotHeading);
            setShooterAngle(relativeShooterHeading);
        } else {
            rotate.setPower(0);
        }
        if (shooterRotate){
            setAngleTuner(currentPose.getX(), currentPose.getY());
        }else{
            shooter.setPower(0);
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
        if (!farShot) {
            if (distance < 25) {
                shooterTargetAngularVelocity = 175;
            } else if (distance < 45) {
                shooterTargetAngularVelocity = 200;
            } else {
                shooterTargetAngularVelocity = 235;
            }
        }else {
            shooterTargetAngularVelocity = 290;
        }
        double targetAngularVelocity = shooterTargetAngularVelocity;

        if (deltaTime > shooterMinCheckSecond) {
            double currentAngularVelocity = shooter.getVelocity() / TICKS_PER_REVOLUTION * 2 * Math.PI;
            double error = targetAngularVelocity - currentAngularVelocity;

            if (isLogging) {
                double logTime = (currentTime - logStartTime) / 1000.0;
                dataPoints.add(new DataPoint(logTime, targetAngularVelocity, currentAngularVelocity, error));
            }

            double algorithmOutput;
            algorithmOutput = getShooterSMCPower(targetAngularVelocity, currentAngularVelocity);
            double targetTicksPerSecond = targetAngularVelocity / (2 * Math.PI) * TICKS_PER_REVOLUTION;
            double algorithmCorrectionTicks = algorithmOutput / (2 * Math.PI) * TICKS_PER_REVOLUTION;
            shooter.setVelocity(targetTicksPerSecond + algorithmCorrectionTicks);
            lastPowerUpdateTime = currentTime;
        }
    }

    private void setShootingAngle(double distance) {
//        double output = distance * 0.01;
        double output = angleTunerAngle;
        if (!farShot) {
            if (distance < 25) {
                angleTunerAngle = 0.15;
            } else if (distance < 45) {
                angleTunerAngle = 0.2;
            } else {
                angleTunerAngle = 0.25;
            }
        }else {
            angleTunerAngle = 0.28;
        }

        //0.04-0.3
        angleTuner.setPosition(output);
    }

    private double getShooterSMCPower(double targetAngularVelocity, double currentAngularVelocity) {
        double error = targetAngularVelocity - currentAngularVelocity;
        double slidingSurface = error + SHOOTER_SMC_LAMBDA * error;
        return SHOOTER_SMC_ETA * Math.signum(slidingSurface);
    }

    public void startLogging() {
        if (!isLogging) {
            dataPoints.clear();
            isLogging = true;
            logStartTime = timer.milliseconds();
        }
    }

    public void stopLoggingAndSave() {
        if (isLogging) {
            isLogging = false;
            saveToCSV();
        }
    }

    public boolean isLogging() {
        return isLogging;
    }

    public int getDataPointCount() {
        return dataPoints.size();
    }

    private void saveToCSV() {
        if (dataPoints.isEmpty()) {
            return;
        }

        try {
            Context context = hardwareMap.appContext;
            File logDir = new File(context.getFilesDir(), "shooter_logs");
            if (!logDir.exists()) {
                logDir.mkdirs();
            }

            String timestamp = new java.text.SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new java.util.Date());
            File csvFile = new File(logDir, "shooter_data_" + timestamp + ".csv");

            BufferedWriter writer = new BufferedWriter(new FileWriter(csvFile));
            writer.write("time,target_angular_velocity,current_angular_velocity,error");
            writer.newLine();

            for (DataPoint dp : dataPoints) {
                writer.write(String.format(Locale.US, "%.4f,%.4f,%.4f,%.4f",
                        dp.time, dp.targetAngularVelocity, dp.currentAngularVelocity, dp.error));
                writer.newLine();
            }

            writer.flush();
            writer.close();

            System.out.println("Shooter data saved to: " + csvFile.getAbsolutePath());

        } catch (Exception e) {
            System.err.println("Failed to save shooter data: " + e.getMessage());
        }
    }

    public String getLastLogFilePath() {
        Context context = hardwareMap.appContext;
        File logDir = new File(context.getFilesDir(), "shooter_logs");
        if (logDir.exists() && logDir.listFiles() != null && logDir.listFiles().length > 0) {
            File[] files = logDir.listFiles();
            if (files != null && files.length > 0) {
                java.util.Arrays.sort(files, (a, b) -> Long.compare(b.lastModified(), a.lastModified()));
                return files[0].getAbsolutePath();
            }
        }
        return null;
    }

    public double getShootVelocity() {
        return shooter.getVelocity() / TICKS_PER_REVOLUTION * 2 * Math.PI;
    }
    public double getDistanceToTarget(){
        if(relativeShootingVector == null){
            return -1;
        }else{
            return Math.sqrt(Math.pow(relativeShootingVector.getX(),2) + Math.pow(relativeShootingVector.getY(),2));
        }
    }
}
