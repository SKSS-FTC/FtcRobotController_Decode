package org.firstinspires.ftc.teamcode.subsystem.shooter;

import static org.firstinspires.ftc.teamcode.subsystem.Constants.blueAprilTag;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.shooterEncoder;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.path.BluePath;

import java.nio.file.attribute.FileOwnerAttributeView;

public class BlueShooter {
    private DcMotor rotate,shooter;
    private Servo angleTuner;
    private Pose currentPose,relativeShootingVector;
    private double absoluteShooterHeading = 0,relativeShooterHeading = 0;
    public boolean shooterAiming = false;

    public BlueShooter(HardwareMap hardwareMap){
        rotate = hardwareMap.get(DcMotor.class, "rotate");
        shooter = hardwareMap.get(DcMotor.class,"shoot");
        angleTuner = hardwareMap.get(Servo.class,"angleTuner");

        rotate.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        angleTuner.setDirection(Servo.Direction.FORWARD);

        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rotate.setTargetPosition(0);
        rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update(Pose inputPose,double robotHeading){
        currentPose = inputPose;
        if (shooterAiming){
            calculateShooterAngle(robotHeading);
            setShooterAngle(relativeShooterHeading);
            setAngleTuner(currentPose.getX(),currentPose.getY());
        }else{
            shooter.setPower(0);
            rotate.setPower(0);
        }
    }

    private void setShooterAngle(double shooterTargetHeading) {
        rotate.setTargetPosition((int) (shooterTargetHeading /360 * shooterEncoder));
        rotate.setPower(1);
    }

    private void calculateShooterAngle(double robotHeading){
        relativeShootingVector = new Pose(currentPose.getX() - blueAprilTag[0],
                currentPose.getY() - blueAprilTag[1]);
        absoluteShooterHeading = 360 - Math.tan(Math.abs(relativeShootingVector.getX()/relativeShootingVector.getY()) * Math.PI /180);
        relativeShooterHeading = absoluteShooterHeading - robotHeading;
        while(relativeShooterHeading > 360){
            relativeShooterHeading -= 360;
        }
        while(relativeShooterHeading<0){
            relativeShooterHeading += 360;
        }
    }

    private void setAngleTuner(double x,double y){
        double distance = Math.sqrt(Math.pow(x,2)+ Math.pow(y,2));
        setShooterPower(distance);
        setShootingAngle(distance);
    }

    private void setShooterPower(double distance){
        double output = distance * 0.01;// algorithm not confirm
        shooter.setPower(output);
    }

    private void setShootingAngle(double distance){
        double output = distance * 0.01;// algorithm not confirm
        angleTuner.setPosition(output);
    }
}
