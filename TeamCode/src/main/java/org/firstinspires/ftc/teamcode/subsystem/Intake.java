package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Intake {
    private DcMotor intake;
    private Servo kicker;
    private ElapsedTime shootTimer;
    private int shootState = 0;
    public static final double SHOOT_DELAY_MS = 150;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class,"intake");
        kicker = hardwareMap.get(Servo.class,"kicker");
        shootTimer = new ElapsedTime();

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        kicker.setDirection(Servo.Direction.REVERSE);
        intake.setPower(0);
        kicker.setPosition(0.4);
    }

    public void intake(){intake.setPower(1);}
    public void stop(){intake.setPower(0);}
    public void reserve(){intake.setPower(-0.7);}

    public void shoot(){
        stop();
        setKickerPosition(0);
        shootState = 1;
        shootTimer.reset();
    }

    public void update(){
        if (shootState == 1 && shootTimer.milliseconds() >= SHOOT_DELAY_MS) {
            setKickerPosition(0.4);
            shootState = 0;
        }
    }

    public void setKickerPosition(double target){
        kicker.setPosition(target);
    }
}
