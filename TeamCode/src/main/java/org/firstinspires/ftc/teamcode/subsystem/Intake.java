package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    private DcMotor intake;
    private Servo kicker;
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class,"intake");
        kicker = hardwareMap.get(Servo.class,"kicker");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        kicker.setDirection(Servo.Direction.REVERSE);
        intake.setPower(0);
        kicker.setPosition(0.4);
    }

    public void intake(){intake.setPower(1);}
    public void stop(){intake.setPower(0);}
    public void reserve(){intake.setPower(-0.2);}
    public void shoot(){
        stop();
        setKickerPosition(0);
        sleep(150);
        setKickerPosition(0.4);
    }
    public void setKickerPosition(double target){
        kicker.setPosition(target);
    }
}
