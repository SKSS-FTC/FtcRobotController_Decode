package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.Constants.leftSliderServoClose;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.leftSliderServoOpen;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.rightSliderServoClose;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.rightSliderServoOpen;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.sliderDriveEncoderValue;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.sliderParkEncoderValue;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Slider {
    private DcMotor slider;
    private Servo leftSliderServo,rightSliderServo;
    public Slider(HardwareMap hardwareMap) {
        slider = hardwareMap.get(DcMotor.class,"slider");
        leftSliderServo = hardwareMap.get(Servo.class,"leftSliderServo");
        rightSliderServo = hardwareMap.get(Servo.class,"rightSliderServo");
        leftSliderServo.setDirection(Servo.Direction.FORWARD);
        rightSliderServo.setDirection(Servo.Direction.FORWARD);

        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        servoClose();
    }

    private void servoClose(){
        //servo close to ready for drive
        leftSliderServo.setPosition(leftSliderServoClose);
        rightSliderServo.setPosition(rightSliderServoClose);
    }

    private void servoOpen(){
        //servo open to ready for park
        leftSliderServo.setPosition(leftSliderServoOpen);
        rightSliderServo.setPosition(rightSliderServoOpen);
    }

    private void sliderPark(){
        slider.setTargetPosition(sliderParkEncoderValue);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(1);
    }

    private void sliderDrive(){
        slider.setTargetPosition(0);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(1);
    }

    public void park() throws InterruptedException {
        servoOpen();
        wait(200);
        sliderPark();
    }

    public void drive(){
        sliderDrive();
        while(slider.isBusy()){}
        servoClose();
    }
}
