package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "main", group = "TeleOP")
public class main extends LinearOpMode {
    private DcMotor leftUp, rightUp, leftDown, rightDown, Intake, Shoot;
    private double x, y, r, L, R;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftUp = hardwareMap.get(DcMotor.class, "leftUp");
        rightUp = hardwareMap.get(DcMotor.class, "rightUp");
        leftDown = hardwareMap.get(DcMotor.class, "leftDown");
        rightDown = hardwareMap.get(DcMotor.class, "rightDown");
        Intake = hardwareMap.get(DcMotor.class, "intake");
        Shoot = hardwareMap.get(DcMotor.class, "shoot");

        waitForStart();

        leftUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftUp.setDirection(DcMotor.Direction.FORWARD);
        rightUp.setDirection(DcMotor.Direction.REVERSE);
        leftDown.setDirection(DcMotor.Direction.FORWARD);
        rightDown.setDirection(DcMotor.Direction.FORWARD);

        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double r = gamepad1.right_stick_x;
            double L = gamepad1.left_trigger;
            double R = gamepad1.right_trigger;

            // Mecanum drive calculations
            leftUp.setPower(-x + y - r);
            rightUp.setPower(-x - y - r);
            leftDown.setPower(x + y - r);
            rightDown.setPower(x - y - r);

            // Intake and shooting controls
            Intake.setPower(L);
            Shoot.setPower(R);
            telemetry.update();
        }
    }
}