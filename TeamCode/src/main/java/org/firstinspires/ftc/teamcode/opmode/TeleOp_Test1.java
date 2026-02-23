package org.firstinspires.ftc.teamcode.opmode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.DriveTrain;

@TeleOp(name = "TeleOp_Test1")
public class TeleOp_Test1 extends LinearOpMode {
    private DriveTrain driveTrain;

    @Override
    public void runOpMode() {
        driveTrain = new DriveTrain(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            driveTrain.setFieldDrive(true);
            driveTrain.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
    }
}