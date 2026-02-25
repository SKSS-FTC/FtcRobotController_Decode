package org.firstinspires.ftc.teamcode.opmode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Slider;

@TeleOp(name = "TeleOp_Test1")
public class TeleOp_Test1 extends LinearOpMode {
    private DriveTrain driveTrain;
    private Shooter shooter;
    private Intake intake;
    private Slider slider;

    @Override
    public void runOpMode() {
        driveTrain = new DriveTrain(hardwareMap);
        intake = new Intake(hardwareMap);
        intake.stop();
        waitForStart();
        while (opModeIsActive()) {
            driveTrain.setFieldDrive(true);
            driveTrain.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            if (gamepad1.left_trigger_pressed) {
                intake.intake();
            } else {
                intake.stop();
            }
        }
        ;
    }
}