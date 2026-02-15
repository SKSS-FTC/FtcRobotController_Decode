package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.shooter.Shooter;

@TeleOp(name = "ShooterAlgorithmTest1", group = "Tests")
public class shooterAlgorithmTest1 extends LinearOpMode {
    private Shooter shooter;

    @Override
    public void runOpMode() {
        shooter = new Shooter(hardwareMap, Shooter.Alliance.BLUE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.triangle) {
                shooter.shooterAiming = true;
            }
            if (gamepad1.cross) {
                shooter.shooterAiming = false;
            }
            if (gamepad1.dpad_left) {
                shooter.setAlliance(Shooter.Alliance.BLUE);
            }
            if (gamepad1.dpad_right) {
                shooter.setAlliance(Shooter.Alliance.RED);
            }
            shooter.update(new Pose(0, 0), 120);
            telemetry.addData("Shooter Aiming", shooter.shooterAiming);
            telemetry.addData("Alliance", shooter.getAlliance());
            telemetry.addData("Angular Velocity (rad/s)", shooter.getAngularVelocity());
            telemetry.update();
        }
    }
}
