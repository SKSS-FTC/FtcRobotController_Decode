package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.shooter.BlueShooter;

@TeleOp(name = "ShooterAlgorithmTest1", group = "Tests")
public class shooterAlgorithmTest1 extends LinearOpMode {
    private BlueShooter blueShooter;

    @Override
    public void runOpMode() {
        blueShooter = new BlueShooter(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.triangle) {
                blueShooter.shooterAiming = true;
            }
            if (gamepad1.cross) {
                blueShooter.shooterAiming = false;
            }
            blueShooter.update(new Pose(0,0),120);
            telemetry.addData("Shooter Aiming", blueShooter.shooterAiming);
            telemetry.addData("current angular velocity", blueShooter.getAngularVelocity());
            telemetry.update();
        }
    }
}
