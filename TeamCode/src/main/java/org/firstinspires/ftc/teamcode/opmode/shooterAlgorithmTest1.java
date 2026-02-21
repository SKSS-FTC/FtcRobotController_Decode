package org.firstinspires.ftc.teamcode.opmode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.shooter.Shooter;

@TeleOp(name = "ShooterAlgorithmTest1", group = "Tests")
public class shooterAlgorithmTest1 extends LinearOpMode {
    private Shooter shooter;
    private TelemetryManager telemetryM;

    @Override
    public void runOpMode() {
        shooter = new Shooter(hardwareMap, Shooter.Alliance.BLUE);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetry.addLine("Status: Initialized");
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

            telemetryM.debug("Shooter Aiming: " + shooter.shooterAiming);
            telemetryM.debug("Alliance: " + shooter.getAlliance().toString());
            telemetryM.debug("Angular Velocity (rad/s): " + shooter.getAngularVelocity());
            telemetryM.update(telemetry);
        }
    }
}
