package org.firstinspires.ftc.teamcode.opmode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Localizer;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;

@TeleOp(name = "ShooterAlgorithmTest1", group = "Tests")
public class shooterAlgorithmTest1 extends LinearOpMode {
    private Shooter shooter;
    private TelemetryManager telemetryM;
    private Localizer localizer;
    private boolean wasLogging = false;
    private Pose STARTING_POSE = new Pose(0,0,0);

    @Override
    public void runOpMode() {
        shooter = new Shooter(hardwareMap, Shooter.Alliance.BLUE);
        localizer = new Localizer.Builder()
                .webcamName("Webcam 1")
                .startingPose(STARTING_POSE)
                .build();
        localizer.init(hardwareMap, "Webcam 1");
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetry.addLine("Status: Initialized");
        telemetry.addLine("Press CIRCLE to start/stop logging");
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

            if (gamepad1.circle && !shooter.isLogging()) {
                shooter.startLogging();
                wasLogging = true;
            } else if (!gamepad1.circle && shooter.isLogging() && wasLogging) {
                shooter.stopLoggingAndSave();
                wasLogging = false;
            }
            localizer.update();
            shooter.update(localizer.getPose(),localizer.getHeading());

            telemetryM.debug("Shooter Aiming: " + shooter.shooterAiming);
            telemetryM.debug("Alliance: " + shooter.getAlliance().toString());
            telemetryM.debug("Shoot Velocity (rad/s): " + shooter.getShootVelocity());
            telemetryM.debug("distacne",shooter.getDistanceToTarget());
            telemetryM.debug("pose",localizer.getPose());
            telemetryM.debug("Logging: " + (shooter.isLogging() ? "YES (" + shooter.getDataPointCount() + " pts)" : "NO"));
            if (!shooter.isLogging() && shooter.getLastLogFilePath() != null) {
                telemetryM.debug("Last Log: " + shooter.getLastLogFilePath());
            }
            telemetryM.update(telemetry);
        }
    }
}
