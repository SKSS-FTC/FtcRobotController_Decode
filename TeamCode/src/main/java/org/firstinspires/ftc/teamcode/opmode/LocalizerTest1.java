package org.firstinspires.ftc.teamcode.opmode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Localizer;

@TeleOp(name = "LocalizerTest1", group = "Tests")
public class LocalizerTest1 extends LinearOpMode {
    private Localizer localizer;
    private TelemetryManager telemetryM;

    private static final double INCH_TO_METER = 0.0254;
    private static final Pose STARTING_POSE = new Pose(56.37, 9.01, Math.toRadians(90));

    @Override
    public void runOpMode() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        localizer = new Localizer.Builder()
                .webcamName("Webcam 1")
                .startingPose(STARTING_POSE)
                .build();

        telemetry.addLine("Initializing Localizer...");
        telemetry.update();

        localizer.init(hardwareMap, "Webcam 1");

        telemetry.addLine("Localizer Initialized");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        telemetryM.debug("=== Localizer Test ===");
        telemetryM.debug("Starting Pose: X=" + STARTING_POSE.getX() + " in, Y=" + STARTING_POSE.getY() + " in, H=" + Math.toDegrees(STARTING_POSE.getHeading()) + " deg");
        telemetryM.update(telemetry);

        waitForStart();

        while (opModeIsActive()) {
            localizer.update();

            double xMeters = localizer.getXMeters();
            double yMeters = localizer.getYMeters();
            double heading = localizer.getHeading();
            double xInches = xMeters / INCH_TO_METER;
            double yInches = yMeters / INCH_TO_METER;
            boolean aprilTagVisible = localizer.isAprilTagVisible();
            int lastTagId = localizer.getLastVisibleTagId();

            telemetry.addData("=== Merged Pose (Map Frame) ===", "");
            telemetry.addData("X (m)", "%.3f", xMeters);
            telemetry.addData("Y (m)", "%.3f", yMeters);
            telemetry.addData("X (in)", "%.2f", xInches);
            telemetry.addData("Y (in)", "%.2f", yInches);
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
            telemetry.addData("AprilTag Visible", aprilTagVisible ? "YES (Tag " + lastTagId + ")" : "NO");
            telemetry.addData("Position Source", aprilTagVisible ? "AprilTag Vision" : "Follower Odometry");
            telemetry.update();

            telemetryM.debug("=== Merged Pose ===");
            telemetryM.debug("X: " + String.format("%.3f m (%.2f in)", xMeters, xInches));
            telemetryM.debug("Y: " + String.format("%.3f m (%.2f in)", yMeters, yInches));
            telemetryM.debug("Heading: " + String.format("%.1f deg", Math.toDegrees(heading)));
            telemetryM.debug("AprilTag Visible: " + (aprilTagVisible ? "YES (Tag " + lastTagId + ")" : "NO"));
            telemetryM.debug("Source: " + (aprilTagVisible ? "AprilTag Vision" : "Follower Odometry"));
            telemetryM.update(telemetry);
        }

        localizer.close();
    }
}
