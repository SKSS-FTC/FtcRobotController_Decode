package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.subsystem.AprilTagReader;

import Jama.Matrix;
import java.util.List;

@TeleOp(name = "ExampleAprilTagReader", group = "Examples")
public class ExampleAprilTagReader extends LinearOpMode {

    private AprilTagReader aprilTagReader;
    private VisionPortal visionPortal;
    private java.util.Queue<Long> frameTimestamps = new java.util.LinkedList<>();
    private static final double MOVING_AVERAGE_WINDOW_SECONDS = 5.0;

    @Override
    public void runOpMode() {

        aprilTagReader = new AprilTagReader();

        // TODO: Update AprilTag initialization to match current SDK API
        // AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
        //         .setTagFamily(AprilTagGameDatabase.getCurrentGameTagFamily())
        //         .setTagLibrary(AprilTagGameDatabase.getTagLibrary())
        //         .build();
        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder().build();

        aprilTagReader.setProcessor(aprilTagProcessor);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.addProcessor(aprilTagProcessor);

        // Set camera (webcam will be selected if configured)
        CameraName cameraName = hardwareMap.get(CameraName.class, "Webcam 1");
        builder.setCamera(cameraName);

        // Build and start portal
        visionPortal = builder.build();
        visionPortal.resumeStreaming();

        telemetry.addData("Status", "Vision Portal initialized");
        telemetry.addData("Status", "AprilTag detection ready. Press START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Calculate FPS using 5-second moving average
            long now = System.nanoTime();
            frameTimestamps.add(now);
            
            // Remove timestamps older than moving average window
            long windowNanos = (long) (MOVING_AVERAGE_WINDOW_SECONDS * 1_000_000_000.0);
            while (!frameTimestamps.isEmpty() && (now - frameTimestamps.peek()) > windowNanos) {
                frameTimestamps.remove();
            }
            
            double fps = 0.0;
            if (frameTimestamps.size() >= 2) {
                long oldestValue = frameTimestamps.peek();
                int frameCount = frameTimestamps.size() - 1;  // number of intervals
                double timeWindowSeconds = (now - oldestValue) / 1_000_000_000.0;
                if (timeWindowSeconds > 0) {
                    fps = frameCount / timeWindowSeconds;
                }
            }

            // Get detections directly - VisionPortal handles frame processing automatically
            List<AprilTagDetection> currentDetections = aprilTagReader.getDetections();

            telemetry.addData("Frequency (Hz)", fps);
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            if (currentDetections.size() > 0) {
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.ftcPose != null) {
                        telemetry.addData("Tag ID", detection.id);

                        Matrix hTagToCamera = aprilTagReader.getTagToCameraMatrix(detection.id);
                        telemetry.addData("Tag->Camera T (m)",
                                "x=%.3f, y=%.3f, z=%.3f",
                                hTagToCamera.get(0, 3),
                                hTagToCamera.get(1, 3),
                                hTagToCamera.get(2, 3));
                        telemetry.addData("Tag->Camera R (deg)",
                                "roll=%.1f, pitch=%.1f, yaw=%.1f",
                                Math.toDegrees(Math.atan2(hTagToCamera.get(2, 1), hTagToCamera.get(2, 2))),
                                Math.toDegrees(Math.atan2(-hTagToCamera.get(2, 0),
                                        Math.sqrt(Math.pow(hTagToCamera.get(2, 1), 2) + Math.pow(hTagToCamera.get(2, 2), 2)))),
                                Math.toDegrees(Math.atan2(hTagToCamera.get(1, 0), hTagToCamera.get(0, 0))));
                    }
                }
            } else {
                telemetry.addData("Status", "No AprilTags detected");
            }

            telemetry.update();
        }

        // Cleanup
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
