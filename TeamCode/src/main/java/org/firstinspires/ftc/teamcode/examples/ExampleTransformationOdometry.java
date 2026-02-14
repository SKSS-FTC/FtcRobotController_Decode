package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.subsystem.AprilTagReader;
import org.firstinspires.ftc.teamcode.subsystem.Transformation;
import org.firstinspires.ftc.teamcode.subsystem.Transformation.RobotPose;
import org.firstinspires.ftc.teamcode.subsystem.Constants;

import Jama.Matrix;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "ExampleTransformationOdometry", group = "Examples")
public class ExampleTransformationOdometry extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private AprilTagReader aprilTagReader;
    private Transformation transformation;
    private java.util.Queue<Long> frameTimestamps = new java.util.LinkedList<>();
    private static final double MOVING_AVERAGE_WINDOW_SECONDS = 5.0;

    @Override
    public void runOpMode() {
        try {
            telemetry.addData("Status", "Initializing");
            telemetry.update();

            List<WebcamName> webcams = hardwareMap.getAll(WebcamName.class);
            if (webcams == null || webcams.isEmpty()) {
                telemetry.addData("Init Error", "No webcam found in hardware config");
                telemetry.update();
                sleep(2000);
                return;
            }

            aprilTag = new AprilTagProcessor.Builder().build();

            visionPortal = new VisionPortal.Builder()
                    .setCamera(webcams.get(0))
                    .addProcessor(aprilTag)
                    .build();

            aprilTagReader = new AprilTagReader();
            aprilTagReader.setProcessor(aprilTag);

            transformation = new Transformation();
            transformation.initialize();

            // Register AprilTag positions in map frame (4x4 homogeneous matrices)
            transformation.registerAprilTag(Constants.BLUE_GOAL_TAG_ID, Constants.TAG_20_TO_MAP);
            transformation.registerAprilTag(Constants.RED_GOAL_TAG_ID, Constants.TAG_24_TO_MAP);
        } catch (Throwable t) {
            telemetry.addData("Init Crash", t.getClass().getSimpleName());
            telemetry.addData("Message", t.getMessage());
            telemetry.update();
            sleep(3000);
            return;
        }

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

            try {
                List<AprilTagDetection> detections = aprilTagReader.getDetections();

                if (detections.isEmpty()) {
                    telemetry.addData("AprilTags", "No tags detected");
                    telemetry.addData("Frequency (Hz)", fps);
                    telemetry.update();
                    sleep(100);
                    continue;
                }

                telemetry.addData("Frequency (Hz)", fps);
                telemetry.addData("Total Tags Detected", detections.size());

                // Single tag odometry
                for (AprilTagDetection detection : detections) {
                    int tagId = detection.id;
                    Matrix H_camera_to_tag = aprilTagReader.getCameraToTagMatrix(tagId);

                    if (H_camera_to_tag != null) {
                        RobotPose pose = transformation.getRobotPoseInMap(tagId, H_camera_to_tag);
                        if (pose == null) {
                            continue;
                        }

                        double yawRad = Math.atan2(pose.rotationMatrix[1][0], pose.rotationMatrix[0][0]);
                        double yawDeg = Math.toDegrees(yawRad);

                        telemetry.addData(String.format("Tag %d Pose [M]", tagId),
                            "X_M: %.3f, Y_M: %.3f, Z_M: %.3f (m), Yaw_M: %.1f deg",
                                pose.translation[0],
                                pose.translation[1],
                                pose.translation[2],
                                yawDeg);
                    }
                }

                // Multi-tag robust odometry (if 2+ tags detected)
                if (detections.size() >= 2) {
                    List<Transformation.TagDetection> tagDetections = new ArrayList<>();
                    for (AprilTagDetection detection : detections) {
                        Matrix H_camera_to_tag = aprilTagReader.getCameraToTagMatrix(detection.id);
                        if (H_camera_to_tag != null) {
                            tagDetections.add(new Transformation.TagDetection(detection.id, H_camera_to_tag));
                        }
                    }

                    RobotPose robustPose = transformation.getRobotPoseInMapFromMultipleTags(tagDetections);
                    if (robustPose != null) {
                        double robustYawRad = Math.atan2(robustPose.rotationMatrix[1][0], robustPose.rotationMatrix[0][0]);
                        double robustYawDeg = Math.toDegrees(robustYawRad);
                        telemetry.addData("Multi-Tag Averaged Pose [M]",
                            "X_M: %.3f, Y_M: %.3f, Z_M: %.3f (m), Yaw_M: %.1f deg",
                            robustPose.translation[0],
                            robustPose.translation[1],
                            robustPose.translation[2],
                            robustYawDeg);
                    }
                }

                telemetry.update();
                // sleep(100);
            } catch (Throwable t) {
                telemetry.addData("Runtime Crash", t.getClass().getSimpleName());
                telemetry.addData("Message", t.getMessage());
                telemetry.update();
                sleep(200);
            }
        }

        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
