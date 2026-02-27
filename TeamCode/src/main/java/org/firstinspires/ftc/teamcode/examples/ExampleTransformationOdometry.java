package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.subsystem.AprilTagReader;
import org.firstinspires.ftc.teamcode.subsystem.Transformation;
import org.firstinspires.ftc.teamcode.subsystem.Transformation.RobotPose;
import org.firstinspires.ftc.teamcode.subsystem.Constants;

import android.util.Size;


import Jama.Matrix;
import java.util.ArrayList;
import java.util.Comparator;
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

            aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(1420.410149146399, 1422.8435764951637, 1026.5786658861796, 565.243885883523)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS)
                .build();

            visionPortal = new VisionPortal.Builder()
                    .setCamera(webcams.get(0))
                    .addProcessor(aprilTag)
                    .setCameraResolution(new Size(1920, 1080))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .build();

            aprilTagReader = new AprilTagReader();
            aprilTagReader.setProcessor(aprilTag);
            aprilTag.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SQPNP);

            transformation = new Transformation();
            transformation.initialize();

            // Register AprilTag positions in map frame (4x4 homogeneous matrices)
            transformation.registerAprilTag(Constants.BLUE_GOAL_TAG_ID, Constants.BLUE_GOAL_TAG_MAP_FRAME);
            transformation.registerAprilTag(Constants.RED_GOAL_TAG_ID, Constants.RED_GOAL_TAG_MAP_FRAME);

            // Sanity-check: print tag positions in map frame (should all be positive)
            // Map origin = bottom-left corner (0,0). Field = 3.658m x 3.658m.
            // TAG_MAP_FRAME is ^map T_tag — translation column IS the tag position in map.
            telemetry.addData("Tag 20 map pos",
                "X:%.3f Y:%.3f Z:%.3f",
                Constants.BLUE_GOAL_TAG_MAP_FRAME.get(0,3),
                Constants.BLUE_GOAL_TAG_MAP_FRAME.get(1,3),
                Constants.BLUE_GOAL_TAG_MAP_FRAME.get(2,3));
            telemetry.addData("Tag 24 map pos",
                "X:%.3f Y:%.3f Z:%.3f",
                Constants.RED_GOAL_TAG_MAP_FRAME.get(0,3),
                Constants.RED_GOAL_TAG_MAP_FRAME.get(1,3),
                Constants.RED_GOAL_TAG_MAP_FRAME.get(2,3));
            // Extract yaw from Rz rotation block: yaw = atan2(R[1][0], R[0][0])
            telemetry.addData("Tag 24 yaw (deg)", "%.1f",
                Math.toDegrees(Math.atan2(
                    Constants.RED_GOAL_TAG_MAP_FRAME.get(1,0),
                    Constants.RED_GOAL_TAG_MAP_FRAME.get(0,0))));
            
            telemetry.addData("Status", "Ready - press PLAY");
            telemetry.update();
        } catch (Throwable t) {
            telemetry.addData("Init Crash", t.getClass().getSimpleName());
            telemetry.addData("Message", t.getMessage());
            telemetry.update();
            sleep(3000);
            return;
        }

        waitForStart();

        while (opModeIsActive()) {
            try {
                List<AprilTagDetection> detections = aprilTagReader.getDetections();
                double frameRateHz = updateFrameRateHz();

                telemetry.addData("Frame Rate (Hz)", String.format("%.2f", frameRateHz));

                if (detections.isEmpty()) {
                    telemetry.addData("AprilTags", "No tags detected");
                    telemetry.update();
                    sleep(100);
                    continue;
                }

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

                        telemetry.addData(String.format("Camera to Tag %d", tagId),
                            "X: %.3f  Y: %.3f  Z: %.3f m",
                            H_camera_to_tag.get(0, 3),
                            H_camera_to_tag.get(1, 3),
                            H_camera_to_tag.get(2, 3));

                        double[] t = pose.translation;
                        double[][] R = pose.rotationMatrix;
                        double[] rpy = pose.getRPY(); // {roll, pitch, yaw} radians

                        telemetry.addData(String.format("Robot Pose from Tag %d", tagId),
                            "X: %.3f  Y: %.3f  Z: %.3f m",
                            t[0], t[1], t[2]);
                        telemetry.addData(String.format("Robot RPY from Tag %d", tagId),
                            "R: %.1f  P: %.1f  Y: %.1f deg",
                            Math.toDegrees(rpy[0]),
                            Math.toDegrees(rpy[1]),
                            Math.toDegrees(rpy[2]));   
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
                        double[] rt = robustPose.translation;
                        double[] rrpy = robustPose.getRPY(); // {roll, pitch, yaw} radians
                        telemetry.addData("Multi-Tag Base Pos [m]",
                            "X: %.3f  Y: %.3f  Z: %.3f", rt[0], rt[1], rt[2]);
                        telemetry.addData("Multi-Tag Base RPY [deg]",
                            "R: %.1f  P: %.1f  Y: %.1f",
                            Math.toDegrees(rrpy[0]),
                            Math.toDegrees(rrpy[1]),
                            Math.toDegrees(rrpy[2]));
                    }
                }

                telemetry.update();
                sleep(20);
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

    private double updateFrameRateHz() {
        long now = System.nanoTime();
        frameTimestamps.add(now);

        long windowNanos = (long) (MOVING_AVERAGE_WINDOW_SECONDS * 1_000_000_000.0);
        while (!frameTimestamps.isEmpty() && (now - frameTimestamps.peek()) > windowNanos) {
            frameTimestamps.remove();
        }

        if (frameTimestamps.size() < 2) {
            return 0.0;
        }

        long oldest = frameTimestamps.peek();
        int frames = frameTimestamps.size() - 1;
        double elapsedSeconds = (now - oldest) / 1_000_000_000.0;
        return elapsedSeconds > 0 ? frames / elapsedSeconds : 0.0;
    }
}
