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
            transformation.registerAprilTag(Constants.BLUE_GOAL_TAG_ID, Constants.getTag20ToMapTransform());
            transformation.registerAprilTag(Constants.RED_GOAL_TAG_ID, Constants.getTag24ToMapTransform());
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

                        telemetry.addData(String.format("Tag %d Pose", tagId),
                                "X: %.3f, Y: %.3f, Z: %.3f (m)",
                                pose.translation[0],
                                pose.translation[1],
                                pose.translation[2]);
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
                        telemetry.addData("Multi-Tag Averaged Pose",
                            "X: %.3f, Y: %.3f, Z: %.3f (m)",
                            robustPose.translation[0],
                            robustPose.translation[1],
                            robustPose.translation[2]);
                    }
                }

                telemetry.update();
                sleep(100);
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
