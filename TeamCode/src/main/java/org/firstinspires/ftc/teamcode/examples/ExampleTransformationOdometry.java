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
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        aprilTagReader = new AprilTagReader();
        aprilTagReader.setProcessor(aprilTag);

        transformation = new Transformation();
        transformation.initialize();

        // Register AprilTag positions in map frame (4x4 homogeneous matrices)
        transformation.registerAprilTag(Constants.BLUE_GOAL_TAG_ID, Constants.TAG_20_TO_MAP);
        transformation.registerAprilTag(Constants.RED_GOAL_TAG_ID, Constants.TAG_24_TO_MAP);

        waitForStart();

        while (opModeIsActive()) {
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

                telemetry.addData("Multi-Tag Averaged Pose",
                        "X: %.3f, Y: %.3f, Z: %.3f (m)",
                        robustPose.translation[0],
                        robustPose.translation[1],
                        robustPose.translation[2]);
            }

            telemetry.update();
            sleep(100);
        }
    }
}
