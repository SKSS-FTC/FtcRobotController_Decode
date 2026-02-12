package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.subsystem.AprilTagReader;
import org.firstinspires.ftc.teamcode.subsystem.Transformation;

import Jama.Matrix;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "ExampleAprilTagReader", group = "Examples")
public class ExampleAprilTagReader extends LinearOpMode {

    private AprilTagReader aprilTagReader;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        // Initialize AprilTagReader subsystem
        aprilTagReader = new AprilTagReader();

        // Initialize VisionPortal with camera and AprilTag processor
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Add AprilTag processor from the subsystem
        builder.addProcessor(aprilTagReader.getProcessor());

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
            // Get detections directly - VisionPortal handles frame processing automatically
            List<AprilTagDetection> currentDetections = aprilTagReader.getDetections();

            telemetry.addData("# AprilTags Detected", currentDetections.size());

            if (currentDetections.size() > 0) {
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) {
                        telemetry.addData("Tag ID", detection.id);
                    telemetry.addData(" XYZ (inch) | RPY (deg)",
                            "x=%.3f, y=%.3f, z=%.3f | r=%.1f, p=%.1f, y=%.1f",
                            detection.ftcPose.x,
                            detection.ftcPose.y,
                            detection.ftcPose.z,
                            detection.ftcPose.roll,
                            detection.ftcPose.pitch,
                            detection.ftcPose.yaw);
                    }
                }

                // Convert to Transformation.TagDetection and compute robot pose
                List<Transformation.TagDetection> tagDetections = new ArrayList<>();
                for (AprilTagDetection detection : currentDetections) {
                    Matrix H_camera_to_tag = aprilTagReader.getCameraToTagMatrix(detection.id);
                    if (H_camera_to_tag != null) {
                        tagDetections.add(new Transformation.TagDetection(detection.id, H_camera_to_tag));
                    }
                }

                Transformation.RobotPose robotPose = Transformation.getRobotPoseInMapFromMultipleTags(tagDetections);

                if (robotPose != null) {
                    double[] position = robotPose.getPosition();
                    telemetry.addData("Robot Position (meters)",
                            "X: %.3f, Y: %.3f, Z: %.3f",
                            position[0], position[1], position[2]);

                    double[][] R = robotPose.getOrientationMatrix();
                    double yaw = Math.atan2(R[1][0], R[0][0]);
                    double yawDegrees = Math.toDegrees(yaw);
                    telemetry.addData("Robot Yaw", "%.2f degrees", yawDegrees);
                } else {
                    telemetry.addData("Robot Pose", "Unable to compute (tags not registered?)");
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
