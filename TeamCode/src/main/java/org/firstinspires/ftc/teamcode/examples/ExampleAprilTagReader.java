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
            // Get detections directly - VisionPortal handles frame processing automatically
            List<AprilTagDetection> currentDetections = aprilTagReader.getDetections();

            telemetry.addData("# AprilTags Detected", currentDetections.size());

            if (currentDetections.size() > 0) {
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.ftcPose != null) {
                        telemetry.addData("Tag ID", detection.id);
                        telemetry.addData(" XYZ (inch) | RPY (deg)",
                                "x=%.3f, y=%.3f, z=%.3f | r=%.1f, p=%.1f, y=%.1f",
                                detection.ftcPose.x,
                                detection.ftcPose.y,
                                detection.ftcPose.z,
                                detection.ftcPose.roll,
                                detection.ftcPose.pitch,
                                detection.ftcPose.yaw);

                        Matrix hTagToCamera = aprilTagReader.getTagToCameraMatrix(detection.id);
                        telemetry.addData("Tag->Camera T (m)",
                                "x=%.3f, y=%.3f, z=%.3f",
                                hTagToCamera.get(0, 3),
                                hTagToCamera.get(1, 3),
                                hTagToCamera.get(2, 3));
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
