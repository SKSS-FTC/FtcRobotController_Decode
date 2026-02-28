package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.subsystem.RobotState.currentPose;

import android.util.Size;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.subsystem.AprilTagReader;
import org.firstinspires.ftc.teamcode.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Localizer;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Slider;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import Jama.Matrix;

@TeleOp(name = "ShooterAlgorithmTest2", group = "Tests")
public class TeleOP_Test2 extends LinearOpMode {
    private Slider slider;
    private DriveTrain driveTrain;
    private Shooter shooter;
    private Intake intake;
    private TelemetryManager telemetryM;
    private Localizer localizer;
    private boolean wasLogging = false;
    private Pose STARTING_POSE = new Pose(0,0,0);
    private AprilTagReader aprilTagReader;
    private VisionPortal visionPortal;
    private java.util.Queue<Long> frameTimestamps = new java.util.LinkedList<>();
    private static final double MOVING_AVERAGE_WINDOW_SECONDS = 5.0;

    @Override
    public void runOpMode() throws InterruptedException {
        slider = new Slider(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap);
        shooter = new Shooter(hardwareMap, Shooter.Alliance.RED);
        intake = new Intake(hardwareMap);
        if(currentPose != null){
            STARTING_POSE = currentPose;
        }
        localizer = new Localizer.Builder()
                .webcamName("Webcam 1")
                .startingPose(STARTING_POSE)
                .build();
        aprilTagReader = new AprilTagReader();

        // TODO: Update AprilTag initialization to match current SDK API
        // AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
        //         .setTagFamily(AprilTagGameDatabase.getCurrentGameTagFamily())
        //         .setTagLibrary(AprilTagGameDatabase.getTagLibrary())
        //         .build();
        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
//                .setLensIntrinsics(1420.410149146399, 1422.8435764951637, 1026.5786658861796, 565.243885883523)
                .setLensIntrinsics(1426.10, 1424.95, 1075.43, 551.19)
                .build();

        aprilTagReader.setProcessor(aprilTagProcessor);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.addProcessor(aprilTagProcessor);

        // Set camera (webcam will be selected if configured)
        CameraName cameraName = hardwareMap.get(CameraName.class, "Webcam 1");
        builder.setCamera(cameraName);
        builder.setCameraResolution(new Size(1920, 1080));

        // Build and start portal
        visionPortal = builder.build();
        visionPortal.resumeStreaming();
        intake.stop();

        telemetry.addData("Status", "Vision Portal initialized");
        telemetry.addData("Status", "AprilTag detection ready. Press START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            shooter.shooterAiming = true;
            localizer.update();
            driveTrain.setFieldDrive(true);
            driveTrain.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            // Get detections directly - VisionPortal handles frame processing automatically
            List<AprilTagDetection> currentDetections = aprilTagReader.getDetections();
            double frameRateHz = updateFrameRateHz();

            telemetry.addData("Frame Rate (Hz)", String.format("%.2f", frameRateHz));
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            if (currentDetections.size() > 0) {
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.ftcPose != null) {
                        telemetry.addData("Tag ID", detection.id);

                        Matrix hTagToCamera = aprilTagReader.getTagToCameraMatrix(detection.id);
//                        telemetry.addData("Tag->Camera T (m)",
//                                "x=%.3f, y=%.3f, z=%.3f",
//                                hTagToCamera.get(0, 3),
//                                hTagToCamera.get(1, 3),
//                                hTagToCamera.get(2, 3));
                        telemetry.addData("Tag->Camera R (deg)",
                                "roll=%.1f, pitch=%.1f, yaw=%.1f",
                                Math.toDegrees(Math.atan2(hTagToCamera.get(2, 1), hTagToCamera.get(2, 2))),
                                Math.toDegrees(Math.atan2(-hTagToCamera.get(2, 0),
                                        Math.sqrt(Math.pow(hTagToCamera.get(2, 1), 2) + Math.pow(hTagToCamera.get(2, 2), 2)))),
                                Math.toDegrees(Math.atan2(hTagToCamera.get(1, 0), hTagToCamera.get(0, 0))));

                        Matrix hCameraToTag = aprilTagReader.getCameraToTagMatrix(detection.id);
                        telemetry.addData("Camera->Tag T (m)",
                                "x=%.3f, y=%.3f, z=%.3f",
                                hCameraToTag.get(0, 3),
                                hCameraToTag.get(1, 3),
                                hCameraToTag.get(2, 3));
                        double distance = Math.sqrt(Math.pow(hCameraToTag.get(0, 3), 2) + Math.pow(hCameraToTag.get(1, 3), 2));
                        telemetry.addData("distance from cam to tag", distance);
                    }
                }
            } else {
                telemetry.addData("Status", "No AprilTags detected");
            }
            if (gamepad1.triangle) {
                shooter.shooterRotate = true;
            } else if (gamepad1.cross) {
                shooter.shooterRotate = false;
            }

            if (gamepad1.dpad_up) {
                intake.reserve();
            } else if (gamepad1.left_trigger > 0) {
                intake.intake();
                shooter.closeGate();
            } else if (gamepad1.right_trigger > 0.5) {
                if (shooter.getGateClosed()) {
                    intake.intake();
                    intake.shoot();
                }else {
                    shooter.openGate();
                }
            } else {
                intake.stop();
            }
            if (gamepad1.dpad_left && gamepad1.b) {
                slider.park();
            } else {
                slider.drive();
            }
        }
        intake.update();
        shooter.update(localizer.getPose(),localizer.getHeading());
        telemetry.addData("kicker timer", intake.shootTimer.milliseconds());
        telemetry.addData("angular velocity", shooter.getShootVelocity());

        shooter.update(STARTING_POSE, 0);

        telemetry.update();
        sleep(20);
        // Cleanup
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
