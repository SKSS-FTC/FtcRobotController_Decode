package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.DetectionYOLO;
import org.firstinspires.ftc.teamcode.subsystem.Constants;

import java.util.List;
import java.util.Locale;

@TeleOp(name = "Detection Test", group = "Tests")
public class DetectionTest extends LinearOpMode {
    private DetectionYOLO detector;
    private String filterTag = null;
    private boolean showAll = true;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing detection system...");
        telemetry.update();

        detector = new DetectionYOLO.Builder()
                .modelPath("yolo26n_int8.tflite")
                .frameSize(640, 480)
                .confidenceThreshold(0.25f)
                .numThreads(4)
                .build();

        detector.init(hardwareMap.appContext, hardwareMap);

        if (!detector.isInitialized()) {
            telemetry.addData("ERROR", detector.getInitError());
            telemetry.update();
            sleep(3000);
            return;
        }

        telemetry.addData("Status", "Detection system initialized");
        telemetry.addData("Focal Length", "%.1f px", Constants.CAMERA_FOCAL_LENGTH);
        telemetry.addData("Object Width", "%.4f m", Constants.KNOWN_OBJECT_WIDTH);
        telemetry.addData("", "Press START to begin detection test");
        telemetry.addData("Controls", "");
        telemetry.addData("  CROSS", "Toggle show all / filtered");
        telemetry.addData("  CIRCLE", "Filter to last detected tag");
        telemetry.addData("  SQUARE", "Clear filter");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            testControls();

            List<DetectionYOLO.DetectionResult> detections;
            if (filterTag != null && !showAll) {
                detections = detector.getDetectionResultsByTag(filterTag);
            } else {
                detections = detector.getDetectionResults();
            }

            telemetry.addData("Status", "Detection Test Running");
            telemetry.addData("Mode", showAll ? "Show All" : "Filter: " + filterTag);
            telemetry.addData("Focal Length", "%.1f px", Constants.CAMERA_FOCAL_LENGTH);
            telemetry.addData("Detections", detections.size());
            telemetry.addLine();

            if (detections.isEmpty()) {
                telemetry.addData("No objects detected", "Point camera at objects");
            } else {
                DetectionYOLO.DetectionResult best = detector.getBestDetectionResult();
                telemetry.addData("Best Detection", detector.getDetectionInfo(best));
                telemetry.addLine();
                telemetry.addData("All Detections:", "");

                int count = Math.min(5, detections.size());
                for (int i = 0; i < count; i++) {
                    DetectionYOLO.DetectionResult d = detections.get(i);
                    telemetry.addData(
                            String.format(Locale.US, "  [%d] %s", i + 1, d.tag),
                            "%.1f%% | %.2fm | %.0fx%.0fpx",
                            d.confidence * 100,
                            d.estimatedDistance,
                            d.pixelWidth,
                            d.pixelHeight
                    );
                }

                if (detections.size() > 5) {
                    telemetry.addData("  ...", "+%d more", detections.size() - 5);
                }
            }

            telemetry.addLine();
            telemetry.addData("Controls:", "");
            telemetry.addData("  CROSS", showAll ? "Switch to filtered mode" : "Switch to show all");
            telemetry.addData("  CIRCLE", "Filter by tag: %s", filterTag != null ? filterTag : "(none)");
            telemetry.addData("  SQUARE", "Clear filter");
            telemetry.addData("  DPAD_UP/DOWN", "Adjust focal length");
            telemetry.addData("  DPAD_LEFT/RIGHT", "Adjust known object width");

            if (gamepad1.dpad_up) {
                Constants.CAMERA_FOCAL_LENGTH += 10;
                sleep(100);
            }
            if (gamepad1.dpad_down) {
                Constants.CAMERA_FOCAL_LENGTH = Math.max(100, Constants.CAMERA_FOCAL_LENGTH - 10);
                sleep(100);
            }
            if (gamepad1.dpad_right) {
                Constants.KNOWN_OBJECT_WIDTH += 0.01;
                sleep(100);
            }
            if (gamepad1.dpad_left) {
                Constants.KNOWN_OBJECT_WIDTH = Math.max(0.01, Constants.KNOWN_OBJECT_WIDTH - 0.01);
                sleep(100);
            }

            telemetry.update();
            sleep(50);
        }

        detector.close();
    }

    private void testControls() {
        if (gamepad1.cross) {
            showAll = !showAll;
            sleep(200);
        }

        if (gamepad1.circle) {
            DetectionYOLO.DetectionResult best = detector.getBestDetectionResult();
            if (best != null) {
                filterTag = best.tag;
                showAll = false;
            }
            sleep(200);
        }

        if (gamepad1.square) {
            filterTag = null;
            showAll = true;
            sleep(200);
        }
    }
}
