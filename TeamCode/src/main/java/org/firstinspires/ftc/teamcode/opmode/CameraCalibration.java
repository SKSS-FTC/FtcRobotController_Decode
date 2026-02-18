package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.DetectionYOLO;
import org.firstinspires.ftc.teamcode.subsystem.Constants;

import java.util.List;

@TeleOp(name = "Camera Calibration", group = "Tests")
public class CameraCalibration extends LinearOpMode {
    private DetectionYOLO detector;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing camera...");
        telemetry.update();

        detector = new DetectionYOLO.Builder()
                .modelPath("yolo26n_int8.tflite")
                .frameSize(640, 480)
                .confidenceThreshold(0.25f)
                .build();

        detector.init(hardwareMap.appContext, hardwareMap);

        if (!detector.isInitialized()) {
            telemetry.addData("ERROR", detector.getInitError());
            telemetry.update();
            sleep(3000);
            return;
        }

        telemetry.addData("Status", "Ready for calibration");
        telemetry.addData("Instructions", "Place object at known distance");
        telemetry.addData("Distance", "Set CALIBRATION_ACTUAL_DISTANCE in Dashboard");
        telemetry.addData("Current Distance", "%.2f m", Constants.CALIBRATION_ACTUAL_DISTANCE);
        telemetry.addData("Object Width", "%.4f m (%.1f in)", Constants.KNOWN_OBJECT_WIDTH, Constants.KNOWN_OBJECT_WIDTH * 39.37);
        telemetry.addData("", "Press START when object is in view");
        telemetry.update();

        waitForStart();

        double calculatedFocalLength = 0;

        while (opModeIsActive() && !isStopRequested()) {
            List<DetectionYOLO.DetectionResult> detections = detector.getDetectionResults();

            telemetry.addData("Actual Distance (Dashboard)", "%.2f m", Constants.CALIBRATION_ACTUAL_DISTANCE);
            telemetry.addData("Current Focal Length", "%.1f px", Constants.CAMERA_FOCAL_LENGTH);
            telemetry.addData("Known Object Width", "%.4f m", Constants.KNOWN_OBJECT_WIDTH);
            telemetry.addData("Detections", detections.size());
            telemetry.addLine();

            if (!detections.isEmpty()) {
                DetectionYOLO.DetectionResult best = detections.get(0);

                telemetry.addData("Detected Object", best.tag);
                telemetry.addData("Confidence", "%.1f%%", best.confidence * 100);
                telemetry.addData("Pixel Width", "%.1f px", best.pixelWidth);
                telemetry.addData("Pixel Height", "%.1f px", best.pixelHeight);
                telemetry.addLine();

                if (Constants.CALIBRATION_ACTUAL_DISTANCE > 0) {
                    calculatedFocalLength = detector.calculateFocalLength(
                            Constants.CALIBRATION_ACTUAL_DISTANCE,
                            best.pixelWidth
                    );

                    telemetry.addData("Calculated Focal Length", "%.1f px", calculatedFocalLength);
                    telemetry.addLine();
                    telemetry.addData("ACTION", "Press CROSS to save focal length");

                    if (gamepad1.cross) {
                        Constants.CAMERA_FOCAL_LENGTH = calculatedFocalLength;
                        telemetry.addLine();
                        telemetry.addData("SAVED", "Focal length = %.1f px", Constants.CAMERA_FOCAL_LENGTH);
                    }
                } else {
                    telemetry.addData("WARNING", "Set CALIBRATION_ACTUAL_DISTANCE > 0 in Dashboard");
                }
            } else {
                telemetry.addData("No detections", "Point camera at object");
            }

            telemetry.addLine();
            telemetry.addData("Press TRIANGLE", "To test current focal length");

            if (gamepad1.triangle && !detections.isEmpty()) {
                DetectionYOLO.DetectionResult best = detections.get(0);
                double estimatedDist = detector.calculateDistance(best.pixelWidth);
                telemetry.addLine();
                telemetry.addData("Distance Test", "");
                telemetry.addData("  Estimated Distance", "%.2f m", estimatedDist);
                telemetry.addData("  Actual Distance", "%.2f m", Constants.CALIBRATION_ACTUAL_DISTANCE);
                if (Constants.CALIBRATION_ACTUAL_DISTANCE > 0) {
                    double error = Math.abs(estimatedDist - Constants.CALIBRATION_ACTUAL_DISTANCE);
                    double errorPercent = (error / Constants.CALIBRATION_ACTUAL_DISTANCE) * 100;
                    telemetry.addData("  Error", "%.2f m (%.1f%%)", error, errorPercent);
                }
            }

            telemetry.update();
            sleep(100);
        }

        detector.close();
    }
}
