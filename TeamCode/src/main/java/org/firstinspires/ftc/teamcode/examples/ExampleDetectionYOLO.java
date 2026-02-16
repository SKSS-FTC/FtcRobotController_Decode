package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystem.DetectionYOLO;
import org.firstinspires.ftc.vision.VisionPortal;
import java.util.List;
import java.util.Locale;

/**
 * Example OpMode demonstrating basic YOLO26 object detection.
 *
 * This example uses the simplified API:
 * - VisionPortal uses detector-provided VisionProcessor for detection + overlay
 * - Telemetry pulls latest detections from detector cache
 */
@Autonomous(name = "Example: YOLO Detection", group = "Examples")
public class ExampleDetectionYOLO extends LinearOpMode {
    private static final String MODEL_PATH = "yolo26n_int8.tflite";
    private static final String WEBCAM_NAME = "Webcam 1";

    private DetectionYOLO detector;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize detector with simplified API
        detector = new DetectionYOLO.Builder()
                .modelPath(MODEL_PATH)
                .numThreads(4)
                .confidenceThreshold(0.25f)
                .build();

        detector.init(hardwareMap.appContext);

        if (!detector.isReady()) {
            telemetry.addData("ERROR", detector.getInitError());
            telemetry.update();
            sleep(2000);
            return;
        }

        // Set up camera with detector-provided processor (does inference + overlay)
        WebcamName webcamName = null;
        try {
            webcamName = hardwareMap.get(WebcamName.class, WEBCAM_NAME);
        } catch (Exception ignored) {
            List<WebcamName> webcams = hardwareMap.getAll(WebcamName.class);
            if (!webcams.isEmpty()) {
                webcamName = webcams.get(0);
            }
        }
        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (webcamName != null) {
            builder.setCamera(webcamName);
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.addProcessor(detector.createVisionProcessor());
        visionPortal = builder.build();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Model", detector.getModelPath());
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            List<DetectionYOLO.Detection> detections = detector.getLastDetectionsSnapshot();
                telemetry.addData("Detections", detections.size());
                for (int i = 0; i < Math.min(3, detections.size()); i++) {
                DetectionYOLO.Detection d = detections.get(i);
                telemetry.addData(
                    "Det " + i,
                    String.format(Locale.US,
                        "%s %.1f%% at [%.4f, %.4f, %.4f, %.4f]",
                        d.className, d.confidence * 100f,
                        d.x, d.y, d.width, d.height));
                }

            telemetry.update();
            sleep(50);
        }

        // Cleanup
        detector.close();
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
