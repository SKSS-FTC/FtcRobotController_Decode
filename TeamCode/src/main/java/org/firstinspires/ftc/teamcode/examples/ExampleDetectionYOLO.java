package org.firstinspires.ftc.teamcode.examples;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystem.DetectionYOLO;
import org.firstinspires.ftc.vision.VisionPortal;
import java.util.List;
import java.util.Locale;

@Config
@Autonomous(name = "Example: YOLO Detection", group = "Examples")
public class ExampleDetectionYOLO extends LinearOpMode {
    private static final String MODEL_PATH = "best_fp16.tflite";
    private static final String[] WEBCAM_NAMES = {"Webcam 1", "Webcam 2"};

    public static int WEBCAM_INDEX = 0;
    public static boolean SWITCH_CAMERA = false;

    private DetectionYOLO detector;
    private VisionPortal visionPortal;
    private int currentWebcamIndex = 0;
    private boolean lastSwitchCamera = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        currentWebcamIndex = Math.max(0, Math.min(WEBCAM_INDEX, WEBCAM_NAMES.length - 1));

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

        buildVisionPortal();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Model", detector.getModelPath());
        telemetry.addData("Camera", WEBCAM_NAMES[currentWebcamIndex]);
        telemetry.addData("Dashboard", "Set WEBCAM_INDEX (0-1) then toggle SWITCH_CAMERA");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (SWITCH_CAMERA && !lastSwitchCamera) {
                int newIndex = Math.max(0, Math.min(WEBCAM_INDEX, WEBCAM_NAMES.length - 1));
                if (newIndex != currentWebcamIndex) {
                    switchCamera(newIndex);
                }
            }
            lastSwitchCamera = SWITCH_CAMERA;

            List<DetectionYOLO.Detection> detections = detector.getLastDetectionsSnapshot();
                telemetry.addData("Camera", WEBCAM_NAMES[currentWebcamIndex]);
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

        detector.close();
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    private void buildVisionPortal() {
        WebcamName webcamName = null;
        try {
            webcamName = hardwareMap.get(WebcamName.class, WEBCAM_NAMES[currentWebcamIndex]);
        } catch (Exception ignored) {
            List<WebcamName> webcams = hardwareMap.getAll(WebcamName.class);
            if (!webcams.isEmpty()) {
                webcamName = webcams.get(currentWebcamIndex < webcams.size() ? currentWebcamIndex : 0);
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
    }

    private void switchCamera(int newWebcamIndex) {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }

        sleep(100);

        currentWebcamIndex = newWebcamIndex;
        buildVisionPortal();

        telemetry.addData("Camera Switched", WEBCAM_NAMES[currentWebcamIndex]);
    }
}
