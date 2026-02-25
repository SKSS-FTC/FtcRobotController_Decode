package org.firstinspires.ftc.teamcode.examples;

import android.content.Context;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystem.DetectionYOLO;
import org.firstinspires.ftc.vision.VisionPortal;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

@Config
@Autonomous(name = "Example: YOLO Time Compare", group = "Examples")
public class ExampleDetectionTimeCompare extends LinearOpMode {
    private static final String[] WEBCAM_NAMES = {"Webcam 1", "Webcam 2"};

    public static int WEBCAM_INDEX = 0;
    public static boolean SWITCH_CAMERA = false;
    public static int MODEL_INDEX = 0;
    public static boolean SWITCH_MODEL = false;

    private static final String[] MODEL_PATHS = {
            "best_int8.tflite",
            "best_fp16.tflite",
            "best_fp32.tflite"
    };

    private static final String[] MODEL_NAMES = {
            "int8",
            "fp16",
            "fp32"
    };

    private DetectionYOLO detector;
    private VisionPortal visionPortal;

    private Map<String, List<Double>> modelFrameTimes = new HashMap<>();
    private List<Double> currentFrameTimes = new ArrayList<>();
    private ElapsedTime frameTimer = new ElapsedTime();
    private boolean isFirstFrame = true;

    private boolean isLogging = false;
    private boolean wasSquarePressed = false;
    private int currentModelIndex = 0;
    private int currentWebcamIndex = 0;
    private boolean lastSwitchModel = false;
    private boolean lastSwitchCamera = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        for (String name : MODEL_NAMES) {
            modelFrameTimes.put(name, new ArrayList<>());
        }

        currentModelIndex = MODEL_INDEX;
        currentWebcamIndex = Math.max(0, Math.min(WEBCAM_INDEX, WEBCAM_NAMES.length - 1));

        detector = new DetectionYOLO.Builder()
                .modelPath(MODEL_PATHS[currentModelIndex])
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
        telemetry.addData("Model", MODEL_NAMES[currentModelIndex] + " (" + MODEL_PATHS[currentModelIndex] + ")");
        telemetry.addData("Camera", WEBCAM_NAMES[currentWebcamIndex]);
        telemetry.addData("Press SQUARE", "Start/Stop timing log");
        telemetry.addData("Dashboard", "Set MODEL_INDEX (0-2) then toggle SWITCH_MODEL");
        telemetry.addData("Dashboard", "Set WEBCAM_INDEX (0-1) then toggle SWITCH_CAMERA");
        telemetry.update();

        waitForStart();

        frameTimer.reset();
        isFirstFrame = true;
        currentFrameTimes.clear();

        while (opModeIsActive() && !isStopRequested()) {
            if (SWITCH_MODEL && !lastSwitchModel) {
                if (isLogging) {
                    modelFrameTimes.get(MODEL_NAMES[currentModelIndex]).addAll(currentFrameTimes);
                    currentFrameTimes.clear();
                }

                int newIndex = Math.max(0, Math.min(MODEL_INDEX, MODEL_PATHS.length - 1));
                if (newIndex != currentModelIndex) {
                    switchModel(newIndex);
                    currentModelIndex = newIndex;
                    isFirstFrame = true;
                    frameTimer.reset();
                }
            }
            lastSwitchModel = SWITCH_MODEL;

            if (SWITCH_CAMERA && !lastSwitchCamera) {
                int newCamIndex = Math.max(0, Math.min(WEBCAM_INDEX, WEBCAM_NAMES.length - 1));
                if (newCamIndex != currentWebcamIndex) {
                    switchCamera(newCamIndex);
                }
            }
            lastSwitchCamera = SWITCH_CAMERA;

            if (gamepad1.square && !wasSquarePressed && !isLogging) {
                isLogging = true;
                currentFrameTimes.clear();
                isFirstFrame = true;
                frameTimer.reset();
                telemetry.addData("Logging", "STARTED for " + MODEL_NAMES[currentModelIndex]);
            } else if (gamepad1.square && !wasSquarePressed && isLogging) {
                isLogging = false;
                modelFrameTimes.get(MODEL_NAMES[currentModelIndex]).addAll(currentFrameTimes);
                currentFrameTimes.clear();
                telemetry.addData("Logging", "STOPPED for " + MODEL_NAMES[currentModelIndex]);
            }
            wasSquarePressed = gamepad1.square;

            if (isLogging) {
                List<DetectionYOLO.Detection> detections = detector.getLastDetectionsSnapshot();

                if (!isFirstFrame) {
                    double frameTimeMs = frameTimer.milliseconds();
                    currentFrameTimes.add(frameTimeMs);
                } else {
                    isFirstFrame = false;
                }
                frameTimer.reset();

                telemetry.addData("Model", MODEL_NAMES[currentModelIndex]);
                telemetry.addData("Camera", WEBCAM_NAMES[currentWebcamIndex]);
                telemetry.addData("Frame Time (ms)", currentFrameTimes.isEmpty() ? "N/A" : String.format("%.2f", currentFrameTimes.get(currentFrameTimes.size() - 1)));
                telemetry.addData("Frame Count", currentFrameTimes.size());
                telemetry.addData("Detections", detections.size());
            } else {
                telemetry.addData("Model", MODEL_NAMES[currentModelIndex]);
                telemetry.addData("Camera", WEBCAM_NAMES[currentWebcamIndex]);
                telemetry.addData("Logging", "OFF - Press SQUARE to start");
                for (String name : MODEL_NAMES) {
                    telemetry.addData(name + " frames", modelFrameTimes.get(name).size());
                }
            }

            telemetry.update();
            sleep(10);
        }

        if (isLogging && !currentFrameTimes.isEmpty()) {
            modelFrameTimes.get(MODEL_NAMES[currentModelIndex]).addAll(currentFrameTimes);
        }

        saveAllToCSV();

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

    private void switchModel(int newModelIndex) {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
        if (detector != null) {
            detector.close();
            detector = null;
        }

        sleep(100);

        detector = new DetectionYOLO.Builder()
                .modelPath(MODEL_PATHS[newModelIndex])
                .numThreads(4)
                .confidenceThreshold(0.25f)
                .build();

        detector.init(hardwareMap.appContext);

        if (!detector.isReady()) {
            telemetry.addData("ERROR", "Failed to load model: " + MODEL_PATHS[newModelIndex]);
            telemetry.addData("Error", detector.getInitError());
            telemetry.update();
            return;
        }

        buildVisionPortal();

        telemetry.addData("Model Switched", MODEL_NAMES[newModelIndex]);
        System.out.println("Switched to model: " + MODEL_NAMES[newModelIndex]);
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

    private void saveAllToCSV() {
        Context context = hardwareMap.appContext;
        File logDir = new File(context.getFilesDir(), "yolo_timing_logs");
        if (!logDir.exists()) {
            logDir.mkdirs();
        }

        String timestamp = new java.text.SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new java.util.Date());

        try {
            File combinedFile = new File(logDir, "yolo_comparison_" + timestamp + ".csv");
            BufferedWriter combinedWriter = new BufferedWriter(new FileWriter(combinedFile));
            
            StringBuilder header = new StringBuilder("frame_index");
            for (String name : MODEL_NAMES) {
                header.append(",").append(name).append("_ms");
            }
            combinedWriter.write(header.toString());
            combinedWriter.newLine();

            int maxFrames = 0;
            for (String name : MODEL_NAMES) {
                maxFrames = Math.max(maxFrames, modelFrameTimes.get(name).size());
            }

            for (int i = 0; i < maxFrames; i++) {
                StringBuilder line = new StringBuilder();
                line.append(i + 1);
                for (String name : MODEL_NAMES) {
                    List<Double> times = modelFrameTimes.get(name);
                    if (i < times.size()) {
                        line.append(String.format(Locale.US, ",%.4f", times.get(i)));
                    } else {
                        line.append(",");
                    }
                }
                combinedWriter.write(line.toString());
                combinedWriter.newLine();
            }

            combinedWriter.flush();
            combinedWriter.close();

            telemetry.addData("Combined CSV", combinedFile.getAbsolutePath());
            System.out.println("Combined YOLO timing data saved to: " + combinedFile.getAbsolutePath());

            for (String name : MODEL_NAMES) {
                List<Double> times = modelFrameTimes.get(name);
                if (!times.isEmpty()) {
                    File modelFile = new File(logDir, "yolo_" + name + "_" + timestamp + ".csv");
                    BufferedWriter writer = new BufferedWriter(new FileWriter(modelFile));
                    writer.write("frame_index,frame_time_ms");
                    writer.newLine();

                    for (int i = 0; i < times.size(); i++) {
                        writer.write(String.format(Locale.US, "%d,%.4f", i + 1, times.get(i)));
                        writer.newLine();
                    }

                    writer.flush();
                    writer.close();

                    double avg = 0, min = Double.MAX_VALUE, max = Double.MIN_VALUE;
                    for (double t : times) {
                        avg += t;
                        if (t < min) min = t;
                        if (t > max) max = t;
                    }
                    avg /= times.size();

                    telemetry.addData(name + " CSV", modelFile.getName());
                    telemetry.addData(name + " Stats", String.format("Avg: %.2f ms, Min: %.2f ms, Max: %.2f ms, Count: %d", avg, min, max, times.size()));
                }
            }
            telemetry.update();

        } catch (Exception e) {
            telemetry.addData("Error", "Failed to save CSV: " + e.getMessage());
            telemetry.update();
            System.err.println("Failed to save YOLO timing data: " + e.getMessage());
        }
    }
}
