package org.firstinspires.ftc.teamcode.examples;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.subsystem.DetectionYOLO;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import android.util.Size;

/**
 * Example OpMode demonstrating YOLO v8 object detection with live camera stream
 * and overlay drawings in Vision Portal preview.
 */
@Autonomous(name = "Detections YOLO", group = "AI")
public class ExampleDetectionYOLO extends LinearOpMode {
    private static final String MODEL_PATH = "yolov8n.tflite";
    private static final int INPUT_SIZE = 640;
    private static final String WEBCAM_NAME = "Webcam 1";
    
    private DetectionYOLO detector;
    private VisionPortal visionPortal;
    private DetectionProcessor detectionProcessor;
    private String activeCameraName = "unknown";
    private String activeStreamConfig = "unknown";

    private static class DetectionProcessor implements VisionProcessor {
        private final DetectionYOLO detector;
        private final Paint boxPaint = new Paint();
        private final Paint textPaint = new Paint();
        private volatile boolean enabled = true;
        private volatile List<DetectionYOLO.Detection> lastDetections = new ArrayList<>();

        DetectionProcessor(DetectionYOLO detector) {
            this.detector = detector;
            boxPaint.setColor(Color.GREEN);
            boxPaint.setStyle(Paint.Style.STROKE);
            boxPaint.setStrokeWidth(4);

            textPaint.setColor(Color.WHITE);
            textPaint.setTextSize(32f);
            textPaint.setAntiAlias(true);
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            // No-op
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            if (!enabled || detector == null || !detector.isReady()) {
                lastDetections = new ArrayList<>();
                return lastDetections;
            }

            Bitmap bitmap = Bitmap.createBitmap(frame.cols(), frame.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(frame, bitmap);

            List<DetectionYOLO.Detection> detections = detector.detect(bitmap);
            bitmap.recycle();

            lastDetections = detections;
            return detections;
        }

        @SuppressWarnings("unchecked")
        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            List<DetectionYOLO.Detection> detections;
            if (userContext instanceof List) {
                detections = (List<DetectionYOLO.Detection>) userContext;
            } else {
                detections = lastDetections;
            }

            if (detections == null) {
                return;
            }

            for (DetectionYOLO.Detection d : detections) {
                float left = d.x * onscreenWidth;
                float top = d.y * onscreenHeight;
                float right = (d.x + d.width) * onscreenWidth;
                float bottom = (d.y + d.height) * onscreenHeight;

                canvas.drawRect(left, top, right, bottom, boxPaint);
                canvas.drawText(
                        d.className + String.format(" %.0f%%", d.confidence * 100f),
                        left,
                        Math.max(32, top - 8),
                        textPaint
                );
            }
        }

        List<DetectionYOLO.Detection> getLastDetections() {
            return lastDetections;
        }

        void shutdown() {
            enabled = false;
            lastDetections = new ArrayList<>();
        }
    }
    
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        
        try {
            detector = new DetectionYOLO.Builder()
                    .modelPath(MODEL_PATH)
                    .inputSize(INPUT_SIZE)
                    .numThreads(1)
                    .telemetry(this.telemetry)
                    .showDebugInfo(true)
                    .build();
            
            detector.init(hardwareMap.appContext);

            if (!detector.isReady()) {
                telemetry.addData("ERROR", detector.getInitError() != null
                        ? detector.getInitError()
                        : "YOLO detector failed to initialize");
                telemetry.update();
                sleep(2000);
                return;
            }

                detectionProcessor = new DetectionProcessor(detector);

            WebcamName selectedWebcam = null;
            try {
                selectedWebcam = hardwareMap.get(WebcamName.class, WEBCAM_NAME);
                activeCameraName = WEBCAM_NAME;
            } catch (Exception ignored) {
                List<WebcamName> webcams = hardwareMap.getAll(WebcamName.class);
                if (!webcams.isEmpty()) {
                    selectedWebcam = webcams.get(0);
                    activeCameraName = selectedWebcam.toString();
                }
            }

            if (selectedWebcam == null) {
                activeCameraName = "Built-in Back Camera";
            }

            visionPortal = buildVisionPortalWithFallback(selectedWebcam, detectionProcessor);
            if (visionPortal == null) {
                telemetry.addData("ERROR", "Camera failed to start stream");
                telemetry.addData("Camera", activeCameraName);
                telemetry.update();
                sleep(2000);
                return;
            }
            
            telemetry.addData("Status", "Initialized");
                telemetry.addData("Model", detector.getModelPath());
                telemetry.addData("Input", detector.getInputTensorShape() != null
                    ? Arrays.toString(detector.getInputTensorShape())
                    : "unknown");
                telemetry.addData("Output", detector.getOutputTensorShape() != null
                    ? Arrays.toString(detector.getOutputTensorShape())
                    : "unknown");
                telemetry.addData("Camera", activeCameraName);
                telemetry.addData("Stream", activeStreamConfig);
            telemetry.addData("Vision", "Camera stream + YOLO overlay ready");
            telemetry.update();
            
        } catch (Exception e) {
            telemetry.addData("ERROR", "Init: " + e.getMessage());
                telemetry.addData("Camera", activeCameraName);
            telemetry.update();
            sleep(2000);
            return;
        }
        
        waitForStart();

        if (detector != null) {
            detector.resetBenchmark();
        }

        try {
            while (opModeIsActive() && !isStopRequested()) {
                List<DetectionYOLO.Detection> detections = detectionProcessor != null
                        ? detectionProcessor.getLastDetections()
                        : new ArrayList<>();

                telemetry.addData("YOLO", "Detector active");
                telemetry.addData("Model", detector.getModelPath());
                telemetry.addData("Layout", detector.getInputLayout());
                telemetry.addData("CameraSrc", activeCameraName);
                telemetry.addData("Stream", activeStreamConfig);
                telemetry.addData("Camera", visionPortal != null ? visionPortal.getCameraState() : "N/A");
                telemetry.addData("FPS", String.format("%.2f (avg %.2f)", detector.getFps(), detector.getFpsAverage()));
                telemetry.addData("Inference", String.format("%.1f ms (avg %.1f ms)", detector.getInferenceTimeMs(), detector.getInferenceTimeAverageMs()));
                telemetry.addData("Inference FPS", String.format("%.2f (avg %.2f)", detector.getInferenceFps(), detector.getInferenceFpsAverage()));
                telemetry.addData("Detections", detections.size());
                for (int i = 0; i < Math.min(3, detections.size()); i++) {
                    DetectionYOLO.Detection d = detections.get(i);
                    telemetry.addData(
                            "Obj " + i,
                            d.className + String.format(" %.1f%%", d.confidence * 100f)
                    );
                }
                telemetry.update();

                sleep(10);
            }
        } finally {
            boolean stopRequested = isStopRequested();

            if (detectionProcessor != null) {
                detectionProcessor.shutdown();
            }

            if (visionPortal != null) {
                try {
                    if (detectionProcessor != null) {
                        visionPortal.setProcessorEnabled(detectionProcessor, false);
                    }
                } catch (Exception ignored) {
                }

                try {
                    visionPortal.stopStreaming();
                } catch (Exception ignored) {
                }

                try {
                    visionPortal.stopLiveView();
                } catch (Exception ignored) {
                }

                if (!isStopRequested()) {
                    try {
                        visionPortal.close();
                    } catch (Exception ignored) {
                    }
                }
            }

            if (detector != null) {
                if (!stopRequested) {
                    detector.close();
                }
            }
        }
    }

    private VisionPortal buildVisionPortalWithFallback(WebcamName webcamName, DetectionProcessor processor) {
        VisionPortal defaultPortal = null;
        try {
            if (webcamName != null) {
                defaultPortal = VisionPortal.easyCreateWithDefaults(webcamName, processor);
            } else {
                defaultPortal = VisionPortal.easyCreateWithDefaults(BuiltinCameraDirection.BACK, processor);
            }

            if (waitForStreaming(defaultPortal, 10000)) {
                activeStreamConfig = "SDK default";
                return defaultPortal;
            }
        } catch (Exception ignored) {
            // Fall through to explicit stream format / resolution retries.
        }

        if (defaultPortal != null) {
            defaultPortal.close();
        }

        Size[] resolutions = new Size[]{
                new Size(640, 480),
                new Size(320, 240)
        };
        VisionPortal.StreamFormat[] formats = new VisionPortal.StreamFormat[]{
                VisionPortal.StreamFormat.MJPEG,
                VisionPortal.StreamFormat.YUY2
        };

        for (Size resolution : resolutions) {
            for (VisionPortal.StreamFormat format : formats) {
                VisionPortal trialPortal = null;
                try {
                    VisionPortal.Builder builder = new VisionPortal.Builder()
                            .setCameraResolution(resolution)
                            .setStreamFormat(format)
                            .enableLiveView(true)
                            .setAutoStartStreamOnBuild(true)
                            .addProcessor(processor);

                    if (webcamName != null) {
                        builder.setCamera(webcamName);
                    } else {
                        builder.setCamera(BuiltinCameraDirection.BACK);
                    }

                    trialPortal = builder.build();
                    trialPortal.resumeStreaming();
                    if (waitForStreaming(trialPortal, 10000)) {
                        activeStreamConfig = resolution.getWidth() + "x" + resolution.getHeight() + " " + format.name();
                        return trialPortal;
                    }
                } catch (Exception ignored) {
                    // Try next fallback.
                }

                if (trialPortal != null) {
                    trialPortal.close();
                }
            }
        }

        return null;
    }

    private boolean waitForStreaming(VisionPortal portal, long timeoutMs) {
        long deadline = System.currentTimeMillis() + timeoutMs;
        while (System.currentTimeMillis() < deadline && !isStopRequested()) {
            VisionPortal.CameraState state = portal.getCameraState();
            if (state == VisionPortal.CameraState.STREAMING) {
                return true;
            }
            if (state == VisionPortal.CameraState.ERROR) {
                return false;
            }
            sleep(50);
        }
        VisionPortal.CameraState finalState = portal.getCameraState();
        return finalState != VisionPortal.CameraState.ERROR;
    }
}
