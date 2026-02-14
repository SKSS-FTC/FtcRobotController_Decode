package org.firstinspires.ftc.teamcode.examples;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.subsystem.SegmentationYOLO;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Example OpMode demonstrating YOLO v8 instance segmentation with live camera stream
 * and segmentation overlays in Vision Portal preview.
 */
@Autonomous(name = "Concept: YOLO Segmentation", group = "AI")
public class ExampleSegmentationYOLO extends LinearOpMode {
    private static final String MODEL_PATH = "yolov8n-seg.tflite";
    private static final String WEBCAM_NAME = "Webcam 1";
    
    private SegmentationYOLO segmentor;
    private VisionPortal visionPortal;
    private SegmentationYOLO.Processor segmentationProcessor;
    private String activeCameraName = "unknown";
    private String activeStreamConfig = "unknown";



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        
        try {
            Context context = hardwareMap.appContext;
            
            segmentor = new SegmentationYOLO.Builder(context)
                    .setModelPath(MODEL_PATH)
                    .setNumThreads(1)
                    .setConfidenceThreshold(0.25f)
                    .setIOUThreshold(0.45f)
                    .build();
            
            segmentor.init();

                if (!segmentor.isReady()) {
                telemetry.addData("ERROR", segmentor.getInitError() != null
                    ? segmentor.getInitError()
                    : "YOLO segmentor failed to initialize");
                telemetry.update();
                sleep(2000);
                return;
                }

            segmentationProcessor = segmentor.createProcessor();

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

            visionPortal = buildVisionPortalWithFallback(selectedWebcam, segmentationProcessor);
            if (visionPortal == null) {
                telemetry.addData("ERROR", "Camera failed to start stream");
                telemetry.addData("Camera", activeCameraName);
                telemetry.update();
                sleep(2000);
                return;
            }
            
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Model", segmentor.getModelPath());
            telemetry.addData("DetectOut", segmentor.getDetectionOutputShape() != null
                    ? Arrays.toString(segmentor.getDetectionOutputShape()) : "unknown");
            telemetry.addData("MaskOut", segmentor.getMaskOutputShape() != null
                    ? Arrays.toString(segmentor.getMaskOutputShape()) : "unknown");
            telemetry.addData("Camera", activeCameraName);
            telemetry.addData("Stream", activeStreamConfig);
            telemetry.addData("Vision", "Camera stream + Segmentation overlay ready");
            telemetry.update();
            
        } catch (Exception e) {
            telemetry.addData("ERROR", "Init: " + e.getMessage());
            telemetry.update();
            sleep(2000);
            return;
        }
        
        waitForStart();

        if (segmentor != null) {
            segmentor.resetBenchmark();
        }

        try {
            while (opModeIsActive() && !isStopRequested()) {
                List<SegmentationYOLO.Segmentation> segmentations = segmentationProcessor != null
                        ? segmentationProcessor.getLastSegmentations()
                        : new ArrayList<>();

                telemetry.addData("YOLO", "Segmentor active");
                telemetry.addData("Model", segmentor.getModelPath());
                telemetry.addData("CameraSrc", activeCameraName);
                telemetry.addData("Stream", activeStreamConfig);
                telemetry.addData("Camera", visionPortal != null ? visionPortal.getCameraState() : "N/A");
                telemetry.addData("FPS", String.format("%.2f (avg %.2f)", segmentor.getFps(), segmentor.getFpsAverage()));
                telemetry.addData("Inference", String.format("%.1f ms (avg %.1f ms)", segmentor.getInferenceTimeMs(), segmentor.getInferenceTimeAverageMs()));
                telemetry.addData("Inference FPS", String.format("%.2f (avg %.2f)", segmentor.getInferenceFps(), segmentor.getInferenceFpsAverage()));
                telemetry.addData("Segments", segmentations.size());
                int maskPixels = 0;
                if (!segmentations.isEmpty() && segmentations.get(0).mask != null) {
                    for (float v : segmentations.get(0).mask) {
                        if (v > 0.5f) {
                            maskPixels++;
                        }
                    }
                }
                telemetry.addData("MaskPx", maskPixels);
                for (int i = 0; i < Math.min(3, segmentations.size()); i++) {
                    SegmentationYOLO.Segmentation seg = segmentations.get(i);
                    telemetry.addData(
                            "Seg " + i,
                            seg.className + String.format(" %.1f%%", seg.confidence * 100f)
                    );
                }
                telemetry.update();

                sleep(10);
            }
        } finally {
            boolean stopRequested = isStopRequested();

            if (segmentationProcessor != null) {
                segmentationProcessor.shutdown();
            }

            if (visionPortal != null) {
                try {
                    if (segmentationProcessor != null) {
                        visionPortal.setProcessorEnabled(segmentationProcessor, false);
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

            if (segmentor != null) {
                if (!stopRequested) {
                    segmentor.close();
                }
            }
        }
    }

    private VisionPortal buildVisionPortalWithFallback(WebcamName webcamName, SegmentationYOLO.Processor processor) {
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
