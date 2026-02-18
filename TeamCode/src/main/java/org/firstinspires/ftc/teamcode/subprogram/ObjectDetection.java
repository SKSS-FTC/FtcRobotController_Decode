package org.firstinspires.ftc.teamcode.subprogram;

import android.content.Context;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystem.Constants;
import org.firstinspires.ftc.teamcode.subsystem.DetectionYOLO;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class ObjectDetection {
    private DetectionYOLO detector;
    private VisionPortal visionPortal;
    private int frameWidth = 640;
    private int frameHeight = 480;
    private boolean initialized = false;

    public static class DetectionResult {
        public final String tag;
        public final double confidence;
        public final double pixelWidth;
        public final double pixelHeight;
        public final double normalizedWidth;
        public final double normalizedHeight;
        public final double estimatedDistance;
        public final double x;
        public final double y;

        public DetectionResult(String tag, double confidence, double pixelWidth, double pixelHeight,
                               double normalizedWidth, double normalizedHeight, double estimatedDistance,
                               double x, double y) {
            this.tag = tag;
            this.confidence = confidence;
            this.pixelWidth = pixelWidth;
            this.pixelHeight = pixelHeight;
            this.normalizedWidth = normalizedWidth;
            this.normalizedHeight = normalizedHeight;
            this.estimatedDistance = estimatedDistance;
            this.x = x;
            this.y = y;
        }
    }

    public static class Builder {
        private String modelPath = "yolo26n.tflite";
        private String labelsPath = null;
        private String webcamName = "Webcam 1";
        private int frameWidth = 640;
        private int frameHeight = 480;
        private float confidenceThreshold = 0.25f;
        private int numThreads = 4;
        private boolean useBuiltinCamera = false;

        public Builder modelPath(String path) { this.modelPath = path; return this; }
        public Builder labelsPath(String path) { this.labelsPath = path; return this; }
        public Builder webcamName(String name) { this.webcamName = name; return this; }
        public Builder frameSize(int width, int height) { this.frameWidth = width; this.frameHeight = height; return this; }
        public Builder confidenceThreshold(float threshold) { this.confidenceThreshold = threshold; return this; }
        public Builder numThreads(int threads) { this.numThreads = threads; return this; }
        public Builder useBuiltinCamera(boolean use) { this.useBuiltinCamera = use; return this; }

        public ObjectDetection build() {
            return new ObjectDetection(this);
        }
    }

    private ObjectDetection(Builder builder) {
        this.frameWidth = builder.frameWidth;
        this.frameHeight = builder.frameHeight;
    }

    public void init(Context context, HardwareMap hardwareMap) {
        init(context, hardwareMap, null);
    }

    public void init(Context context, HardwareMap hardwareMap, Builder config) {
        if (config == null) {
            config = new Builder();
        }

        detector = new DetectionYOLO.Builder()
                .modelPath(config.modelPath)
                .labelsPath(config.labelsPath)
                .confidenceThreshold(config.confidenceThreshold)
                .numThreads(config.numThreads)
                .build();

        detector.init(context);

        if (!detector.isReady()) {
            return;
        }

        VisionPortal.Builder portalBuilder = new VisionPortal.Builder();

        if (config.useBuiltinCamera) {
            portalBuilder.setCamera(BuiltinCameraDirection.BACK);
        } else {
            WebcamName webcam = null;
            try {
                webcam = hardwareMap.get(WebcamName.class, config.webcamName);
            } catch (Exception ignored) {
                List<WebcamName> webcams = hardwareMap.getAll(WebcamName.class);
                if (!webcams.isEmpty()) {
                    webcam = webcams.get(0);
                }
            }

            if (webcam != null) {
                portalBuilder.setCamera(webcam);
            } else {
                portalBuilder.setCamera(BuiltinCameraDirection.BACK);
            }
        }

        portalBuilder.setCameraResolution(new Size(frameWidth, frameHeight));
        portalBuilder.addProcessor(detector.createVisionProcessor());

        visionPortal = portalBuilder.build();
        initialized = true;
    }

    public boolean isInitialized() {
        return initialized && detector != null && detector.isReady();
    }

    public String getInitError() {
        return detector != null ? detector.getInitError() : "Detector not created";
    }

    public List<DetectionResult> getDetections() {
        List<DetectionResult> results = new ArrayList<>();

        if (!isInitialized()) {
            return results;
        }

        List<DetectionYOLO.Detection> detections = detector.getLastDetectionsSnapshot();

        for (DetectionYOLO.Detection d : detections) {
            double pixelWidth = d.width * frameWidth;
            double pixelHeight = d.height * frameHeight;
            double distance = calculateDistance(pixelWidth);

            results.add(new DetectionResult(
                    d.className,
                    d.confidence,
                    pixelWidth,
                    pixelHeight,
                    d.width,
                    d.height,
                    distance,
                    d.x,
                    d.y
            ));
        }

        return results;
    }

    public DetectionResult getBestDetection() {
        List<DetectionResult> detections = getDetections();
        if (detections.isEmpty()) {
            return null;
        }

        DetectionResult best = detections.get(0);
        for (DetectionResult d : detections) {
            if (d.confidence > best.confidence) {
                best = d;
            }
        }
        return best;
    }

    public DetectionResult getDetectionByTag(String tag) {
        List<DetectionResult> detections = getDetections();
        for (DetectionResult d : detections) {
            if (d.tag.equalsIgnoreCase(tag)) {
                return d;
            }
        }
        return null;
    }

    public List<DetectionResult> getDetectionsByTag(String tag) {
        List<DetectionResult> results = new ArrayList<>();
        List<DetectionResult> detections = getDetections();
        for (DetectionResult d : detections) {
            if (d.tag.equalsIgnoreCase(tag)) {
                results.add(d);
            }
        }
        return results;
    }

    public double calculateDistance(double pixelWidth) {
        if (pixelWidth <= 0) {
            return Double.POSITIVE_INFINITY;
        }
        return (Constants.KNOWN_OBJECT_WIDTH * Constants.CAMERA_FOCAL_LENGTH) / pixelWidth;
    }

    public double calculateFocalLength(double actualDistance, double pixelWidth) {
        if (pixelWidth <= 0 || actualDistance <= 0) {
            return 0;
        }
        return (actualDistance * pixelWidth) / Constants.KNOWN_OBJECT_WIDTH;
    }

    public String getDetectionInfo(DetectionResult result) {
        if (result == null) {
            return "No detection";
        }
        return String.format(Locale.US, "%s: %.1f%% | dist=%.2fm | px=%.0fx%.0f",
                result.tag,
                result.confidence * 100,
                result.estimatedDistance,
                result.pixelWidth,
                result.pixelHeight);
    }

    public int getFrameWidth() {
        return frameWidth;
    }

    public int getFrameHeight() {
        return frameHeight;
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    public void close() {
        if (detector != null) {
            detector.close();
            detector = null;
        }
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
        initialized = false;
    }
}
