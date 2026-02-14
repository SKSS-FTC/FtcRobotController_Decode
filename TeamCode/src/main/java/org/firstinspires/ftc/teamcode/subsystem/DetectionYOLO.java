package org.firstinspires.ftc.teamcode.subsystem;

import android.content.Context;
import android.content.res.AssetFileDescriptor;
import android.graphics.Bitmap;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.tensorflow.lite.Interpreter;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

/**
 * YOLO Object Detection Subsystem
 *
 * Uses TensorFlow Lite for real-time object detection with YOLOv8 TFLite models.
 * Supports YOLOv8n (nano) models embedded in assets.
 *
 * Lifecycle:
 * - init(): Load model from assets, prepare interpreter
 * - detect(): Run inference on bitmap from camera
 * - close(): Release TensorFlow Lite resources
 */
public class DetectionYOLO {
    private static final String TAG = "DetectionYOLO";
    private static final float DEFAULT_CONFIDENCE_THRESHOLD = 0.25f;
    private static final float IOU_THRESHOLD = 0.45f;

    private Interpreter interpreter;
    private TensorFlowFloatModel model;

    private int inputSize = 640;
    private float confidenceThreshold = DEFAULT_CONFIDENCE_THRESHOLD;

    private Datalogger datalogger;  // YOLO default input size
    private int numThreads = 4;
    private int modelInputWidth = 640;
    private int modelInputHeight = 640;
    private boolean inputIsNCHW = false;
    private String initError;
    private int[] inputTensorShape;
    private int[] outputTensorShape;
    private final Object inferenceLock = new Object();
    private volatile boolean closing = false;

    private Telemetry telemetry;
    private boolean showDebugInfo = false;
    
    // FPS tracking
    private long lastFrameTimeNanos = 0;
    private double fps = 0.0;
    private double fpsMovingAverage = 0.0;
    private int fpsSampleCount = 0;
    private double fpsSum = 0.0;
    private static final double FPS_MOVING_AVERAGE_WEIGHT = 0.1;
    private static final double INFERENCE_MOVING_AVERAGE_WEIGHT = 0.1;

    private double inferenceTimeMs = 0.0;
    private double inferenceTimeMovingAverageMs = 0.0;
    private double inferenceTimeSumMs = 0.0;
    private double inferenceFps = 0.0;
    private double inferenceFpsMovingAverage = 0.0;
    private double inferenceFpsSum = 0.0;
    private int inferenceSampleCount = 0;

    /**
     * Detection result
     */
    public static class Detection {
        public int classId;
        public String className;
        public float confidence;
        public float x, y;      // Center coordinates (normalized 0-1)
        public float width, height;  // Dimensions (normalized 0-1)

        public Detection(int classId, String className, float confidence,
                         float x, float y, float width, float height) {
            this.classId = classId;
            this.className = className;
            this.confidence = confidence;
            this.x = x;
            this.y = y;
            this.width = width;
            this.height = height;
        }

        @Override
        public String toString() {
            return String.format("%s (%.2f%%) [%.2f, %.2f, %.2f x %.2f]",
                    className, confidence * 100, x, y, width, height);
        }
    }

    /**
     * Builder for DetectionYOLO configuration
     */
    public static class Builder {
        private String modelPath = "yolov8n.tflite";
        private int inputSize = 640;
        private float confidenceThreshold = DEFAULT_CONFIDENCE_THRESHOLD;

    private Datalogger datalogger;
        private int numThreads = 4;
        private Telemetry telemetry;
        private boolean showDebugInfo = false;

        public Builder modelPath(String modelPath) {
            this.modelPath = modelPath;
            return this;
        }

        public Builder inputSize(int inputSize) {
            this.inputSize = inputSize;
            return this;
        }

        public Builder numThreads(int numThreads) {
            this.numThreads = numThreads;
            return this;
        }

        public Builder confidenceThreshold(float confidenceThreshold) {
            this.confidenceThreshold = confidenceThreshold;
            return this;
        }

        public Builder telemetry(Telemetry telemetry) {
            this.telemetry = telemetry;
            return this;
        }

        public Builder showDebugInfo(boolean showDebugInfo) {
            this.showDebugInfo = showDebugInfo;
            return this;
        }

        public DetectionYOLO build() {
            return new DetectionYOLO(this);
        }
    }

    private DetectionYOLO(Builder builder) {
        this.inputSize = builder.inputSize;
        this.confidenceThreshold = builder.confidenceThreshold;
        this.numThreads = builder.numThreads;
        this.telemetry = builder.telemetry;
        this.showDebugInfo = builder.showDebugInfo;
        this.modelPath = builder.modelPath;
    }

    // Internal helper class for scikit-learn model interface
    private static class TensorFlowFloatModel {
        private MappedByteBuffer modelBuffer;

        TensorFlowFloatModel(MappedByteBuffer modelBuffer) {
            this.modelBuffer = modelBuffer;
        }

        boolean exists() {
            return modelBuffer != null;
        }
    }

    private String modelPath;

    /**
     * Initialize YOLO detection subsystem
     */
    public void init(Context context) {
        initError = null;
        closing = false;
        try {
            if (!ensureTensorFlowLiteNativeLoaded(context)) {
                interpreter = null;
                model = null;
                if (telemetry != null) {
                    telemetry.addData("ERROR", initError);
                    telemetry.update();
                }
                return;
            }

            // Load model from assets
            MappedByteBuffer tfliteModel = loadModelBuffer(context);
            model = new TensorFlowFloatModel(tfliteModel);

            // Create TensorFlow Lite interpreter
            Interpreter.Options options = new Interpreter.Options();
            options.setNumThreads(numThreads);

            interpreter = new Interpreter(tfliteModel, options);

            // Verify model input shape and detect layout (NHWC/NCHW)
            int[] inputShape = interpreter.getInputTensor(0).shape();
            inputTensorShape = inputShape;
            if (inputShape.length != 4) {
                if (telemetry != null) {
                    telemetry.addLine("ERROR: Unsupported model input rank");
                    telemetry.update();
                }
                initError = "Unsupported input rank: " + inputShape.length;
                interpreter.close();
                interpreter = null;
                return;
            }

            if (inputShape[3] == 3) {
                inputIsNCHW = false;
                modelInputHeight = inputShape[1];
                modelInputWidth = inputShape[2];
            } else if (inputShape[1] == 3) {
                inputIsNCHW = true;
                modelInputHeight = inputShape[2];
                modelInputWidth = inputShape[3];
            } else {
                if (telemetry != null) {
                    telemetry.addLine("ERROR: Unsupported input tensor layout");
                    telemetry.update();
                }
                initError = "Unsupported input layout: " + java.util.Arrays.toString(inputShape);
                interpreter.close();
                interpreter = null;
                return;
            }

            if (modelInputWidth <= 0 || modelInputHeight <= 0) {
                initError = "Invalid model input shape: " + java.util.Arrays.toString(inputShape);
                interpreter.close();
                interpreter = null;
                return;
            }

            outputTensorShape = interpreter.getOutputTensor(0).shape();

            inputSize = Math.max(modelInputWidth, modelInputHeight);

            if (showDebugInfo && telemetry != null) {
                telemetry.addData("DetectionYOLO", "Model loaded successfully");
                telemetry.addData("Input shape", java.util.Arrays.toString(inputShape));
                telemetry.addData("Output shape", java.util.Arrays.toString(outputTensorShape));
                telemetry.addData("Layout", inputIsNCHW ? "NCHW" : "NHWC");
                telemetry.update();
            }

            // Model initialized

        } catch (IOException e) {
            // Error logged above via telemetry
            initError = "Failed to load model: " + e.getMessage();
            if (telemetry != null) {
                telemetry.addData("ERROR", "Failed to load model: " + modelPath);
                telemetry.addData("Details", e.getMessage());
                telemetry.update();
            }
        } catch (Throwable t) {
            interpreter = null;
            model = null;
            initError = "YOLO init failed: " + t.getClass().getSimpleName() + " - " + t.getMessage();
            if (telemetry != null) {
                telemetry.addData("ERROR", "YOLO init failed");
                telemetry.addData("Details", t.getClass().getSimpleName());
                telemetry.update();
            }
        }
    }

    /**
     * Detect objects in a bitmap image
     */
    public List<Detection> detect(Bitmap bitmap) {
        if (closing) {
            return new ArrayList<>();
        }

        // FPS calculation
        long currentTimeNanos = System.nanoTime();
        if (lastFrameTimeNanos > 0) {
            long elapsedTimeNanos = currentTimeNanos - lastFrameTimeNanos;
            double currentFps = 1e9 / elapsedTimeNanos;
            
            fps = currentFps;
            
            // Update moving average
            if (fpsSampleCount == 0) {
                fpsMovingAverage = currentFps;
            } else {
                fpsMovingAverage = (FPS_MOVING_AVERAGE_WEIGHT * currentFps + 
                              (1.0 - FPS_MOVING_AVERAGE_WEIGHT) * fpsMovingAverage);
            }
            
            fpsSum += currentFps;
            fpsSampleCount++;
        }
        lastFrameTimeNanos = currentTimeNanos;

        if (datalogger != null) {
            datalogger.startFrame();
        }

        if (interpreter == null || model == null || !model.exists()) {
            /* Removed DbgLog */
            if (telemetry != null) {
                telemetry.addLine("WARN: DetectionYOLO not initialized");
            }
            return new ArrayList<>();
        }

        // Resize and preprocess bitmap to model input size
        Bitmap inputBitmap = Bitmap.createScaledBitmap(bitmap, modelInputWidth, modelInputHeight, false);

        // Create input buffer for model layout with normalized RGB values (0-255 -> 0-1)
        ByteBuffer inputBuffer = ByteBuffer.allocateDirect(modelInputWidth * modelInputHeight * 3 * 4)
                .order(ByteOrder.nativeOrder());

        int[] pixels = new int[modelInputWidth * modelInputHeight];
        inputBitmap.getPixels(pixels, 0, modelInputWidth, 0, 0, modelInputWidth, modelInputHeight);

        if (inputIsNCHW) {
            for (int i = 0; i < pixels.length; i++) {
                int pixel = pixels[i];
                inputBuffer.putFloat(((pixel >> 16) & 0xFF) / 255.0f);
            }
            for (int i = 0; i < pixels.length; i++) {
                int pixel = pixels[i];
                inputBuffer.putFloat(((pixel >> 8) & 0xFF) / 255.0f);
            }
            for (int i = 0; i < pixels.length; i++) {
                int pixel = pixels[i];
                inputBuffer.putFloat((pixel & 0xFF) / 255.0f);
            }
        } else {
            for (int i = 0; i < pixels.length; i++) {
                int pixel = pixels[i];
                inputBuffer.putFloat(((pixel >> 16) & 0xFF) / 255.0f);
                inputBuffer.putFloat(((pixel >> 8) & 0xFF) / 255.0f);
                inputBuffer.putFloat((pixel & 0xFF) / 255.0f);
            }
        }
        inputBuffer.rewind();

        // Prepare output tensor: [1, 84, 8400] for YOLOv8n (COCO 80 classes + 4 box coords)
        int numDetections;
        int numOutputClasses;
        float[][][] output3d;
        if (outputTensorShape != null && outputTensorShape.length == 3) {
            output3d = new float[outputTensorShape[0]][outputTensorShape[1]][outputTensorShape[2]];
            if (outputTensorShape[1] == 84) {
                numOutputClasses = outputTensorShape[1];
                numDetections = outputTensorShape[2];
            } else if (outputTensorShape[2] == 84) {
                numOutputClasses = outputTensorShape[2];
                numDetections = outputTensorShape[1];
            } else {
                numOutputClasses = 84;
                numDetections = outputTensorShape[1];
            }
        } else {
            numDetections = 8400;
            numOutputClasses = 84;
            output3d = new float[1][numOutputClasses][numDetections];
        }

        // Run inference
        try {
            long inferenceStartNanos = System.nanoTime();
            synchronized (inferenceLock) {
                if (closing || interpreter == null) {
                    inputBitmap.recycle();
                    return new ArrayList<>();
                }
                interpreter.run(inputBuffer, output3d);
            }
            updateInferenceStats(System.nanoTime() - inferenceStartNanos);
        } catch (RuntimeException e) {
            if (telemetry != null) {
                telemetry.addData("ERROR", "YOLO inference failed");
                telemetry.addData("Details", e.getMessage());
                telemetry.update();
            }
            inputBitmap.recycle();
            return new ArrayList<>();
        }

        // Process output: extract boxes, scores, classes
        List<Detection> detections = new ArrayList<>();

        for (int i = 0; i < numDetections; i++) {
            float cx;
            float cy;
            float w;
            float h;
            boolean classesSecond = outputTensorShape != null && outputTensorShape.length == 3 && outputTensorShape[1] == 84;
            if (classesSecond) {
                cx = output3d[0][0][i];
                cy = output3d[0][1][i];
                w = output3d[0][2][i];
                h = output3d[0][3][i];
            } else {
                cx = output3d[0][i][0];
                cy = output3d[0][i][1];
                w = output3d[0][i][2];
                h = output3d[0][i][3];
            }

            // Find max confidence and class
            int maxClassId = 0;
            float maxConfidence = 0;
            for (int c = 4; c < numOutputClasses; c++) {
                float score = classesSecond ? output3d[0][c][i] : output3d[0][i][c];
                if (score > maxConfidence) {
                    maxConfidence = score;
                    maxClassId = c - 4;
                }
            }

            // Filter by confidence threshold
            if (maxConfidence > confidenceThreshold) {
                // Convert center coords to corner coords (x1, y1, x2, y2)
                float x1 = cx - w / 2;
                float y1 = cy - h / 2;
                float x2 = cx + w / 2;
                float y2 = cy + h / 2;

                // Clip to [0, 1]
                x1 = Math.max(0, Math.min(1, x1));
                y1 = Math.max(0, Math.min(1, y1));
                x2 = Math.max(0, Math.min(1, x2));
                y2 = Math.max(0, Math.min(1, y2));

                detections.add(new Detection(
                        maxClassId,
                        getCOCOClassName(maxClassId),
                        maxConfidence,
                        x1, y1, x2 - x1, y2 - y1
                ));
            }
        }

        // Apply Non-Maximum Suppression to remove overlapping boxes
        detections = applyNMS(detections, IOU_THRESHOLD);

        if (showDebugInfo && telemetry != null) {
            telemetry.addData("Detections", detections.size());
            for (Detection d : detections) {
                telemetry.addData("  " + d.className, String.format("%.1f%%", d.confidence * 100));
            }
            telemetry.update();
        }

        inputBitmap.recycle();

        return detections;
    }

    /**
     * Apply Non-Maximum Suppression to remove duplicate detections
     */
    private List<Detection> applyNMS(List<Detection> detections, float iouThreshold) {
        if (detections.isEmpty()) {
            return detections;
        }

        // Sort by confidence (descending)
        detections.sort((a, b) -> Float.compare(b.confidence, a.confidence));

        List<Detection> nmsDetections = new ArrayList<>();
        boolean[] suppressed = new boolean[detections.size()];

        for (int i = 0; i < detections.size(); i++) {
            if (suppressed[i]) {
                continue;
            }

            nmsDetections.add(detections.get(i));

            for (int j = i + 1; j < detections.size(); j++) {
                if (suppressed[j]) {
                    continue;
                }

                if (calculateIOU(detections.get(i), detections.get(j)) > iouThreshold) {
                    suppressed[j] = true;
                }
            }
        }

        return nmsDetections;
    }

    /**
     * Calculate Intersection over Union (IoU) between two detections
     */
    private float calculateIOU(Detection a, Detection b) {
        float interX1 = Math.max(a.x, b.x);
        float interY1 = Math.max(a.y, b.y);
        float interX2 = Math.min(a.x + a.width, b.x + b.width);
        float interY2 = Math.min(a.y + a.height, b.y + b.height);

        float interArea = Math.max(0, interX2 - interX1) * Math.max(0, interY2 - interY1);

        float aArea = a.width * a.height;
        float bArea = b.width * b.height;

        return interArea / (aArea + bArea - interArea);
    }

    private static final String[] COCO_CLASSES = {
            "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
            "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
            "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
            "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
            "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
            "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
            "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
            "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
            "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book",
            "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
    };
    
    /**
     * Get COCO class name by class ID
     */
    public void setDatalogger(Datalogger datalogger) {
        this.datalogger = datalogger;
    }

    /**
     * Get current FPS (instantaneous)
     */
    public double getFps() {
        return fps;
    }

    /**
     * Get moving average FPS
     */
    public double getFpsMovingAverage() {
        return fpsMovingAverage;
    }

    /**
     * Get average FPS since last reset
     */
    public double getFpsAverage() {
        return fpsSampleCount > 0 ? fpsSum / fpsSampleCount : 0.0;
    }

    /**
     * Get number of FPS samples collected
     */
    public int getFpsSampleCount() {
        return fpsSampleCount;
    }

    public double getInferenceTimeMs() {
        return inferenceTimeMs;
    }

    public double getInferenceTimeMovingAverageMs() {
        return inferenceTimeMovingAverageMs;
    }

    public double getInferenceTimeAverageMs() {
        return inferenceSampleCount > 0 ? inferenceTimeSumMs / inferenceSampleCount : 0.0;
    }

    public double getInferenceFps() {
        return inferenceFps;
    }

    public double getInferenceFpsMovingAverage() {
        return inferenceFpsMovingAverage;
    }

    public double getInferenceFpsAverage() {
        return inferenceSampleCount > 0 ? inferenceFpsSum / inferenceSampleCount : 0.0;
    }

    public int getInferenceSampleCount() {
        return inferenceSampleCount;
    }

    public void resetBenchmark() {
        resetFps();
    }

    /**
     * Reset FPS tracking
     */
    public void resetFps() {
        lastFrameTimeNanos = 0;
        fps = 0.0;
        fpsMovingAverage = 0.0;
        fpsSampleCount = 0;
        fpsSum = 0.0;
        inferenceTimeMs = 0.0;
        inferenceTimeMovingAverageMs = 0.0;
        inferenceTimeSumMs = 0.0;
        inferenceFps = 0.0;
        inferenceFpsMovingAverage = 0.0;
        inferenceFpsSum = 0.0;
        inferenceSampleCount = 0;
    }

    private void updateInferenceStats(long inferenceElapsedNanos) {
        if (inferenceElapsedNanos <= 0) {
            return;
        }

        double currentInferenceTimeMs = inferenceElapsedNanos / 1_000_000.0;
        double currentInferenceFps = 1_000_000_000.0 / inferenceElapsedNanos;

        inferenceTimeMs = currentInferenceTimeMs;
        inferenceFps = currentInferenceFps;

        if (inferenceSampleCount == 0) {
            inferenceTimeMovingAverageMs = currentInferenceTimeMs;
            inferenceFpsMovingAverage = currentInferenceFps;
        } else {
            inferenceTimeMovingAverageMs = INFERENCE_MOVING_AVERAGE_WEIGHT * currentInferenceTimeMs
                    + (1.0 - INFERENCE_MOVING_AVERAGE_WEIGHT) * inferenceTimeMovingAverageMs;
            inferenceFpsMovingAverage = INFERENCE_MOVING_AVERAGE_WEIGHT * currentInferenceFps
                    + (1.0 - INFERENCE_MOVING_AVERAGE_WEIGHT) * inferenceFpsMovingAverage;
        }

        inferenceTimeSumMs += currentInferenceTimeMs;
        inferenceFpsSum += currentInferenceFps;
        inferenceSampleCount++;
    }

    private String getCOCOClassName(int classId) {
        if (classId >= 0 && classId < COCO_CLASSES.length) {
            return COCO_CLASSES[classId];
        }
        return "class_" + classId;
    }

    /**
     * Close and release TensorFlow Lite resources
     */
    public void close() {
        synchronized (inferenceLock) {
            closing = true;
            if (interpreter != null) {
                interpreter.close();
                interpreter = null;
                if (showDebugInfo && telemetry != null) {
                    telemetry.addData("DetectionYOLO", "Closed");
                }
            }
            model = null;
        }
    }

    /**
     * Load model file from InputStream to temporary file
     * TensorFlow Lite requires a file path for MappedByteBuffer
     */
    private MappedByteBuffer loadModelBuffer(Context context) throws IOException {
        try (AssetFileDescriptor afd = context.getAssets().openFd(modelPath);
             FileInputStream fileInputStream = new FileInputStream(afd.getFileDescriptor());
             FileChannel fileChannel = fileInputStream.getChannel()) {
            return fileChannel.map(
                    FileChannel.MapMode.READ_ONLY,
                    afd.getStartOffset(),
                    afd.getDeclaredLength());
        } catch (IOException ignored) {
            java.io.File cacheFile = new java.io.File(context.getCacheDir(), modelPath);

            if (!cacheFile.exists() || cacheFile.length() == 0) {
                try (java.io.InputStream is = context.getAssets().open(modelPath);
                     FileOutputStream fos = new FileOutputStream(cacheFile)) {
                    byte[] buffer = new byte[8192];
                    int bytesRead;
                    while ((bytesRead = is.read(buffer)) != -1) {
                        fos.write(buffer, 0, bytesRead);
                    }
                    fos.flush();
                }
            }

            try (FileInputStream fis = new FileInputStream(cacheFile);
                 FileChannel fileChannel = fis.getChannel()) {
                return fileChannel.map(FileChannel.MapMode.READ_ONLY, 0, fileChannel.size());
            }
        }
    }

    /**
     * Check if detector is initialized and ready
     */
    public boolean isReady() {
        return interpreter != null && model != null && model.exists();
    }

    public String getInitError() {
        return initError;
    }

    /**
     * Get input image size
     */
    public int getInputSize() {
        return inputSize;
    }

    public String getModelPath() {
        return modelPath;
    }

    public String getInputLayout() {
        return inputIsNCHW ? "NCHW" : "NHWC";
    }

    public int[] getInputTensorShape() {
        return inputTensorShape;
    }

    public int[] getOutputTensorShape() {
        return outputTensorShape;
    }

    private boolean ensureTensorFlowLiteNativeLoaded(Context context) {
        String[] candidateNames = {
                "tensorflowlite_jni",
                "tensorflowlite_jni_stable",
                "tensorflowlite_jni_gms_client"
        };

        StringBuilder loadErrors = new StringBuilder();
        for (String name : candidateNames) {
            try {
                System.loadLibrary(name);
                return true;
            } catch (UnsatisfiedLinkError e) {
                if (loadErrors.length() > 0) {
                    loadErrors.append(" | ");
                }
                loadErrors.append(name).append(": ").append(e.getMessage());
            }
        }

        java.io.File directLib = new java.io.File(
                context.getApplicationInfo().nativeLibraryDir,
                "libtensorflowlite_jni.so");
        if (directLib.exists()) {
            try {
                System.load(directLib.getAbsolutePath());
                return true;
            } catch (UnsatisfiedLinkError e) {
                if (loadErrors.length() > 0) {
                    loadErrors.append(" | ");
                }
                loadErrors.append("direct:").append(e.getMessage());
            }
        }

        initError = "TensorFlow Lite native load failed: " + loadErrors;
        return false;
    }
}
