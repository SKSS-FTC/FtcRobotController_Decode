package org.firstinspires.ftc.teamcode.subsystem;

import android.content.Context;
import android.content.res.AssetFileDescriptor;
import android.graphics.Bitmap;
import android.graphics.RectF;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.tensorflow.lite.Interpreter;

import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;


/**
 * YOLOv8n Segmentation Subsystem for Object Segmentation
 *
 * @author Your Name
 * @version 1.0
 */
public class SegmentationYOLO {
    private static final String TAG = "SegmentationYOLO";

    // Model configuration
    private static final int MODEL_INPUT_WIDTH = 640;
    private static final int MODEL_INPUT_HEIGHT = 640;
    private static final int NUM_CLASSES = 80;
    private static final float CONFIDENCE_THRESHOLD = 0.5f;
    private static final float IOU_THRESHOLD = 0.45f;

    // FPS tracking
    private static final double FPS_MOVING_AVERAGE_WEIGHT = 0.2;
    private static final double INFERENCE_MOVING_AVERAGE_WEIGHT = 0.2;
    private long lastFrameTimeNanos = 0;
    private double fps = 0;
    private double fpsMovingAverage = 0;
    private double fpsSum = 0;
    private int fpsSampleCount = 0;
    private double inferenceTimeMs = 0;
    private double inferenceTimeMovingAverageMs = 0;
    private double inferenceTimeSumMs = 0;
    private double inferenceFps = 0;
    private double inferenceFpsMovingAverage = 0;
    private double inferenceFpsSum = 0;
    private int inferenceSampleCount = 0;

    // COCO class names
    private static final String[] COCO_CLASSES = {
        "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
        "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
        "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
        "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
        "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
        "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
        "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
        "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
        "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
        "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
        "toothbrush"
    };

    private Interpreter interpreter;
    private boolean initialized = false;
    private String initError = null;
    private String modelPath;
    private float confidenceThreshold;
    private float iouThreshold;
    private int[] detectionOutputShape;
    private int[] maskOutputShape;
    private int modelInputWidth = MODEL_INPUT_WIDTH;
    private int modelInputHeight = MODEL_INPUT_HEIGHT;
    private boolean inputIsNCHW = false;
    private final Object inferenceLock = new Object();
    private volatile boolean closing = false;
    private final ElapsedTime runtime = new ElapsedTime();

    /**
     * Segmented detected object with mask
     */
    public static class Segmentation {
        public final int classId;
        public final String className;
        public final float confidence;
        public final RectF bbox;  // [x, y, width, height] normalized (0-1)
        public final float[] mask;  // segmentation mask values
        public final int maskWidth;
        public final int maskHeight;

        public Segmentation(int classId, String className, float confidence, float x, float y,
                           float width, float height, float[] mask, int maskWidth, int maskHeight) {
            this.classId = classId;
            this.className = className;
            this.confidence = confidence;
            this.bbox = new RectF(x, y, x + width, y + height);
            this.mask = mask;
            this.maskWidth = maskWidth;
            this.maskHeight = maskHeight;
        }

        public float getX() {
            return bbox.left;
        }

        public float getY() {
            return bbox.top;
        }

        public float getWidth() {
            return bbox.width();
        }

        public float getHeight() {
            return bbox.height();
        }

        @Override
        public String toString() {
            return String.format("%s (%.2f%%) bbox=[%.2f,%.2f,%.2f,%.2f] mask=%dx%d",
                className, confidence * 100, getX(), getY(), getWidth(), getHeight(),
                maskWidth, maskHeight);
        }
    }

    /**
     * Builder for SegmentationYOLO
     */
    public static class Builder {
        private Context context;
        private String modelPath = "yolov8n-seg.tflite";
        private int numThreads = 4;
        private float confidenceThreshold = CONFIDENCE_THRESHOLD;
        private float iouThreshold = IOU_THRESHOLD;

        public Builder(Context context) {
            this.context = context;
        }

        public Builder setModelPath(String modelPath) {
            this.modelPath = modelPath;
            return this;
        }

        public Builder setNumThreads(int numThreads) {
            this.numThreads = numThreads;
            return this;
        }

        public Builder setConfidenceThreshold(float threshold) {
            this.confidenceThreshold = threshold;
            return this;
        }

        public Builder setIOUThreshold(float threshold) {
            this.iouThreshold = threshold;
            return this;
        }

        public SegmentationYOLO build() {
            return new SegmentationYOLO(context, modelPath, numThreads, confidenceThreshold, iouThreshold);
        }
    }

    /**
     * Constructor
     */
    private SegmentationYOLO(Context context, String modelPath, int numThreads,
                            float confidenceThreshold, float iouThreshold) {
        try {
            if (!ensureTensorFlowLiteNativeLoaded(context)) {
                initialized = false;
                interpreter = null;
                return;
            }

            // Load model
            MappedByteBuffer modelBuffer = loadModelFile(context, modelPath);
            if (modelBuffer == null) {
                initError = "Failed to load model: " + modelPath;
                return;
            }

            // Create interpreter options
            Interpreter.Options options = new Interpreter.Options();
            options.setNumThreads(numThreads);

            // Create interpreter
            interpreter = new Interpreter(modelBuffer, options);
            this.modelPath = modelPath;
            this.confidenceThreshold = confidenceThreshold;
            this.iouThreshold = iouThreshold;
            int[] inputShape = interpreter.getInputTensor(0).shape();
            if (inputShape.length == 4) {
                if (inputShape[3] == 3) {
                    inputIsNCHW = false;
                    modelInputHeight = inputShape[1];
                    modelInputWidth = inputShape[2];
                } else if (inputShape[1] == 3) {
                    inputIsNCHW = true;
                    modelInputHeight = inputShape[2];
                    modelInputWidth = inputShape[3];
                }
            }
            this.detectionOutputShape = interpreter.getOutputTensor(0).shape();
            if (interpreter.getOutputTensorCount() > 1) {
                this.maskOutputShape = interpreter.getOutputTensor(1).shape();
            }

            initialized = true;
            runtime.reset();
        } catch (Exception e) {
            initError = "Failed to initialize SegmentationYOLO: " + e.getMessage();
            initialized = false;
            interpreter = null;
        }
    }

    /**
     * Initialize subsystem
     */
    public void init() {
        // Initialization done in constructor
    }

    /**
     * Close subsystem and release resources
     */
    public void close() {
        synchronized (inferenceLock) {
            closing = true;
            if (interpreter != null) {
                interpreter.close();
                interpreter = null;
            }
            initialized = false;
        }
    }

    /**
     * Load model file from assets
     */
    private MappedByteBuffer loadModelFile(Context context, String modelPath) throws IOException {
        try (AssetFileDescriptor afd = context.getAssets().openFd(modelPath);
             FileInputStream fileInputStream = new FileInputStream(afd.getFileDescriptor());
             FileChannel fileChannel = fileInputStream.getChannel()) {
            return fileChannel.map(
                FileChannel.MapMode.READ_ONLY,
                afd.getStartOffset(),
                afd.getDeclaredLength()
            );
        }
    }

    /**
     * Detect objects with segmentation masks
     *
     * @param bitmap Input image (will be resized to 640x640)
     * @return List of detected segmentations
     */
    public List<Segmentation> detect(Bitmap bitmap) {
        // FPS calculation
        if (!initialized || interpreter == null || closing) {
            return new ArrayList<>();
        }

        long currentTimeNanos = System.nanoTime();
        if (lastFrameTimeNanos > 0) {
            double elapsedTimeNanos = currentTimeNanos - lastFrameTimeNanos;
            fps = 1e9 / elapsedTimeNanos;
            fpsMovingAverage = FPS_MOVING_AVERAGE_WEIGHT * fps + (1.0 - FPS_MOVING_AVERAGE_WEIGHT) * fpsMovingAverage;
            fpsSum += fps;
            fpsSampleCount++;
        }
        lastFrameTimeNanos = currentTimeNanos;

        // Preprocess bitmap
        Bitmap processedBitmap = Bitmap.createScaledBitmap(bitmap, modelInputWidth, modelInputHeight, true);
        ByteBuffer inputBuffer = preprocessImage(processedBitmap);

        // Prepare output buffers
        // YOLOv8-seg outputs: [1, 84, 8400] for detection + [1, 160, 8400] for masks
        // 84 = 4 (bbox) + 80 (classes)
        // 160 = 160 mask coefficients (mask resolution 160x160 but flattened)
        int numDetections = 8400;
        int featureCount = 84;
        boolean channelsFirst = true;
        if (detectionOutputShape != null && detectionOutputShape.length == 3) {
            int d1 = detectionOutputShape[1];
            int d2 = detectionOutputShape[2];
            if (d1 >= 84 && d2 > 100) {
                featureCount = d1;
                numDetections = d2;
                channelsFirst = true;
            } else if (d2 >= 84 && d1 > 100) {
                featureCount = d2;
                numDetections = d1;
                channelsFirst = false;
            }
        }

        float[][][] detectionOutput;
        if (channelsFirst) {
            detectionOutput = new float[1][featureCount][numDetections];
        } else {
            detectionOutput = new float[1][numDetections][featureCount];
        }

        int maskCoeffCount = Math.max(0, featureCount - 4 - NUM_CLASSES);
        Object maskOutput = null;
        if (interpreter.getOutputTensorCount() > 1) {
            if (maskOutputShape != null && maskOutputShape.length == 3) {
                maskOutput = new float[maskOutputShape[0]][maskOutputShape[1]][maskOutputShape[2]];
            } else if (maskOutputShape != null && maskOutputShape.length == 4) {
                maskOutput = new float[maskOutputShape[0]][maskOutputShape[1]][maskOutputShape[2]][maskOutputShape[3]];
            } else {
                maskOutput = new float[1][Math.max(maskCoeffCount, 1)][numDetections];
            }
        }

        // Run inference
        try {
            Map<Integer, Object> outputs = new HashMap<>();
            outputs.put(0, detectionOutput);
            if (maskOutput != null) {
                outputs.put(1, maskOutput);
            }
            long inferenceStartNanos = System.nanoTime();
            synchronized (inferenceLock) {
                if (closing || interpreter == null) {
                    processedBitmap.recycle();
                    return new ArrayList<>();
                }
                interpreter.runForMultipleInputsOutputs(new Object[]{inputBuffer}, outputs);
            }
            updateInferenceStats(System.nanoTime() - inferenceStartNanos);
        } catch (Throwable e) {
            processedBitmap.recycle();
            return new ArrayList<>();
        }

        ProtoLayout protoLayout = resolveProtoLayout(maskOutput);

        // Process outputs
        List<Segmentation> segmentations = new ArrayList<>();
        for (int i = 0; i < numDetections; i++) {
            // Extract bbox (cx, cy, w, h)
            float cx = channelsFirst ? detectionOutput[0][0][i] : detectionOutput[0][i][0];
            float cy = channelsFirst ? detectionOutput[0][1][i] : detectionOutput[0][i][1];
            float w = channelsFirst ? detectionOutput[0][2][i] : detectionOutput[0][i][2];
            float h = channelsFirst ? detectionOutput[0][3][i] : detectionOutput[0][i][3];

            if (!Float.isFinite(cx) || !Float.isFinite(cy) || !Float.isFinite(w) || !Float.isFinite(h) || w <= 0 || h <= 0) {
                continue;
            }

            if (cx > 1f || cy > 1f || w > 1f || h > 1f) {
                cx /= modelInputWidth;
                cy /= modelInputHeight;
                w /= modelInputWidth;
                h /= modelInputHeight;
            }

            // Find max class confidence
            float maxConfidence = 0;
            int classId = -1;
            for (int c = 0; c < NUM_CLASSES; c++) {
                float conf = channelsFirst ? detectionOutput[0][4 + c][i] : detectionOutput[0][i][4 + c];
                if (conf > maxConfidence) {
                    maxConfidence = conf;
                    classId = c;
                }
            }

            // Filter by confidence
            if (maxConfidence >= confidenceThreshold && classId >= 0) {
                // Extract mask coefficients
                float[] maskCoefficients = new float[Math.max(maskCoeffCount, 1)];
                for (int m = 0; m < maskCoeffCount; m++) {
                    int coeffIndex = 4 + NUM_CLASSES + m;
                    maskCoefficients[m] = channelsFirst
                            ? detectionOutput[0][coeffIndex][i]
                            : detectionOutput[0][i][coeffIndex];
                }

                // Convert bbox to [x, y, width, height] format
                float x = Math.max(0f, cx - w / 2f);
                float y = Math.max(0f, cy - h / 2f);
                float clippedW = Math.min(1f - x, w);
                float clippedH = Math.min(1f - y, h);

                if (clippedW <= 0 || clippedH <= 0) {
                    continue;
                }

                float[] decodedMask = decodeMask(maskCoefficients, protoLayout, x, y, clippedW, clippedH);

                Segmentation seg = new Segmentation(
                    classId,
                    getCOCOClassName(classId),
                    maxConfidence,
                    x, y, clippedW, clippedH,
                    decodedMask,
                    protoLayout.width,
                    protoLayout.height
                );
                segmentations.add(seg);
            }
        }

        processedBitmap.recycle();

        // Apply NMS
        return applyNMS(segmentations, iouThreshold);
    }

    private static class ProtoLayout {
        final float[][][] proto;
        final int channels;
        final int width;
        final int height;
        final boolean channelFirst;

        ProtoLayout(float[][][] proto, int channels, int width, int height, boolean channelFirst) {
            this.proto = proto;
            this.channels = channels;
            this.width = width;
            this.height = height;
            this.channelFirst = channelFirst;
        }
    }

    private ProtoLayout resolveProtoLayout(Object maskOutputObject) {
        if (maskOutputObject == null) {
            return new ProtoLayout(new float[1][1][1], 1, 1, 1, true);
        }

        if (maskOutputObject instanceof float[][][][]) {
            float[][][][] maskOutput = (float[][][][]) maskOutputObject;
            int dim1 = maskOutput[0].length;
            int dim2 = maskOutput[0][0].length;
            int dim3 = maskOutput[0][0][0].length;

            if (dim1 <= 64) {
                float[][][] proto = new float[dim1][dim2][dim3];
                for (int ch = 0; ch < dim1; ch++) {
                    for (int y = 0; y < dim2; y++) {
                        System.arraycopy(maskOutput[0][ch][y], 0, proto[ch][y], 0, dim3);
                    }
                }
                return new ProtoLayout(proto, dim1, dim3, dim2, true);
            }

            float[][][] proto = new float[dim3][dim1][dim2];
            for (int y = 0; y < dim1; y++) {
                for (int x = 0; x < dim2; x++) {
                    for (int ch = 0; ch < dim3; ch++) {
                        proto[ch][y][x] = maskOutput[0][y][x][ch];
                    }
                }
            }
            return new ProtoLayout(proto, dim3, dim2, dim1, false);
        }

        float[][][] maskOutput = (float[][][]) maskOutputObject;
        if (maskOutput == null) {
            return new ProtoLayout(new float[1][1][1], 1, 1, 1, true);
        }

        int channels = maskOutput.length > 0 ? maskOutput[0].length : 1;
        int detections = (maskOutput.length > 0 && maskOutput[0].length > 0) ? maskOutput[0][0].length : 1;
        float[][][] proto = new float[channels][1][detections];
        for (int ch = 0; ch < channels; ch++) {
            System.arraycopy(maskOutput[0][ch], 0, proto[ch][0], 0, detections);
        }
        return new ProtoLayout(proto, channels, detections, 1, true);
    }

    private float[] decodeMask(float[] coeffs, ProtoLayout layout, float bx, float by, float bw, float bh) {
        int channelsToUse = Math.min(coeffs.length, layout.channels);
        float[] mask = new float[layout.width * layout.height];

        int xMin = Math.max(0, Math.min(layout.width - 1, (int) (bx * layout.width)));
        int yMin = Math.max(0, Math.min(layout.height - 1, (int) (by * layout.height)));
        int xMax = Math.max(xMin + 1, Math.min(layout.width, (int) ((bx + bw) * layout.width)));
        int yMax = Math.max(yMin + 1, Math.min(layout.height, (int) ((by + bh) * layout.height)));

        for (int y = yMin; y < yMax; y++) {
            for (int x = xMin; x < xMax; x++) {
                float sum = 0f;
                for (int ch = 0; ch < channelsToUse; ch++) {
                    sum += coeffs[ch] * layout.proto[ch][y][x];
                }
                float probability = 1f / (1f + (float) Math.exp(-sum));
                mask[y * layout.width + x] = probability;
            }
        }

        return mask;
    }

    /**
     * Preprocess image for TFLite model
     */
    private ByteBuffer preprocessImage(Bitmap bitmap) {
        int[] pixels = new int[modelInputWidth * modelInputHeight];
        bitmap.getPixels(pixels, 0, modelInputWidth, 0, 0, modelInputWidth, modelInputHeight);

        ByteBuffer buffer = ByteBuffer.allocateDirect(4 * modelInputWidth * modelInputHeight * 3);
        buffer.order(ByteOrder.nativeOrder());

        if (inputIsNCHW) {
            for (int i = 0; i < pixels.length; i++) {
                buffer.putFloat(((pixels[i] >> 16) & 0xFF) / 255.0f);
            }
            for (int i = 0; i < pixels.length; i++) {
                buffer.putFloat(((pixels[i] >> 8) & 0xFF) / 255.0f);
            }
            for (int i = 0; i < pixels.length; i++) {
                buffer.putFloat((pixels[i] & 0xFF) / 255.0f);
            }
        } else {
            for (int i = 0; i < pixels.length; i++) {
                int pixel = pixels[i];
                float r = ((pixel >> 16) & 0xFF) / 255.0f;
                float g = ((pixel >> 8) & 0xFF) / 255.0f;
                float b = (pixel & 0xFF) / 255.0f;
                buffer.putFloat(r);
                buffer.putFloat(g);
                buffer.putFloat(b);
            }
        }
        buffer.rewind();
        return buffer;
    }

    /**
     * Apply Non-Maximum Suppression to remove overlapping detections
     */
    private List<Segmentation> applyNMS(List<Segmentation> detections, float iouThreshold) {
        List<Segmentation> result = new ArrayList<>();
        List<Segmentation> sorted = new ArrayList<>(detections);
        sorted.sort((a, b) -> Float.compare(b.confidence, a.confidence));

        while (!sorted.isEmpty()) {
            Segmentation best = sorted.remove(0);
            result.add(best);

            // Remove overlapping detections
            sorted.removeIf(detection -> calculateIOU(best.bbox, detection.bbox) > iouThreshold);
        }
        return result;
    }

    /**
     * Calculate Intersection over Union (IoU) for two bounding boxes
     */
    private float calculateIOU(RectF box1, RectF box2) {
        float xLeft = Math.max(box1.left, box2.left);
        float yTop = Math.max(box1.top, box2.top);
        float xRight = Math.min(box1.right, box2.right);
        float yBottom = Math.min(box1.bottom, box2.bottom);

        if (xRight < xLeft || yBottom < yTop) {
            return 0.0f;
        }

        float intersectionArea = (xRight - xLeft) * (yBottom - yTop);
        float box1Area = box1.width() * box1.height();
        float box2Area = box2.width() * box2.height();
        float unionArea = box1Area + box2Area - intersectionArea;

        return intersectionArea / unionArea;
    }

    /**
     * Get COCO class name by ID
     */
    public String getCOCOClassName(int classId) {
        if (classId >= 0 && classId < COCO_CLASSES.length) {
            return COCO_CLASSES[classId];
        }
        return "unknown";
    }

    public boolean isReady() {
        return initialized && interpreter != null;
    }

    public String getInitError() {
        return initError;
    }

    public String getModelPath() {
        return modelPath;
    }

    public int[] getDetectionOutputShape() {
        return detectionOutputShape;
    }

    public int[] getMaskOutputShape() {
        return maskOutputShape;
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

    /**
     * Get number of COCO classes
     */
    public int getNumCOCOClasses() {
        return COCO_CLASSES.length;
    }

    /**
     * Check if initialized
     */
    public boolean isInitialized() {
        return initialized;
    }

    /**
     * Get runtime
     */
    public double getRuntime() {
        return runtime.seconds();
    }

    /**
     * Reset runtime
     */
    public void resetRuntime() {
        runtime.reset();
    }

    /**
     * Get FPS (instantaneous)
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
        if (fpsSampleCount == 0) {
            return 0;
        }
        return fpsSum / fpsSampleCount;
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
        if (inferenceSampleCount == 0) {
            return 0;
        }
        return inferenceTimeSumMs / inferenceSampleCount;
    }

    public double getInferenceFps() {
        return inferenceFps;
    }

    public double getInferenceFpsMovingAverage() {
        return inferenceFpsMovingAverage;
    }

    public double getInferenceFpsAverage() {
        if (inferenceSampleCount == 0) {
            return 0;
        }
        return inferenceFpsSum / inferenceSampleCount;
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
        fps = 0;
        fpsMovingAverage = 0;
        fpsSum = 0;
        fpsSampleCount = 0;
        inferenceTimeMs = 0;
        inferenceTimeMovingAverageMs = 0;
        inferenceTimeSumMs = 0;
        inferenceFps = 0;
        inferenceFpsMovingAverage = 0;
        inferenceFpsSum = 0;
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

    /**
     * Vision processor integrated into the SegmentationYOLO subsystem so examples
     * can reuse drawing and processing logic without redefining a local processor.
     */
    public static class Processor implements org.firstinspires.ftc.vision.VisionProcessor {
        private final SegmentationYOLO segmentor;
        private final android.graphics.Paint boxPaint = new android.graphics.Paint();
        private final android.graphics.Paint textPaint = new android.graphics.Paint();
        private final android.graphics.Paint maskPaint = new android.graphics.Paint();
        private volatile boolean enabled = true;
        private volatile long lastInferenceMs = 0;
        private volatile java.util.List<Segmentation> lastSegmentations = new java.util.ArrayList<>();

        public Processor(SegmentationYOLO segmentor) {
            this.segmentor = segmentor;
            boxPaint.setColor(android.graphics.Color.CYAN);
            boxPaint.setStyle(android.graphics.Paint.Style.STROKE);
            boxPaint.setStrokeWidth(4);

            textPaint.setColor(android.graphics.Color.WHITE);
            textPaint.setTextSize(32f);
            textPaint.setAntiAlias(true);

            maskPaint.setColor(android.graphics.Color.argb(110, 0, 255, 255));
            maskPaint.setStyle(android.graphics.Paint.Style.FILL);
        }

        @Override
        public void init(int width, int height, org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration calibration) {
            // No-op
        }

        @Override
        public Object processFrame(org.opencv.core.Mat frame, long captureTimeNanos) {
            if (!enabled || segmentor == null || !segmentor.isReady()) {
                lastSegmentations = new java.util.ArrayList<>();
                return lastSegmentations;
            }

            long nowMs = System.currentTimeMillis();

            android.graphics.Bitmap bitmap = android.graphics.Bitmap.createBitmap(frame.cols(), frame.rows(), android.graphics.Bitmap.Config.ARGB_8888);
            org.opencv.android.Utils.matToBitmap(frame, bitmap);

            java.util.List<Segmentation> segmentations;
            try {
                segmentations = segmentor.detect(bitmap);
            } catch (Throwable t) {
                segmentations = new java.util.ArrayList<>();
            }
            bitmap.recycle();

            lastSegmentations = segmentations;
            lastInferenceMs = nowMs;
            return segmentations;
        }

        @SuppressWarnings("unchecked")
        @Override
        public void onDrawFrame(android.graphics.Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            java.util.List<Segmentation> segmentations;
            if (userContext instanceof java.util.List) {
                segmentations = (java.util.List<Segmentation>) userContext;
            } else {
                segmentations = lastSegmentations;
            }

            if (segmentations == null) {
                return;
            }

            for (Segmentation seg : segmentations) {
                drawMask(canvas, seg, onscreenWidth, onscreenHeight);

                float left = seg.getX() * onscreenWidth;
                float top = seg.getY() * onscreenHeight;
                float right = (seg.getX() + seg.getWidth()) * onscreenWidth;
                float bottom = (seg.getY() + seg.getHeight()) * onscreenHeight;

                canvas.drawRect(left, top, right, bottom, boxPaint);
                canvas.drawText(
                        seg.className + String.format(" %.0f%%", seg.confidence * 100f),
                        left,
                        Math.max(32, top - 8),
                        textPaint
                );
            }
        }

        public java.util.List<Segmentation> getLastSegmentations() { return lastSegmentations; }

        private void drawMask(android.graphics.Canvas canvas, Segmentation seg, int onscreenWidth, int onscreenHeight) {
            if (seg.mask == null || seg.maskWidth <= 0 || seg.maskHeight <= 0) {
                return;
            }

            int step = 6;
            for (int y = 0; y < seg.maskHeight; y += step) {
                for (int x = 0; x < seg.maskWidth; x += step) {
                    float score = seg.mask[y * seg.maskWidth + x];
                    if (score < 0.5f) {
                        continue;
                    }

                    float px = (x / (float) Math.max(1, seg.maskWidth - 1)) * onscreenWidth;
                    float py = (y / (float) Math.max(1, seg.maskHeight - 1)) * onscreenHeight;
                    canvas.drawRect(px, py, px + step, py + step, maskPaint);
                }
            }
        }

        public void shutdown() {
            enabled = false;
            lastSegmentations = new java.util.ArrayList<>();
        }
    }

    /** Create a processor tied to this SegmentationYOLO instance. */
    public Processor createProcessor() { return new Processor(this); }
}
