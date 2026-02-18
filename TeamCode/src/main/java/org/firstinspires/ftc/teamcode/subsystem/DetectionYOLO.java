package org.firstinspires.ftc.teamcode.subsystem;

import android.content.Context;
import android.content.res.AssetFileDescriptor;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Rect;

import android.graphics.Paint;
import android.graphics.RectF;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.tensorflow.lite.DataType;
import org.tensorflow.lite.Interpreter;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import java.util.Locale;

public class DetectionYOLO {

    private static final float DEFAULT_CONFIDENCE_THRESHOLD = 0.25f;

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

    private String modelPath;
    private String labelsPath;
    private float confidenceThreshold;
    private int numThreads;

    private Interpreter interpreter;
    private MappedByteBuffer modelBuffer;
    private String initError;

    private int modelInputWidth = 640;
    private int modelInputHeight = 640;
    private boolean inputIsNCHW = false;
    private DataType inputDataType = DataType.FLOAT32;
    private float inputQuantScale = 1.0f;
    private int inputQuantZeroPoint = 0;

    private DataType outputDataType = DataType.FLOAT32;
    private float outputQuantScale = 1.0f;
    private int outputQuantZeroPoint = 0;

    private int numCandidates = 300;
    private boolean outputBoxesOnLastDim = true;
    private final Object lastDetectionsLock = new Object();
    private List<Detection> lastDetections = new ArrayList<>();

    private final List<String> classNames = new ArrayList<>();
    private final Object inferenceLock = new Object();
    private volatile boolean closing = false;

    private VisionPortal visionPortal;
    private int frameWidth = 640;
    private int frameHeight = 480;
    private boolean initialized = false;

    public static class Detection {
        public final int classId;
        public final String className;
        public final float confidence;
        public final float x, y;
        public final float width, height;

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
    }

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
        private float confidenceThreshold = DEFAULT_CONFIDENCE_THRESHOLD;
        private int numThreads = 4;
        private String webcamName = "Webcam 1";
        private int frameWidth = 640;
        private int frameHeight = 480;
        private boolean useBuiltinCamera = false;

        public Builder modelPath(String modelPath) { this.modelPath = modelPath; return this; }
        public Builder labelsPath(String labelsPath) { this.labelsPath = labelsPath; return this; }
        public Builder numThreads(int numThreads) { this.numThreads = numThreads; return this; }
        public Builder confidenceThreshold(float t) { this.confidenceThreshold = t; return this; }
        public Builder webcamName(String name) { this.webcamName = name; return this; }
        public Builder frameSize(int width, int height) { this.frameWidth = width; this.frameHeight = height; return this; }
        public Builder useBuiltinCamera(boolean use) { this.useBuiltinCamera = use; return this; }

        public DetectionYOLO build() { return new DetectionYOLO(this); }
    }

    private DetectionYOLO(Builder b) {
        this.modelPath = b.modelPath;
        this.labelsPath = b.labelsPath;
        this.confidenceThreshold = b.confidenceThreshold;
        this.numThreads = b.numThreads;
        this.frameWidth = b.frameWidth;
        this.frameHeight = b.frameHeight;
    }

    public void init(Context context) {
        init(context, null);
    }

    public void init(Context context, HardwareMap hardwareMap) {
        initError = null;
        closing = false;

        try {
            if (!ensureTensorFlowLiteNativeLoaded(context)) return;

            modelBuffer = loadModelBuffer(context);

            Interpreter.Options options = new Interpreter.Options();
            options.setNumThreads(numThreads);
            interpreter = new Interpreter(modelBuffer, options);

            org.tensorflow.lite.Tensor inTensor = interpreter.getInputTensor(0);
            int[] inputShape = inTensor.shape();
            inputDataType = inTensor.dataType();

            org.tensorflow.lite.Tensor.QuantizationParams inQ = inTensor.quantizationParams();
            if (inQ != null && inQ.getScale() > 0f) {
                inputQuantScale = inQ.getScale();
                inputQuantZeroPoint = inQ.getZeroPoint();
            }

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

            org.tensorflow.lite.Tensor outTensor = interpreter.getOutputTensor(0);
            int[] outputShape = outTensor.shape();
            outputDataType = outTensor.dataType();

            org.tensorflow.lite.Tensor.QuantizationParams outQ = outTensor.quantizationParams();
            if (outQ != null && outQ.getScale() > 0f) {
                outputQuantScale = outQ.getScale();
                outputQuantZeroPoint = outQ.getZeroPoint();
            }

            if (outputShape.length != 3 || (outputShape[1] != 6 && outputShape[2] != 6)) {
                initError = "YOLO26 O2O expects [1,N,6] or [1,6,N]; got " + Arrays.toString(outputShape);
                interpreter = null;
                modelBuffer = null;
                return;
            }

            outputBoxesOnLastDim = outputShape[2] == 6;
            numCandidates = outputBoxesOnLastDim ? outputShape[1] : outputShape[2];

            loadClassNames(context);

            if (hardwareMap != null) {
                initVisionPortal(hardwareMap);
            }

        } catch (IOException e) {
            initError = "Failed to load model: " + e.getMessage();
            interpreter = null;
            modelBuffer = null;
        } catch (Throwable t) {
            initError = "YOLO init failed: " + t.getMessage();
            interpreter = null;
            modelBuffer = null;
        }
    }

    private void initVisionPortal(HardwareMap hardwareMap) {
        VisionPortal.Builder portalBuilder = new VisionPortal.Builder();

        boolean useBuiltin = false;
        WebcamName webcam = null;
        try {
            webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
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

        portalBuilder.setCameraResolution(new Size(frameWidth, frameHeight));
        portalBuilder.addProcessor(createVisionProcessor());

        visionPortal = portalBuilder.build();
        initialized = true;
    }

    public boolean isInitialized() {
        return initialized && isReady();
    }

    public List<DetectionResult> getDetectionResults() {
        List<DetectionResult> results = new ArrayList<>();

        if (!isInitialized()) {
            return results;
        }

        List<Detection> detections = getLastDetectionsSnapshot();

        for (Detection d : detections) {
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

    public DetectionResult getBestDetectionResult() {
        List<DetectionResult> detections = getDetectionResults();
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

    public DetectionResult getDetectionResultByTag(String tag) {
        List<DetectionResult> detections = getDetectionResults();
        for (DetectionResult d : detections) {
            if (d.tag.equalsIgnoreCase(tag)) {
                return d;
            }
        }
        return null;
    }

    public List<DetectionResult> getDetectionResultsByTag(String tag) {
        List<DetectionResult> results = new ArrayList<>();
        List<DetectionResult> detections = getDetectionResults();
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

    public List<Detection> detect(Bitmap bitmap) {
        if (closing || interpreter == null) {
            return new ArrayList<>();
        }

        PreprocessResult preprocess = createLetterboxedInput(bitmap);
        Bitmap inputBitmap = preprocess.inputBitmap;
        ByteBuffer inputBuffer = preprocessInput(inputBitmap);

        org.tensorflow.lite.Tensor outTensor = interpreter.getOutputTensor(0);
        ByteBuffer outputBuffer = ByteBuffer.allocateDirect(outTensor.numBytes());
        outputBuffer.order(ByteOrder.nativeOrder());

        try {
            synchronized (inferenceLock) {
                if (closing || interpreter == null) {
                    inputBitmap.recycle();
                    return new ArrayList<>();
                }
                interpreter.run(inputBuffer, outputBuffer);
            }
        } catch (RuntimeException e) {
            inputBitmap.recycle();
            return new ArrayList<>();
        }

        outputBuffer.rewind();
        float[] output = readOutputToFloat(outputBuffer);

        List<Detection> detections = parseOneToOneOutput(output, bitmap.getWidth(), bitmap.getHeight(), preprocess);

        inputBitmap.recycle();
        return detections;
    }

    private static class PreprocessResult {
        final Bitmap inputBitmap;
        final float scale;
        final float padX;
        final float padY;

        PreprocessResult(Bitmap inputBitmap, float scale, float padX, float padY) {
            this.inputBitmap = inputBitmap;
            this.scale = scale;
            this.padX = padX;
            this.padY = padY;
        }
    }

    private PreprocessResult createLetterboxedInput(Bitmap src) {
        int srcW = src.getWidth();
        int srcH = src.getHeight();

        float scale = Math.min(modelInputWidth / (float) srcW, modelInputHeight / (float) srcH);
        int resizedW = Math.max(1, Math.round(srcW * scale));
        int resizedH = Math.max(1, Math.round(srcH * scale));

        int dx = (modelInputWidth - resizedW) / 2;
        int dy = (modelInputHeight - resizedH) / 2;

        Bitmap input = Bitmap.createBitmap(modelInputWidth, modelInputHeight, Bitmap.Config.ARGB_8888);
        Canvas canvas = new Canvas(input);
        canvas.drawColor(Color.BLACK);
        Rect dst = new Rect(dx, dy, dx + resizedW, dy + resizedH);
        canvas.drawBitmap(src, null, dst, null);

        return new PreprocessResult(input, scale, dx, dy);
    }

    private Detection mapToOriginal(float nx, float ny, float nw, float nh,
                                     int origW, int origH, float scale, float padX, float padY) {
        float x1Model = nx * modelInputWidth;
        float y1Model = ny * modelInputHeight;
        float x2Model = (nx + nw) * modelInputWidth;
        float y2Model = (ny + nh) * modelInputHeight;

        float x1Orig = (x1Model - padX) / scale;
        float y1Orig = (y1Model - padY) / scale;
        float x2Orig = (x2Model - padX) / scale;
        float y2Orig = (y2Model - padY) / scale;

        x1Orig = clampRange(x1Orig, 0f, origW);
        y1Orig = clampRange(y1Orig, 0f, origH);
        x2Orig = clampRange(x2Orig, 0f, origW);
        y2Orig = clampRange(y2Orig, 0f, origH);

        float wOrig = x2Orig - x1Orig;
        float hOrig = y2Orig - y1Orig;
        if (wOrig <= 0f || hOrig <= 0f) return null;

        float normX = round4(x1Orig / origW);
        float normY = round4(y1Orig / origH);
        float normW = round4(wOrig / origW);
        float normH = round4(hOrig / origH);

        return new Detection(-1, "", 0f, normX, normY, normW, normH);
    }

    private List<Detection> parseOneToOneOutput(float[] output, int origW, int origH, PreprocessResult preprocess) {
        List<Detection> detections = new ArrayList<>();
        int stride = 6;
        int maxBoxes = Math.min(numCandidates, output.length / stride);

        for (int i = 0; i < maxBoxes; i++) {
            float x1, y1, x2, y2, score, clsIdF;

            if (outputBoxesOnLastDim) {
                int base = i * stride;
                x1 = output[base];
                y1 = output[base + 1];
                x2 = output[base + 2];
                y2 = output[base + 3];
                score = output[base + 4];
                clsIdF = output[base + 5];
            } else {
                x1 = output[0 * numCandidates + i];
                y1 = output[1 * numCandidates + i];
                x2 = output[2 * numCandidates + i];
                y2 = output[3 * numCandidates + i];
                score = output[4 * numCandidates + i];
                clsIdF = output[5 * numCandidates + i];
            }

            if (score < confidenceThreshold) continue;

            float maxCoord = Math.max(Math.max(Math.abs(x1), Math.abs(y1)), Math.max(Math.abs(x2), Math.abs(y2)));
            if (maxCoord > 1.5f) {
                x1 /= modelInputWidth;
                y1 /= modelInputHeight;
                x2 /= modelInputWidth;
                y2 /= modelInputHeight;
            }

            float nx1 = clamp01(x1);
            float ny1 = clamp01(y1);
            float nx2 = clamp01(x2);
            float ny2 = clamp01(y2);

            float boxW = nx2 - nx1;
            float boxH = ny2 - ny1;
            if (boxW <= 0f || boxH <= 0f) continue;

            Detection mapped = mapToOriginal(nx1, ny1, boxW, boxH, origW, origH, preprocess.scale, preprocess.padX, preprocess.padY);
            if (mapped == null) continue;

            int classId = Math.max(0, Math.round(clsIdF));
            detections.add(new Detection(classId, getClassName(classId), score,
                    mapped.x, mapped.y, mapped.width, mapped.height));
        }

        detections.sort((a, b) -> Float.compare(b.confidence, a.confidence));
        return detections;
    }

    private ByteBuffer preprocessInput(Bitmap bitmap) {
        int pixelCount = modelInputWidth * modelInputHeight;
        int[] pixels = new int[pixelCount];
        bitmap.getPixels(pixels, 0, modelInputWidth, 0, 0, modelInputWidth, modelInputHeight);

        int bytesPerElement = (inputDataType == DataType.FLOAT32) ? 4 : 1;
        ByteBuffer buffer = ByteBuffer.allocateDirect(pixelCount * 3 * bytesPerElement);
        buffer.order(ByteOrder.nativeOrder());

        if (inputIsNCHW) {
            for (int i = 0; i < pixelCount; i++) writeChannel(buffer, (pixels[i] >> 16) & 0xFF);
            for (int i = 0; i < pixelCount; i++) writeChannel(buffer, (pixels[i] >> 8) & 0xFF);
            for (int i = 0; i < pixelCount; i++) writeChannel(buffer, pixels[i] & 0xFF);
        } else {
            for (int i = 0; i < pixelCount; i++) {
                int px = pixels[i];
                writeChannel(buffer, (px >> 16) & 0xFF);
                writeChannel(buffer, (px >> 8) & 0xFF);
                writeChannel(buffer, px & 0xFF);
            }
        }

        buffer.rewind();
        return buffer;
    }

    private void writeChannel(ByteBuffer buffer, int value) {
        if (inputDataType == DataType.FLOAT32) {
            buffer.putFloat(value / 255.0f);
        } else if (inputDataType == DataType.UINT8) {
            buffer.put((byte) value);
        } else {
            int q = Math.round((value / 255.0f) / inputQuantScale) + inputQuantZeroPoint;
            buffer.put((byte) Math.max(-128, Math.min(127, q)));
        }
    }

    private float[] readOutputToFloat(ByteBuffer buffer) {
        buffer.rewind();
        if (outputDataType == DataType.FLOAT32) {
            int count = buffer.remaining() / 4;
            float[] result = new float[count];
            buffer.asFloatBuffer().get(result);
            return result;
        }

        int count = buffer.remaining();
        float[] result = new float[count];
        for (int i = 0; i < count; i++) {
            int q = (outputDataType == DataType.UINT8) ? (buffer.get() & 0xFF) : buffer.get();
            result[i] = (q - outputQuantZeroPoint) * outputQuantScale;
        }
        return result;
    }

    private void loadClassNames(Context context) {
        classNames.clear();
        if (labelsPath != null && !labelsPath.trim().isEmpty()) {
            try (BufferedReader reader = new BufferedReader(
                    new InputStreamReader(context.getAssets().open(labelsPath)))) {
                String line;
                while ((line = reader.readLine()) != null) {
                    if (!line.trim().isEmpty()) classNames.add(line.trim());
                }
            } catch (IOException ignored) { }
        }
        if (classNames.isEmpty()) {
            for (String n : COCO_CLASSES) classNames.add(n);
        }
    }

    private String getClassName(int classId) {
        if (classId >= 0 && classId < classNames.size()) return classNames.get(classId);
        if (classId >= 0 && classId < COCO_CLASSES.length) return COCO_CLASSES[classId];
        return "class_" + classId;
    }

    public boolean isReady() { return interpreter != null && modelBuffer != null; }
    public String getInitError() { return initError; }
    public String getModelPath() { return modelPath; }

    public VisionProcessor createVisionProcessor() {
        return new YoloVisionProcessor();
    }

    public List<Detection> getLastDetectionsSnapshot() {
        synchronized (lastDetectionsLock) {
            return new ArrayList<>(lastDetections);
        }
    }

    public void close() {
        synchronized (inferenceLock) {
            closing = true;
            if (interpreter != null) {
                interpreter.close();
                interpreter = null;
            }
            modelBuffer = null;
        }
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
        initialized = false;
    }

    private MappedByteBuffer loadModelBuffer(Context context) throws IOException {
        try (AssetFileDescriptor afd = context.getAssets().openFd(modelPath);
             FileInputStream fis = new FileInputStream(afd.getFileDescriptor());
             FileChannel ch = fis.getChannel()) {
            return ch.map(FileChannel.MapMode.READ_ONLY, afd.getStartOffset(), afd.getDeclaredLength());
        } catch (IOException ignored) {
            java.io.File cacheFile = new java.io.File(context.getCacheDir(), modelPath);
            if (!cacheFile.exists() || cacheFile.length() == 0) {
                try (java.io.InputStream is = context.getAssets().open(modelPath);
                     FileOutputStream fos = new FileOutputStream(cacheFile)) {
                    byte[] buf = new byte[8192];
                    int n;
                    while ((n = is.read(buf)) != -1) fos.write(buf, 0, n);
                    fos.flush();
                }
            }
            try (FileInputStream fis = new FileInputStream(cacheFile);
                 FileChannel ch = fis.getChannel()) {
                return ch.map(FileChannel.MapMode.READ_ONLY, 0, ch.size());
            }
        }
    }

    private boolean ensureTensorFlowLiteNativeLoaded(Context context) {
        String[] names = { "tensorflowlite_jni", "tensorflowlite_jni_stable", "tensorflowlite_jni_gms_client" };
        for (String name : names) {
            try { System.loadLibrary(name); return true; } catch (UnsatisfiedLinkError ignored) { }
        }
        java.io.File direct = new java.io.File(context.getApplicationInfo().nativeLibraryDir, "libtensorflowlite_jni.so");
        if (direct.exists()) {
            try { System.load(direct.getAbsolutePath()); return true; } catch (UnsatisfiedLinkError ignored) { }
        }
        initError = "Failed to load TFLite native library";
        return false;
    }

    private float clamp01(float v) {
        return clampRange(v, 0f, 1f);
    }

    private float clampRange(float v, float min, float max) {
        return Math.max(min, Math.min(max, v));
    }

    private float round4(float v) {
        return Math.round(v * 10000f) / 10000f;
    }

    private class YoloVisionProcessor implements VisionProcessor {
        private final Paint boxPaint = new Paint();
        private final Paint textPaint = new Paint();
        private int processorFrameWidth = 1;
        private int processorFrameHeight = 1;

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            boxPaint.setColor(Color.GREEN);
            boxPaint.setStyle(Paint.Style.STROKE);
            boxPaint.setStrokeWidth(3f);
            boxPaint.setAntiAlias(true);

            textPaint.setColor(Color.WHITE);
            textPaint.setTextSize(24f);
            textPaint.setAntiAlias(true);
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            if (closing) return null;

            processorFrameWidth = frame.cols();
            processorFrameHeight = frame.rows();

            Bitmap bmp = Bitmap.createBitmap(frame.cols(), frame.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(frame, bmp);
            List<Detection> detections = detect(bmp);
            bmp.recycle();

            synchronized (lastDetectionsLock) {
                lastDetections = detections;
            }
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            List<Detection> snapshot;
            synchronized (lastDetectionsLock) {
                snapshot = new ArrayList<>(lastDetections);
            }

            if (snapshot.isEmpty()) return;

            boxPaint.setStrokeWidth(3f * scaleCanvasDensity);
            textPaint.setTextSize(20f * scaleCanvasDensity);

            float scale = Math.min(onscreenWidth / (float) processorFrameWidth, onscreenHeight / (float) processorFrameHeight);
            float padX = (onscreenWidth - processorFrameWidth * scale) / 2f;
            float padY = (onscreenHeight - processorFrameHeight * scale) / 2f;

            for (Detection d : snapshot) {
                float left = padX + d.x * processorFrameWidth * scale;
                float top = padY + d.y * processorFrameHeight * scale;
                float right = padX + (d.x + d.width) * processorFrameWidth * scale;
                float bottom = padY + (d.y + d.height) * processorFrameHeight * scale;

                RectF box = new RectF(left, top, right, bottom);
                canvas.drawRect(box, boxPaint);

                String label = String.format(Locale.US, "%s %.1f%% @ %.4f,%.4f,%.4f,%.4f",
                        d.className, d.confidence * 100f, d.x, d.y, d.width, d.height);
                canvas.drawText(label, left + 6f, top + 22f * scaleCanvasDensity, textPaint);
            }
        }
    }
}
