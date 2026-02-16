package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import android.content.Context;

/**
 * DataLogger subsystem for logging FPS, voltage, and custom metrics.
 * Uses FTC HardwareMap for voltage sensors, System.nanoTime for timing.
 * Supports both SD card and internal app storage.
 * 
 * Storage Options:
 * 1. Internal storage (default) - No SD card required
 * 2. SD card - External storage for larger logs
 */
public class Datalogger {

    // Constants
    private static final long NANOSECONDS_PER_SECOND = 1_000_000_000L;
    private static final int MAX_HISTORY = 1000;

    // Configuration
    private final String tag;
    private final boolean enableFileLogging;
    private final boolean enableTelemetryLogging;
    private final StorageLocation storageLocation;
    private String logFilePath;

    // Hardware
    private HardwareMap hardwareMap;
    private Context context;

    // State
    private boolean initialized = false;
    private long frameCount = 0;
    private long previousTimestampNanos = 0;
    private double fps = 0.0;
    private double fpsMovingAverage = 0.0;
    private int fpsSampleCount = 0;
    private double fpsSum = 0.0;

    private double voltage = 0.0;
    private double voltageMovingAverage = 0.0;
    private double voltageSum = 0.0;
    private int voltageSampleCount = 0;

    // Metrics storage
    private final java.util.concurrent.ConcurrentHashMap<String, Double> metrics = new java.util.concurrent.ConcurrentHashMap<>();

    // File logging
    private File logFile;
    private BufferedWriter logWriter;
    private final java.text.SimpleDateFormat dateFormat = 
        new java.text.SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS", java.util.Locale.US);

    private static final String TAG = "Datalogger";

    /**
     * Storage location options
     */
    public enum StorageLocation {
        INTERNAL,    // Internal app storage (no SD card required)
        SD_CARD,      // External SD card storage
        MEMORY_ONLY  // No file logging, memory only
    }

    /**
     * Private constructor - use Builder
     */
    private Datalogger(Builder builder) {
        this.tag = builder.tag;
        this.enableFileLogging = builder.enableFileLogging && builder.storageLocation != StorageLocation.MEMORY_ONLY;
        this.enableTelemetryLogging = builder.enableTelemetryLogging;
        this.storageLocation = builder.storageLocation;
        this.logFilePath = builder.logFilePath;
    }

    /**
     * Initialize the DataLogger
     */
    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        // Get Context from HardwareMap (FTC provides this)
        this.context = hardwareMap.appContext;
        this.initialized = true;

        if (enableFileLogging && context != null) {
            setupFileLogging();
        }

        previousTimestampNanos = System.nanoTime();
    }

    /**
     * Setup file logging with appropriate storage location
     */
    private void setupFileLogging() {
        try {
            File logDir;
            File logFile;
            
            if (storageLocation == StorageLocation.INTERNAL) {
                // Use internal app storage - No SD card required
                logDir = new File(context.getFilesDir(), "logs");
            } else {
                // Use SD card storage
                logDir = new File(logFilePath).getParentFile();
            }

            if (logDir != null && !logDir.exists()) {
                logDir.mkdirs();
            }

            if (storageLocation == StorageLocation.INTERNAL) {
                logFile = new File(logDir, logFilePath.substring(logFilePath.lastIndexOf('/') + 1));
            } else {
                logFile = new File(logFilePath);
            }

            this.logWriter = new BufferedWriter(new FileWriter(logFile, false));

            // Write header
            logWriter.write("timestamp,fps,voltage,frame_count");
            if (!metrics.isEmpty()) {
                logWriter.write(",");
                logWriter.write(String.join(",", metrics.keySet()));
            }
            logWriter.newLine();
            logWriter.flush();

            this.logFile = logFile;

        } catch (IOException e) {
            System.err.println(TAG + ": Failed to setup file logging: " + e.getMessage());
        }
    }

    /**
     * Call at the start of each frame to track FPS
     */
    public void startFrame() {
        // FPS calculation must be BEFORE any early returns to count ALL frames
        long now = System.nanoTime();
        
        if (previousTimestampNanos > 0) {
            long deltaNanos = now - previousTimestampNanos;
            if (deltaNanos > 0) {
                fps = NANOSECONDS_PER_SECOND / (double) deltaNanos;
                
                fpsSum += fps;
                fpsSampleCount++;
                fpsMovingAverage = fpsSum / fpsSampleCount;
            }
        }

        previousTimestampNanos = now;
        frameCount++;

        // Now check for early return AFTER FPS calculation
        if (!initialized) {
            return;
        }

        updateVoltage();
    }

    /**
     * Update voltage reading from HardwareMap
     */
    public void updateVoltage() {
        if (!initialized || hardwareMap == null) {
            return;
        }

        try {
            double minVoltage = Double.POSITIVE_INFINITY;
            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                double sensorVoltage = sensor.getVoltage();
                if (sensorVoltage > 0 && sensorVoltage < minVoltage) {
                    minVoltage = sensorVoltage;
                }
            }

            if (minVoltage != Double.POSITIVE_INFINITY) {
                voltage = minVoltage;
                
                voltageSum += voltage;
                voltageSampleCount++;
                voltageMovingAverage = voltageSum / voltageSampleCount;
            }
        } catch (Exception e) {
            // Error reading voltage - use last known value
        }
    }

    /**
     * Log a custom metric
     */
    public void logMetric(String name, double value) {
        metrics.put(name, value);
    }

    /**
     * Call at end of frame to log data
     */
    public void endFrame() {
        if (!initialized) {
            return;
        }

        updateVoltage();

        if (enableFileLogging && logWriter != null) {
            try {
                String timestamp = dateFormat.format(new java.util.Date());
                StringBuilder line = new StringBuilder();
                line.append(timestamp).append(",");
                line.append(String.format(java.util.Locale.US, "%.2f", fps)).append(",");
                line.append(String.format(java.util.Locale.US, "%.2f", voltage)).append(",");
                line.append(frameCount);

                for (String metricName : metrics.keySet()) {
                    line.append(",");
                    line.append(String.format(java.util.Locale.US, "%.4f", metrics.get(metricName)));
                }

                logWriter.write(line.toString());
                logWriter.newLine();
                logWriter.flush();

            } catch (IOException e) {
                System.err.println(TAG + ": Failed to log to file: " + e.getMessage());
            }
        }
    }

    /**
     * Get current FPS
     */
    public double getFPS() {
        return fps;
    }

    /**
     * Get moving average FPS
     */
    public double getFPSMovingAverage() {
        return fpsMovingAverage;
    }

    /**
     * Get current voltage
     */
    public double getVoltage() {
        return voltage;
    }

    /**
     * Get moving average voltage
     */
    public double getVoltageMovingAverage() {
        return voltageMovingAverage;
    }

    /**
     * Get frame count
     */
    public long getFrameCount() {
        return frameCount;
    }

    /**
     * Get a custom metric value
     */
    public Double getMetric(String name) {
        return metrics.get(name);
    }

    /**
     * Reset statistics
     */
    public void reset() {
        frameCount = 0;
        fpsMovingAverage = 0.0;
        fpsSum = 0.0;
        fpsSampleCount = 0;
        voltageMovingAverage = 0.0;
        voltageSum = 0.0;
        voltageSampleCount = 0;
        metrics.clear();
    }

    /**
     * Close the DataLogger and release resources
     */
    public void close() {
        if (logWriter != null) {
            try {
                logWriter.flush();
                logWriter.close();
            } catch (IOException e) {
                System.err.println(TAG + ": Error closing logger: " + e.getMessage());
            }
            logWriter = null;
        }

        initialized = false;
    }

    /**
     * Get log file path (may be internal or SD card)
     */
    public String getLogFilePath() {
        if (logFile != null) {
            return logFile.getAbsolutePath();
        }
        return logFilePath + " (" + storageLocation + ")";
    }

    /**
     * Check if initialized
     */
    public boolean isInitialized() {
        return initialized;
    }

    /**
     * Get storage location
     */
    public StorageLocation getStorageLocation() {
        return storageLocation;
    }

    /**
     * Builder class for Datalogger configuration
     */
    public static class Builder {
        private String tag = "Yologger";
        private boolean enableFileLogging = false;
        private boolean enableTelemetryLogging = true;
        private StorageLocation storageLocation = StorageLocation.INTERNAL; // Default to INTERNAL
        private String logFilePath = "yolo_log.csv";

        public Builder() {
        }

        public Builder tag(String tag) {
            this.tag = tag;
            return this;
        }

        public Builder enableFileLogging(boolean enable) {
            this.enableFileLogging = enable;
            return this;
        }

        public Builder enableTelemetryLogging(boolean enable) {
            this.enableTelemetryLogging = enable;
            return this;
        }

        public Builder storageLocation(StorageLocation location) {
            this.storageLocation = location;
            return this;
        }

        public Builder logFilePath(String path) {
            this.logFilePath = path;
            return this;
        }

        /**
         * Convenience method for SD card storage
         */
        public Builder useSDCard(String path) {
            this.storageLocation = StorageLocation.SD_CARD;
            this.logFilePath = path;
            return this;
        }

        /**
         * Convenience method for internal storage (no SD card)
         */
        public Builder useInternalStorage(String filename) {
            this.storageLocation = StorageLocation.INTERNAL;
            this.logFilePath = filename;
            return this;
        }

        public Datalogger build() {
            return new Datalogger(this);
        }
    }
}
