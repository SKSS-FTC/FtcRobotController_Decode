package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import android.util.Size;

public class Camera {
    private VisionPortal visionPortal;
    private String webcamName;
    private Size resolution;
    private boolean streamEnabled;

    public static class Builder {
        private String webcamName = "Webcam 1";
        private Size resolution = new Size(640, 480);
        private boolean streamEnabled = true;

        public Builder webcamName(String name) {
            this.webcamName = name;
            return this;
        }

        public Builder resolution(Size resolution) {
            this.resolution = resolution;
            return this;
        }

        public Builder streamEnabled(boolean enabled) {
            this.streamEnabled = enabled;
            return this;
        }

        public Camera build() {
            return new Camera(this);
        }
    }

    private Camera(Builder builder) {
        this.webcamName = builder.webcamName;
        this.resolution = builder.resolution;
        this.streamEnabled = builder.streamEnabled;
    }

    public void init(HardwareMap hardwareMap, VisionProcessor processor) {
        WebcamName webcam = hardwareMap.get(WebcamName.class, webcamName);
        if (webcam == null) {
            throw new RuntimeException("Webcam \'" + webcamName + "\' not found in hardware map");
        }

        VisionPortal.Builder portalBuilder = new VisionPortal.Builder();
        portalBuilder.setCamera(webcam);
        portalBuilder.setCameraResolution(resolution);
        portalBuilder.addProcessor(processor);

        visionPortal = portalBuilder.build();

        if (!streamEnabled) {
            visionPortal.stopStreaming();
        }
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    public void startStreaming() {
        if (visionPortal != null) {
            visionPortal.resumeStreaming();
        }
    }

    public void stopStreaming() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();
        }
    }

    public VisionPortal.CameraState getCameraState() {
        return visionPortal != null ? visionPortal.getCameraState() : VisionPortal.CameraState.STREAMING;
    }

    public void saveNextFrameRaw(String filename) {
        if (visionPortal != null) {
            visionPortal.saveNextFrameRaw(filename);
        }
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }
}
