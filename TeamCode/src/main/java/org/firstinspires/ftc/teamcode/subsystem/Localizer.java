package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import Jama.Matrix;

import java.util.ArrayList;
import java.util.List;

public class Localizer {
    private Camera camera;
    private AprilTagReader aprilTagReader;
    private AprilTagProcessor aprilTagProcessor;
    private Follower follower;

    private static final double INCH_TO_METER = 0.0254;
    private static final double METER_TO_INCH = 1.0 / INCH_TO_METER;

    private double mergedXMeters;
    private double mergedYMeters;
    private double mergedHeading;

    private boolean aprilTagVisible;
    private int lastVisibleTagId;

    public static class Builder {
        private String webcamName = "Webcam 1";
        private Pose startingPose = new Pose(0, 0, 0);

        public Builder webcamName(String name) {
            this.webcamName = name;
            return this;
        }

        public Builder startingPose(Pose pose) {
            this.startingPose = pose;
            return this;
        }

        public Localizer build() {
            return new Localizer(this);
        }
    }

    private Localizer(Builder builder) {
        Transformation.initialize();
        Transformation.registerAprilTag(Constants.BLUE_GOAL_TAG_ID, Constants.TAG_20_TO_MAP);
        Transformation.registerAprilTag(Constants.RED_GOAL_TAG_ID, Constants.TAG_24_TO_MAP);

        this.mergedXMeters = builder.startingPose.getX() * INCH_TO_METER;
        this.mergedYMeters = builder.startingPose.getY() * INCH_TO_METER;
        this.mergedHeading = builder.startingPose.getHeading();
        this.aprilTagVisible = false;
        this.lastVisibleTagId = -1;
    }

    public void init(HardwareMap hardwareMap) {
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);

        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        aprilTagReader = new AprilTagReader();
        aprilTagReader.setProcessor(aprilTagProcessor);

        camera = new Camera.Builder()
                .webcamName("Webcam 1")
                .build();
        camera.init(hardwareMap, aprilTagProcessor);

        Pose startingPoseInches = new Pose(
                mergedXMeters * METER_TO_INCH,
                mergedYMeters * METER_TO_INCH,
                mergedHeading
        );
        follower.setStartingPose(startingPoseInches);
    }

    public void init(HardwareMap hardwareMap, String webcamName) {
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);

        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        aprilTagReader = new AprilTagReader();
        aprilTagReader.setProcessor(aprilTagProcessor);

        camera = new Camera.Builder()
                .webcamName(webcamName)
                .build();
        camera.init(hardwareMap, aprilTagProcessor);

        Pose startingPoseInches = new Pose(
                mergedXMeters * METER_TO_INCH,
                mergedYMeters * METER_TO_INCH,
                mergedHeading
        );
        follower.setStartingPose(startingPoseInches);
    }

    public void update() {
        follower.update();

        List<AprilTagDetection> detections = aprilTagReader.getDetections();
        List<Transformation.TagDetection> tagDetections = new ArrayList<>();

        for (AprilTagDetection det : detections) {
            if (Transformation.isTagRegistered(det.id)) {
                Matrix cameraToTag = aprilTagReader.getCameraToTagMatrix(det.id);
                tagDetections.add(new Transformation.TagDetection(det.id, cameraToTag));
            }
        }

        if (!tagDetections.isEmpty()) {
            Transformation.RobotPose visionPose = Transformation.getRobotPoseInMapFromMultipleTags(tagDetections);
            if (visionPose != null) {
                mergedXMeters = visionPose.translation[0];
                mergedYMeters = visionPose.translation[1];
                mergedHeading = Math.atan2(visionPose.rotationMatrix[1][0], visionPose.rotationMatrix[0][0]);

                Pose visionPoseInches = new Pose(
                        mergedXMeters * METER_TO_INCH,
                        mergedYMeters * METER_TO_INCH,
                        mergedHeading
                );
                follower.setPose(visionPoseInches);

                aprilTagVisible = true;
                lastVisibleTagId = tagDetections.get(0).tagId;
            } else {
                updateFromFollower();
            }
        } else {
            updateFromFollower();
        }
    }

    private void updateFromFollower() {
        Pose followerPose = follower.getPose();
        mergedXMeters = followerPose.getX() * INCH_TO_METER;
        mergedYMeters = followerPose.getY() * INCH_TO_METER;
        mergedHeading = followerPose.getHeading();
        aprilTagVisible = false;
        lastVisibleTagId = -1;
    }

    public double getXMeters() {
        return mergedXMeters;
    }

    public double getYMeters() {
        return mergedYMeters;
    }

    public double getHeading() {
        return mergedHeading;
    }

    public Pose getPose() {
        return new Pose(
                mergedXMeters * METER_TO_INCH,
                mergedYMeters * METER_TO_INCH,
                mergedHeading
        );
    }

    public Pose getPoseMeters() {
        return new Pose(mergedXMeters, mergedYMeters, mergedHeading);
    }

    public boolean isAprilTagVisible() {
        return aprilTagVisible;
    }

    public int getLastVisibleTagId() {
        return lastVisibleTagId;
    }

    public Follower getFollower() {
        return follower;
    }

    public AprilTagReader getAprilTagReader() {
        return aprilTagReader;
    }

    public void close() {
        if (camera != null) {
            camera.close();
        }
    }
}
