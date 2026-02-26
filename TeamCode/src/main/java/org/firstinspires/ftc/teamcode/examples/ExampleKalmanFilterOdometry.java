package org.firstinspires.ftc.teamcode.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.subsystem.AprilTagReader;
import org.firstinspires.ftc.teamcode.subsystem.Constants;
import org.firstinspires.ftc.teamcode.subsystem.KalmanFilter;
import org.firstinspires.ftc.teamcode.subsystem.Transformation;
import org.firstinspires.ftc.teamcode.subsystem.Transformation.RobotPose;

import Jama.Matrix;

import java.util.ArrayList;
import java.util.List;

/**
 * Example: Kalman Filter fusion of PedroPathing odometry + AprilTag vision.
 *
 * <p>This OpMode demonstrates how to fuse two localization sources:
 * <ol>
 *   <li><b>Predict</b> — PedroPathing 3-wheel + IMU odometry provides fast, continuous
 *       pose updates every loop (high frequency, drifts over time).</li>
 *   <li><b>Update</b> — AprilTag vision provides absolute pose corrections when tags
 *       are visible (lower frequency, no drift).</li>
 * </ol>
 *
 * <p>All coordinates are in the <b>map frame</b> (origin at the bottom-left corner of the
 * field, units in meters and radians). PedroPathing uses inches internally — the
 * KalmanFilter handles the conversion.
 *
 * <p><b>Hardware required:</b>
 * <ul>
 *   <li>3 dead-wheel odometry pods wired to motor encoder ports</li>
 *   <li>IMU (built into Control Hub)</li>
 *   <li>Webcam named in hardware config</li>
 * </ul>
 *
 * <p><b>Tuning guide:</b>
 * <ul>
 *   <li>Increase processNoiseXY if the filter is too slow to follow odometry</li>
 *   <li>Decrease visionNoiseXY if AprilTag measurements are very reliable</li>
 *   <li>Watch positionUncertainty on telemetry — it should drop when tags are seen</li>
 * </ul>
 */
@TeleOp(name = "ExampleKalmanFilterOdometry", group = "Examples")
public class ExampleKalmanFilterOdometry extends LinearOpMode {

    private Follower follower;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private AprilTagReader aprilTagReader;
    private KalmanFilter kalmanFilter;
    private IMU imu;

    private double prevOdomXMeters = 0;
    private double prevOdomYMeters = 0;
    private double prevOdomHeading = 0;
    private static final Pose STARTING_POSE = new Pose(56.37, 9.01, Math.toRadians(90)); // TODO: update to actual starting pose
    private static final long APRILTAG_WARMUP_MILLIS = 3000;
    private static final double INCH_TO_METER = 0.0254;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);

        List<WebcamName> webcams = hardwareMap.getAll(WebcamName.class);
        if (webcams == null || webcams.isEmpty()) {
            telemetry.addData("ERROR", "No webcam found in hardware config");
            telemetry.update();
            sleep(3000);
            return;
        }

        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcams.get(0))
                .addProcessor(aprilTagProcessor)
                .build();

        aprilTagReader = new AprilTagReader();
        aprilTagReader.setProcessor(aprilTagProcessor);
        Transformation.initialize();
        Transformation.registerAprilTag(Constants.BLUE_GOAL_TAG_ID, Constants.BLUE_GOAL_TAG_MAP_FRAME);
        Transformation.registerAprilTag(Constants.RED_GOAL_TAG_ID, Constants.RED_GOAL_TAG_MAP_FRAME);

        kalmanFilter = new KalmanFilter.Builder()
                .setProcessNoiseXY(0.02)
                .setProcessNoiseHeading(0.01)
                .setProcessNoiseVelocity(0.20)
                .setProcessNoiseOmega(0.40)
            .setProcessNoiseImuBias(0.02)
                .setVisionNoiseXY(0.05)
                .setVisionNoiseHeading(0.03)
                .setImuYawRateNoise(0.20)
                .setInitialUncertaintyXY(0.5)
                .setInitialUncertaintyHeading(0.3)
                .setInitialUncertaintyVelocity(1.0)
                .setInitialUncertaintyOmega(1.0)
            .setInitialUncertaintyImuBias(0.5)
                .setVisionDistanceWeightGain(0.35)
                .setSlipProcessNoiseGain(2.0)
                .build();

        imu = hardwareMap.get(IMU.class, "imu");

        Pose initialPose = estimateInitialPoseFromAprilTagWarmup(APRILTAG_WARMUP_MILLIS);
        if (initialPose == null) {
            initialPose = STARTING_POSE;
        }

        follower.setStartingPose(initialPose);

        double initialXMeters = initialPose.getX() * INCH_TO_METER;
        double initialYMeters = initialPose.getY() * INCH_TO_METER;
        kalmanFilter.init(initialXMeters, initialYMeters, initialPose.getHeading());

        prevOdomXMeters = initialXMeters;
        prevOdomYMeters = initialYMeters;
        prevOdomHeading = initialPose.getHeading();

        telemetry.addData("Status", "Ready — press START");
        telemetry.addData("Init Pose Source", initialPose == STARTING_POSE ? "Fallback default" : "AprilTag 3s average");
        telemetry.addData("Init X (m)", "%.3f", initialXMeters);
        telemetry.addData("Init Y (m)", "%.3f", initialYMeters);
        telemetry.addData("Init Heading (deg)", "%.1f", Math.toDegrees(initialPose.getHeading()));
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            follower.update();
            Pose currentOdomPose = follower.getPose();

            // PedroPathing is inches internally. Convert immediately to meters to keep
            // all Kalman-side computations in map meters.
            double odomXInches = currentOdomPose.getX();
            double odomYInches = currentOdomPose.getY();
            double odomXMeters = odomXInches * INCH_TO_METER;
            double odomYMeters = odomYInches * INCH_TO_METER;
            double odomHeading = currentOdomPose.getHeading();

            double dxMeters = odomXMeters - prevOdomXMeters;
            double dyMeters = odomYMeters - prevOdomYMeters;
            double dHeading = KalmanFilter.normalizeAngle(odomHeading - prevOdomHeading);

            prevOdomXMeters = odomXMeters;
            prevOdomYMeters = odomYMeters;
            prevOdomHeading = odomHeading;

            kalmanFilter.predict(dxMeters, dyMeters, dHeading);

            List<AprilTagDetection> detections = aprilTagReader.getDetections();

            boolean visionUpdated = false;
            double avgTagDistanceMeters = Double.NaN;
            if (!detections.isEmpty()) {
                List<Transformation.TagDetection> tagDetections = new ArrayList<>();
                double sumTagDistanceMeters = 0.0;
                int tagDistanceCount = 0;
                for (AprilTagDetection det : detections) {
                    if (Transformation.isTagRegistered(det.id)) {
                        Matrix cameraToTag = aprilTagReader.getCameraToTagMatrix(det.id);
                        tagDetections.add(new Transformation.TagDetection(det.id, cameraToTag));

                        if (det.ftcPose != null
                                && !Double.isNaN(det.ftcPose.range)
                                && !Double.isInfinite(det.ftcPose.range)) {
                            sumTagDistanceMeters += det.ftcPose.range * INCH_TO_METER;
                            tagDistanceCount++;
                        }
                    }
                }

                if (tagDistanceCount > 0) {
                    avgTagDistanceMeters = sumTagDistanceMeters / tagDistanceCount;
                }

                RobotPose visionPose = Transformation.getRobotPoseInMapFromMultipleTags(tagDetections);
                if (visionPose != null) {
                    visionUpdated = kalmanFilter.updateVision(visionPose, avgTagDistanceMeters);
                }
            }

            boolean imuUpdated = false;
            if (imu != null) {
                AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
                if (angularVelocity != null
                        && !Double.isNaN(angularVelocity.zRotationRate)
                        && !Double.isInfinite(angularVelocity.zRotationRate)) {
                    kalmanFilter.updateImuYawRate(angularVelocity.zRotationRate);
                    imuUpdated = true;
                }
            }

            KalmanFilter.PoseEstimate fusedPose = kalmanFilter.getPose();

            telemetry.addData("--- Fused Pose (map frame) ---", "");
            telemetry.addData("  X (m)", "%.3f", fusedPose.xMeters);
            telemetry.addData("  Y (m)", "%.3f", fusedPose.yMeters);
            telemetry.addData("  Heading (deg)", "%.1f", fusedPose.headingDeg());

            telemetry.addData("--- Raw Odometry (PedroPathing) ---", "");
            telemetry.addData("  X (m)", "%.3f", odomXMeters);
            telemetry.addData("  Y (m)", "%.3f", odomYMeters);
            telemetry.addData("  Heading (deg)", "%.1f", Math.toDegrees(odomHeading));

            telemetry.addData("--- Filter Stats ---", "");
            telemetry.addData("  Vision update", visionUpdated ? "YES" : "no");
            telemetry.addData("  Tags visible", detections.size());
            telemetry.addData("  Tag dist avg (m)", Double.isNaN(avgTagDistanceMeters) ? "n/a" : String.format("%.2f", avgTagDistanceMeters));
            telemetry.addData("  Vision accepted", kalmanFilter.wasLastVisionAccepted() ? "YES" : "no");
            telemetry.addData("  Vision maha^2", "%.2f", kalmanFilter.getLastVisionMahalanobisDistance());
            telemetry.addData("  Vision gate", "%.2f", kalmanFilter.getVisionMahalanobisGate());
            telemetry.addData("  Vision dist weight", "%.2f", kalmanFilter.getLastVisionDistanceWeight());
            telemetry.addData("  Slip indicator", "%.2f", kalmanFilter.getLastSlipIndicator());
            telemetry.addData("  IMU yaw-rate update", imuUpdated ? "YES" : "no");
            telemetry.addData("  IMU bias (deg/s)", "%.2f", fusedPose.imuBiasDegPerSec());
            telemetry.addData("  IMU bias σ (deg/s)", "%.2f", Math.toDegrees(kalmanFilter.getImuBiasUncertainty()));
            telemetry.addData("  Pos uncertainty (m)", "%.4f", kalmanFilter.getPositionUncertainty());
            telemetry.addData("  Hdg uncertainty (deg)", "%.2f",
                    Math.toDegrees(kalmanFilter.getHeadingUncertainty()));

            telemetry.addData("--- Velocity State ---", "");
            telemetry.addData("  vx (m/s)", "%.3f", fusedPose.vxMetersPerSec);
            telemetry.addData("  vy (m/s)", "%.3f", fusedPose.vyMetersPerSec);
            telemetry.addData("  omega (deg/s)", "%.1f", fusedPose.omegaDegPerSec());
            telemetry.update();
        }
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    private Pose estimateInitialPoseFromAprilTagWarmup(long warmupMillis) {
        long startNanos = System.nanoTime();
        long durationNanos = warmupMillis * 1_000_000L;

        double sumX = 0.0;
        double sumY = 0.0;
        double sumSinHeading = 0.0;
        double sumCosHeading = 0.0;
        int sampleCount = 0;

        while (!isStopRequested() && (System.nanoTime() - startNanos) < durationNanos) {
            List<AprilTagDetection> detections = aprilTagReader.getDetections();

            if (!detections.isEmpty()) {
                List<Transformation.TagDetection> tagDetections = new ArrayList<>();
                for (AprilTagDetection det : detections) {
                    if (Transformation.isTagRegistered(det.id)) {
                        Matrix cameraToTag = aprilTagReader.getCameraToTagMatrix(det.id);
                        tagDetections.add(new Transformation.TagDetection(det.id, cameraToTag));
                    }
                }

                RobotPose visionPose = Transformation.getRobotPoseInMapFromMultipleTags(tagDetections);
                if (visionPose != null) {
                    double heading = Math.atan2(visionPose.rotationMatrix[1][0], visionPose.rotationMatrix[0][0]);
                    sumX += visionPose.translation[0];
                    sumY += visionPose.translation[1];
                    sumSinHeading += Math.sin(heading);
                    sumCosHeading += Math.cos(heading);
                    sampleCount++;
                }
            }

            double elapsedSec = (System.nanoTime() - startNanos) / 1_000_000_000.0;
            double remainingSec = Math.max(0.0, warmupMillis / 1000.0 - elapsedSec);
            telemetry.addData("Status", "AprilTag warmup");
            telemetry.addData("Warmup remaining (s)", "%.1f", remainingSec);
            telemetry.addData("Warmup samples", sampleCount);
            telemetry.update();

            sleep(20);
        }

        if (sampleCount == 0) {
            return null;
        }

        double avgXMeters = sumX / sampleCount;
        double avgYMeters = sumY / sampleCount;
        double avgHeading = Math.atan2(sumSinHeading / sampleCount, sumCosHeading / sampleCount);

        return new Pose(avgXMeters / INCH_TO_METER, avgYMeters / INCH_TO_METER, avgHeading);
    }
}
