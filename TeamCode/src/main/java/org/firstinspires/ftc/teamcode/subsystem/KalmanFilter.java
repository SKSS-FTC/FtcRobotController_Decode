package org.firstinspires.ftc.teamcode.subsystem;

import Jama.Matrix;

/**
 * 7-state Kalman filter for robot localization.
 * Fuses odometry prediction, AprilTag absolute pose, and IMU yaw-rate.
 *
 * <p>State vector: [x, y, heading, vx, vy, omega, imuBias] in map frame
 * (meters, radians, m/s, rad/s, rad/s).
 * <ul>
 *   <li>Predict: odometry deltas from PedroPathing (already converted to meters/radians by caller)</li>
 *   <li>Vision update: AprilTag pose correction with distance-weighted covariance + Mahalanobis gating</li>
 *   <li>IMU update: yaw-rate correction channel</li>
 * </ul>
 *
 * <p>All coordinates are in the map frame (origin at bottom-left corner of the field).
 * PedroPathing provides inches — callers must convert to meters before calling init()/predict().
 * AprilTag poses from {@link Transformation} are already in meters.
 */
public class KalmanFilter {

    private static final int IDX_X = 0;
    private static final int IDX_Y = 1;
    private static final int IDX_HEADING = 2;
    private static final int IDX_VX = 3;
    private static final int IDX_VY = 4;
    private static final int IDX_OMEGA = 5;
    private static final int IDX_IMU_BIAS = 6;

    private static final int STATE_SIZE = 7;

    private static final Matrix I_STATE = Matrix.identity(STATE_SIZE, STATE_SIZE);
    private static final double MIN_DT_SECONDS = 1e-3;
    private static final double MAX_DT_SECONDS = 0.2;
    private static final double DEFAULT_DT_SECONDS = 0.02;

    private Matrix x;
    private Matrix P;

    private Matrix baseQ;
    private Matrix baseRVision;
    private Matrix baseRImuYawRate;

    private double visionMahalanobisGate = 11.34; // Chi-square threshold, DOF=3, ~99%
    private boolean enableVisionGating = true;

    private double visionDistanceWeightGain = 0.35;
    private double slipProcessNoiseGain = 2.0;
    private double velocityBlend = 0.25;

    private boolean lastVisionAccepted = false;
    private double lastVisionMahalanobisDistance = Double.NaN;
    private double lastVisionDistanceWeight = 1.0;
    private double lastSlipIndicator = 0.0;

    private long lastPredictTimeNanos = 0;
    private boolean initialized = false;

    /**
     * Builder for configuring KalmanFilter noise parameters.
     *
     * <p>Default noise values are reasonable starting points for a typical FTC mecanum robot
     * with 3-wheel dead-wheel odometry and a webcam running AprilTag detection.
     */
    public static class Builder {
        private double processNoiseXY = 0.02;
        private double processNoiseHeading = 0.01;
        private double processNoiseVelocity = 0.20;
        private double processNoiseOmega = 0.40;
        private double processNoiseImuBias = 0.02;
        private double visionNoiseXY = 0.05;
        private double visionNoiseHeading = 0.03;
        private double imuYawRateNoise = 0.20;
        private double initialUncertaintyXY = 0.5;    // meters — initial position uncertainty
        private double initialUncertaintyHeading = 0.3; // radians — initial heading uncertainty
        private double initialUncertaintyVelocity = 1.0; // m/s
        private double initialUncertaintyOmega = 1.0; // rad/s
        private double initialUncertaintyImuBias = 0.5; // rad/s
        private double visionMahalanobisGate = 11.34;
        private boolean enableVisionGating = true;
        private double visionDistanceWeightGain = 0.35;
        private double slipProcessNoiseGain = 2.0;
        private double velocityBlend = 0.25;

        /** Odometry position noise per predict step (meters). Default: 0.02 */
        public Builder setProcessNoiseXY(double noise) {
            this.processNoiseXY = noise;
            return this;
        }

        /** Odometry heading noise per predict step (radians). Default: 0.01 */
        public Builder setProcessNoiseHeading(double noise) {
            this.processNoiseHeading = noise;
            return this;
        }

        /** Velocity process noise (m/s). Default: 0.20 */
        public Builder setProcessNoiseVelocity(double noise) {
            this.processNoiseVelocity = noise;
            return this;
        }

        /** Angular velocity process noise (rad/s). Default: 0.40 */
        public Builder setProcessNoiseOmega(double noise) {
            this.processNoiseOmega = noise;
            return this;
        }

        /** IMU bias process noise (rad/s). Default: 0.02 */
        public Builder setProcessNoiseImuBias(double noise) {
            this.processNoiseImuBias = noise;
            return this;
        }

        /** AprilTag position measurement noise (meters). Default: 0.05 */
        public Builder setVisionNoiseXY(double noise) {
            this.visionNoiseXY = noise;
            return this;
        }

        /** AprilTag heading measurement noise (radians). Default: 0.03 */
        public Builder setVisionNoiseHeading(double noise) {
            this.visionNoiseHeading = noise;
            return this;
        }

        /** IMU yaw-rate measurement noise (rad/s). Default: 0.20 */
        public Builder setImuYawRateNoise(double noise) {
            this.imuYawRateNoise = noise;
            return this;
        }

        /** Initial position uncertainty (meters). Default: 0.5 */
        public Builder setInitialUncertaintyXY(double uncertainty) {
            this.initialUncertaintyXY = uncertainty;
            return this;
        }

        /** Initial heading uncertainty (radians). Default: 0.3 */
        public Builder setInitialUncertaintyHeading(double uncertainty) {
            this.initialUncertaintyHeading = uncertainty;
            return this;
        }

        /** Initial velocity uncertainty (m/s). Default: 1.0 */
        public Builder setInitialUncertaintyVelocity(double uncertainty) {
            this.initialUncertaintyVelocity = uncertainty;
            return this;
        }

        /** Initial angular velocity uncertainty (rad/s). Default: 1.0 */
        public Builder setInitialUncertaintyOmega(double uncertainty) {
            this.initialUncertaintyOmega = uncertainty;
            return this;
        }

        /** Initial IMU bias uncertainty (rad/s). Default: 0.5 */
        public Builder setInitialUncertaintyImuBias(double uncertainty) {
            this.initialUncertaintyImuBias = uncertainty;
            return this;
        }

        /**
         * Mahalanobis gate threshold for vision updates.
         *
         * <p>Typical value for 3D measurement [x, y, heading]:
         * 11.34 (chi-square 99% confidence).
         */
        public Builder setVisionMahalanobisGate(double gate) {
            this.visionMahalanobisGate = gate;
            return this;
        }

        /** Enable/disable Mahalanobis gating for vision updates. Default: true */
        public Builder setEnableVisionGating(boolean enabled) {
            this.enableVisionGating = enabled;
            return this;
        }

        /** Distance weight gain for AprilTag covariance scaling. Default: 0.35 */
        public Builder setVisionDistanceWeightGain(double gain) {
            this.visionDistanceWeightGain = gain;
            return this;
        }

        /** Additional process noise gain during detected slip. Default: 2.0 */
        public Builder setSlipProcessNoiseGain(double gain) {
            this.slipProcessNoiseGain = gain;
            return this;
        }

        /** Blend factor for velocity state from odometry velocity. Default: 0.25 */
        public Builder setVelocityBlend(double blend) {
            this.velocityBlend = Math.max(0.0, Math.min(1.0, blend));
            return this;
        }

        public KalmanFilter build() {
            KalmanFilter kf = new KalmanFilter();

                kf.baseQ = makeDiagonal7(
                    processNoiseXY,
                    processNoiseXY,
                    processNoiseHeading,
                    processNoiseVelocity,
                    processNoiseVelocity,
                    processNoiseOmega,
                    processNoiseImuBias);

            kf.baseRVision = makeDiagonal3(visionNoiseXY, visionNoiseXY, visionNoiseHeading);
            kf.baseRImuYawRate = new Matrix(new double[][]{{imuYawRateNoise * imuYawRateNoise}});

                kf.P = makeDiagonal7(
                    initialUncertaintyXY,
                    initialUncertaintyXY,
                    initialUncertaintyHeading,
                    initialUncertaintyVelocity,
                    initialUncertaintyVelocity,
                    initialUncertaintyOmega,
                    initialUncertaintyImuBias);

            kf.x = new Matrix(STATE_SIZE, 1, 0.0);
            kf.visionMahalanobisGate = visionMahalanobisGate;
            kf.enableVisionGating = enableVisionGating;
            kf.visionDistanceWeightGain = visionDistanceWeightGain;
            kf.slipProcessNoiseGain = slipProcessNoiseGain;
            kf.velocityBlend = velocityBlend;
            return kf;
        }
    }

    private KalmanFilter() {
    }

    /**
     * Set the initial state of the filter.
     *
     * @param xMeters  robot X in map frame (meters)
     * @param yMeters  robot Y in map frame (meters)
     * @param headingRad robot heading in map frame (radians)
     */
    public void init(double xMeters, double yMeters, double headingRad) {
        x.set(IDX_X, 0, xMeters);
        x.set(IDX_Y, 0, yMeters);
        x.set(IDX_HEADING, 0, normalizeAngle(headingRad));
        x.set(IDX_VX, 0, 0.0);
        x.set(IDX_VY, 0, 0.0);
        x.set(IDX_OMEGA, 0, 0.0);
        x.set(IDX_IMU_BIAS, 0, 0.0);
        lastPredictTimeNanos = System.nanoTime();
        initialized = true;
        lastVisionAccepted = false;
        lastVisionMahalanobisDistance = Double.NaN;
        lastVisionDistanceWeight = 1.0;
        lastSlipIndicator = 0.0;
    }

    /**
     * Predict step using odometry deltas in map frame (meters, radians).
     *
     * <p>The deltas should be the change since the last predict call,
     * already rotated into the map frame.
     *
     * @param dxMeters    change in X (meters, map frame)
     * @param dyMeters    change in Y (meters, map frame)
     * @param dHeadingRad change in heading (radians)
     */
    public void predict(double dxMeters, double dyMeters, double dHeadingRad) {
        if (!initialized) return;

        double dt = computeDtSeconds();

        double previousVx = x.get(IDX_VX, 0);
        double previousVy = x.get(IDX_VY, 0);
        double previousOmega = x.get(IDX_OMEGA, 0);

        double measuredVx = dxMeters / dt;
        double measuredVy = dyMeters / dt;
        double measuredOmega = dHeadingRad / dt;

        double updatedVx = blend(previousVx, measuredVx, velocityBlend);
        double updatedVy = blend(previousVy, measuredVy, velocityBlend);
        double updatedOmega = blend(previousOmega, measuredOmega, velocityBlend);

        double predictedDx = previousVx * dt;
        double predictedDy = previousVy * dt;
        double translation = Math.hypot(dxMeters, dyMeters);
        double modelMismatch = Math.hypot(dxMeters - predictedDx, dyMeters - predictedDy)
                / (translation + 0.01);
        double curvature = Math.abs(dHeadingRad) / (translation + 0.02);

        double slipIndicator = clamp01(0.65 * modelMismatch + 0.35 * Math.min(1.0, curvature / 1.2));
        lastSlipIndicator = slipIndicator;

        // State update from odometry increment + velocity state refresh
        x.set(IDX_X, 0, x.get(IDX_X, 0) + dxMeters);
        x.set(IDX_Y, 0, x.get(IDX_Y, 0) + dyMeters);
        x.set(IDX_HEADING, 0, normalizeAngle(x.get(IDX_HEADING, 0) + dHeadingRad));
        x.set(IDX_VX, 0, updatedVx);
        x.set(IDX_VY, 0, updatedVy);
        x.set(IDX_OMEGA, 0, updatedOmega);

        Matrix F = buildStateTransition(dt);
        Matrix adaptiveQ = baseQ.times(1.0 + slipProcessNoiseGain * slipIndicator);
        P = F.times(P).times(F.transpose()).plus(adaptiveQ);

        // Ensure covariance symmetry
        P = P.plus(P.transpose()).times(0.5);

        lastPredictTimeNanos = System.nanoTime();
    }

    /**
     * Update step using an AprilTag-derived pose measurement in map frame (meters, radians).
     *
     * <p>The measurement is typically obtained from
     * {@link Transformation#getRobotPoseInMap(int, Matrix)}.
     *
     * @param measuredXMeters    measured X in map frame (meters)
     * @param measuredYMeters    measured Y in map frame (meters)
     * @param measuredHeadingRad measured heading in map frame (radians)
     */
    public boolean updateVision(double measuredXMeters, double measuredYMeters, double measuredHeadingRad) {
        return updateVision(measuredXMeters, measuredYMeters, measuredHeadingRad, Double.NaN);
    }

    /**
     * Vision update with tag-distance weighted covariance.
     *
     * @param measuredXMeters    measured X in map frame (meters)
     * @param measuredYMeters    measured Y in map frame (meters)
     * @param measuredHeadingRad measured heading in map frame (radians)
     * @param tagDistanceMeters  average distance from camera to observed tag(s) in meters
     */
    public boolean updateVision(double measuredXMeters,
                                double measuredYMeters,
                                double measuredHeadingRad,
                                double tagDistanceMeters) {
        if (!initialized) {
            lastVisionAccepted = false;
            lastVisionMahalanobisDistance = Double.NaN;
            lastVisionDistanceWeight = 1.0;
            return false;
        }

        Matrix z = new Matrix(new double[]{measuredXMeters, measuredYMeters,
                normalizeAngle(measuredHeadingRad)}, 3);

        Matrix H = buildVisionObservationModel();

        Matrix hx = H.times(x);
        Matrix innovation = z.minus(hx);
        innovation.set(2, 0, normalizeAngle(innovation.get(2, 0)));

        double distanceWeight = 1.0;
        if (isFinite(tagDistanceMeters) && tagDistanceMeters > 0.0) {
            distanceWeight = 1.0 + visionDistanceWeightGain * tagDistanceMeters * tagDistanceMeters;
        }
        lastVisionDistanceWeight = distanceWeight;

        Matrix R = baseRVision.times(distanceWeight);

        Matrix S = H.times(P).times(H.transpose()).plus(R);

        Matrix SInv = S.inverse();

        double maha2 = innovation.transpose().times(SInv).times(innovation).get(0, 0);
        lastVisionMahalanobisDistance = maha2;

        if (enableVisionGating && maha2 > visionMahalanobisGate) {
            lastVisionAccepted = false;
            return false;
        }

        Matrix K = P.times(H.transpose()).times(SInv);

        x = x.plus(K.times(innovation));
        x.set(IDX_HEADING, 0, normalizeAngle(x.get(IDX_HEADING, 0)));

        Matrix IminusKH = I_STATE.minus(K.times(H));
        P = IminusKH.times(P);
        P = P.plus(P.transpose()).times(0.5);

        lastVisionAccepted = true;
        return true;
    }

    /**
     * Update using a {@link Transformation.RobotPose} from the AprilTag pipeline.
     *
     * @param robotPose pose computed by {@link Transformation#getRobotPoseInMap}
     */
    public boolean updateVision(Transformation.RobotPose robotPose) {
        return updateVision(robotPose, Double.NaN);
    }

    /**
     * Vision update from Transformation pose with optional tag distance.
     */
    public boolean updateVision(Transformation.RobotPose robotPose, double tagDistanceMeters) {
        if (robotPose == null) {
            lastVisionAccepted = false;
            lastVisionMahalanobisDistance = Double.NaN;
            lastVisionDistanceWeight = 1.0;
            return false;
        }

        double xM = robotPose.translation[0];
        double yM = robotPose.translation[1];
        double heading = Math.atan2(
                robotPose.rotationMatrix[1][0],
                robotPose.rotationMatrix[0][0]);

        return updateVision(xM, yM, heading, tagDistanceMeters);
    }

    /**
     * IMU yaw-rate update channel (rad/s).
     */
    public void updateImuYawRate(double yawRateRadPerSec) {
        if (!initialized || !isFinite(yawRateRadPerSec)) {
            return;
        }

        // IMU measurement model: yawRate_measured = omega + imuBias
        Matrix H = new Matrix(new double[][]{{0, 0, 0, 0, 0, 1, 1}});
        Matrix z = new Matrix(new double[][]{{yawRateRadPerSec}});

        Matrix innovation = z.minus(H.times(x));
        Matrix S = H.times(P).times(H.transpose()).plus(baseRImuYawRate);
        Matrix K = P.times(H.transpose()).times(S.inverse());

        x = x.plus(K.times(innovation));
        x.set(IDX_HEADING, 0, normalizeAngle(x.get(IDX_HEADING, 0)));

        Matrix IminusKH = I_STATE.minus(K.times(H));
        P = IminusKH.times(P);
        P = P.plus(P.transpose()).times(0.5);
    }

    /** Immutable fused pose output from the filter in map frame (meters, radians). */
    public static class PoseEstimate {
        public final double xMeters;
        public final double yMeters;
        public final double headingRad;
        public final double vxMetersPerSec;
        public final double vyMetersPerSec;
        public final double omegaRadPerSec;
        public final double imuBiasRadPerSec;

        public PoseEstimate(double xMeters,
                            double yMeters,
                            double headingRad,
                            double vxMetersPerSec,
                            double vyMetersPerSec,
                            double omegaRadPerSec,
                            double imuBiasRadPerSec) {
            this.xMeters = xMeters;
            this.yMeters = yMeters;
            this.headingRad = headingRad;
            this.vxMetersPerSec = vxMetersPerSec;
            this.vyMetersPerSec = vyMetersPerSec;
            this.omegaRadPerSec = omegaRadPerSec;
            this.imuBiasRadPerSec = imuBiasRadPerSec;
        }

        public double headingDeg() {
            return Math.toDegrees(headingRad);
        }

        public double omegaDegPerSec() {
            return Math.toDegrees(omegaRadPerSec);
        }

        public double imuBiasDegPerSec() {
            return Math.toDegrees(imuBiasRadPerSec);
        }
    }

    /**
     * Single fused pose output function.
     *
     * @return pose in map frame (x/y meters, heading radians)
     */
    public PoseEstimate getPose() {
        return new PoseEstimate(
                x.get(IDX_X, 0),
                x.get(IDX_Y, 0),
                x.get(IDX_HEADING, 0),
                x.get(IDX_VX, 0),
                x.get(IDX_VY, 0),
                x.get(IDX_OMEGA, 0),
                x.get(IDX_IMU_BIAS, 0));
    }

    /** State covariance matrix (7x7). */
    public Matrix getCovariance() {
        return P.copy();
    }

    /** Position uncertainty (1-sigma) in meters: sqrt(Pxx + Pyy). */
    public double getPositionUncertainty() {
        return Math.sqrt(P.get(IDX_X, IDX_X) + P.get(IDX_Y, IDX_Y));
    }

    /** Heading uncertainty (1-sigma) in radians: sqrt(Ptheta,theta). */
    public double getHeadingUncertainty() {
        return Math.sqrt(P.get(IDX_HEADING, IDX_HEADING));
    }

    /** IMU yaw-rate bias uncertainty (1-sigma) in rad/s. */
    public double getImuBiasUncertainty() {
        return Math.sqrt(P.get(IDX_IMU_BIAS, IDX_IMU_BIAS));
    }

    /** Whether the filter has been initialized. */
    public boolean isInitialized() {
        return initialized;
    }

    /** Whether the last vision update was accepted (after optional gating). */
    public boolean wasLastVisionAccepted() {
        return lastVisionAccepted;
    }

    /** Mahalanobis distance^2 from the last vision innovation (lower is better). */
    public double getLastVisionMahalanobisDistance() {
        return lastVisionMahalanobisDistance;
    }

    /** Vision covariance distance weight applied on latest update attempt. */
    public double getLastVisionDistanceWeight() {
        return lastVisionDistanceWeight;
    }

    /** Current vision gate threshold in Mahalanobis distance^2. */
    public double getVisionMahalanobisGate() {
        return visionMahalanobisGate;
    }

    /** Latest slip heuristic value in [0, 1]. */
    public double getLastSlipIndicator() {
        return lastSlipIndicator;
    }

    /**
     * Normalize angle to [-pi, pi].
     */
    public static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private static Matrix makeDiagonal3(double a, double b, double c) {
        return new Matrix(new double[][]{
                {a * a, 0, 0},
                {0, b * b, 0},
                {0, 0, c * c}
        });
    }

    private static Matrix makeDiagonal7(double a, double b, double c, double d, double e, double f, double g) {
        return new Matrix(new double[][]{
                {a * a, 0, 0, 0, 0, 0, 0},
                {0, b * b, 0, 0, 0, 0, 0},
                {0, 0, c * c, 0, 0, 0, 0},
                {0, 0, 0, d * d, 0, 0, 0},
                {0, 0, 0, 0, e * e, 0, 0},
                {0, 0, 0, 0, 0, f * f, 0},
                {0, 0, 0, 0, 0, 0, g * g}
        });
    }

    private Matrix buildStateTransition(double dt) {
        Matrix F = Matrix.identity(STATE_SIZE, STATE_SIZE);
        F.set(IDX_X, IDX_VX, dt);
        F.set(IDX_Y, IDX_VY, dt);
        F.set(IDX_HEADING, IDX_OMEGA, dt);
        return F;
    }

    private Matrix buildVisionObservationModel() {
        return new Matrix(new double[][]{
                {1, 0, 0, 0, 0, 0, 0},
                {0, 1, 0, 0, 0, 0, 0},
                {0, 0, 1, 0, 0, 0, 0}
        });
    }

    private double computeDtSeconds() {
        if (lastPredictTimeNanos == 0) {
            return DEFAULT_DT_SECONDS;
        }
        double dt = (System.nanoTime() - lastPredictTimeNanos) / 1_000_000_000.0;
        if (!isFinite(dt) || dt <= 0.0) {
            return DEFAULT_DT_SECONDS;
        }
        return Math.max(MIN_DT_SECONDS, Math.min(MAX_DT_SECONDS, dt));
    }

    private static double blend(double previous, double measurement, double alpha) {
        return (1.0 - alpha) * previous + alpha * measurement;
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    private static double clamp01(double value) {
        if (value < 0.0) return 0.0;
        if (value > 1.0) return 1.0;
        return value;
    }
}
