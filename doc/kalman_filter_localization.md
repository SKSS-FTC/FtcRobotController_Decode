# Kalman Filter Localization (PedroPathing + AprilTag)

## Overview

This project uses a 7-state Kalman filter to fuse:

1. **Odometry prediction** from 3-wheel + IMU (via PedroPathing)
2. **Absolute correction** from AprilTag camera localization

The output is a smoother, drift-resistant robot pose estimate in the **map frame** (origin at the left-bottom corner of the map).

- State units in filter: **meters**, **radians**, **m/s**, **rad/s**
- PedroPathing inputs: **inches** and **radians** (converted internally)
- AprilTag pose from `Transformation`: **meters** and **radians**

---

## Why this implementation (and package choice)

We already depend on **JAMA** (`gov.nist.math:jama`) for matrix math in this codebase, and the current filter needs only small dense matrices (3x3, 3x1). That makes JAMA a good fit:

- No additional runtime complexity
- Consistent with existing `Transformation` subsystem
- Easy to inspect and tune

So the filter is implemented directly in:

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/KalmanFilter.java`

This implementation now includes:

- 7-state model: $[x, y, \theta, v_x, v_y, \omega, b_{imu}]$
- Tag-distance weighted vision covariance
- IMU yaw-rate measurement channel
- IMU yaw-rate bias estimation
- Adaptive process noise scaling for slip events

---

## System model used in this code

## State vector

$$
\mathbf{x}_k =
\begin{bmatrix}
 x_k \\
 y_k \\
 	heta_k \\
 v_{x,k} \\
 v_{y,k} \\
 \omega_k \\
 b_{imu,k}
\end{bmatrix}
$$

where:
- $x, y$ are robot position in map frame (meters)
- $\theta$ is heading in map frame (radians)
- $v_x, v_y$ are planar velocities (m/s)
- $\omega$ is yaw-rate (rad/s)
- $b_{imu}$ is IMU yaw-rate bias (rad/s)

## Motion model (predict)

Odometry provides incremental motion between loops (already in meters/radians in caller):

$$
\Delta \mathbf{u}_k =
\begin{bmatrix}
 \Delta x_k \\
 \Delta y_k \\
 \Delta \theta_k
\end{bmatrix}
$$

Predict step used in code:

$$
x \leftarrow x + \Delta x,\quad y \leftarrow y + \Delta y,\quad \theta \leftarrow \theta + \Delta\theta
$$

Velocity states are refreshed from odometry-derived rates with a blend factor:

$$
v_x \leftarrow (1-\alpha)v_x + \alpha\frac{\Delta x}{\Delta t},\quad
v_y \leftarrow (1-\alpha)v_y + \alpha\frac{\Delta y}{\Delta t},\quad
\omega \leftarrow (1-\alpha)\omega + \alpha\frac{\Delta\theta}{\Delta t}
$$

$$
\mathbf{P}_{k|k-1} = \mathbf{F}\mathbf{P}_{k-1|k-1}\mathbf{F}^T + \mathbf{Q}_{\text{adaptive}}
$$

with

$$
\mathbf{F} =
\begin{bmatrix}
1&0&0&\Delta t&0&0\\
0&1&0&0&\Delta t&0\\
0&0&1&0&0&\Delta t\\
0&0&0&1&0&0\\
0&0&0&0&1&0\\
0&0&0&0&0&1
\end{bmatrix}
$$

and adaptive process scaling:

$$
\mathbf{Q}_{\text{adaptive}} = \mathbf{Q}_{\text{base}}\left(1 + g_{slip}\,s\right),\quad s\in[0,1]
$$

## Measurement model (update)

AprilTag subsystem provides a direct pose measurement of position and heading:

$$
\mathbf{z}_k =
\begin{bmatrix}
 x^{tag}_k \\
 y^{tag}_k \\
 \theta^{tag}_k
\end{bmatrix}
$$

with observation matrix

$$
\mathbf{H}_{vision} =
\begin{bmatrix}
1&0&0&0&0&0\\
0&1&0&0&0&0\\
0&0&1&0&0&0
\end{bmatrix}
$$

Innovation:

$$
\mathbf{y}_k = \mathbf{z}_k - \mathbf{x}_{k|k-1}
$$

(heading innovation is angle-normalized to $[-\pi, \pi]$).

Innovation covariance:

$$
\mathbf{S}_k = \mathbf{H}\mathbf{P}_{k|k-1}\mathbf{H}^T + \mathbf{R}_{vision,weighted}
$$

Kalman gain:

$$
\mathbf{K}_k = \mathbf{P}_{k|k-1}\mathbf{H}^T\mathbf{S}_k^{-1}
$$

State/covariance update:

$$
\mathbf{x}_{k|k} = \mathbf{x}_{k|k-1} + \mathbf{K}_k\mathbf{y}_k
$$

$$
\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k\mathbf{H})\mathbf{P}_{k|k-1}
$$

### Tag-distance weighted covariance (implemented)

The vision measurement covariance is scaled with observed tag distance:

$$
w_d = 1 + g_d\,d^2
$$

$$
\mathbf{R}_{vision,weighted} = w_d\,\mathbf{R}_{vision,base}
$$

where $d$ is average tag range in meters.

### Outlier rejection (implemented)

Before applying a vision correction, the filter computes the innovation Mahalanobis distance:

$$
d_k^2 = \mathbf{y}_k^T\mathbf{S}_k^{-1}\mathbf{y}_k
$$

If gating is enabled and $d_k^2$ is larger than a threshold $\gamma$, the vision update is rejected for that cycle:

$$
d_k^2 > \gamma \Rightarrow \text{reject measurement}
$$

Current default in code: $\gamma = 11.34$ (approximately 99% chi-square gate for 3 DoF).

### IMU yaw-rate update channel (implemented)

IMU contributes a 1D yaw-rate measurement:

$$
z_{imu} = \omega + b_{imu} + v
$$

with

$$
\mathbf{H}_{imu} = \begin{bmatrix}0&0&0&0&0&1&1\end{bmatrix}
$$

This lets the filter separate true yaw-rate from IMU bias drift. Over time, $b_{imu}$ is learned,
which reduces heading drift and prevents steady offset from contaminating $\omega$.

---

## How this maps to your code

## 1) Filter subsystem

File:
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/KalmanFilter.java`

Important methods:

- `init(...)`
  - Sets initial state
- `predict(...)`
  - Uses odometry deltas each loop
- `updateVision(...)` and `updateVision(..., tagDistanceMeters)`
  - Corrects estimate when AprilTag pose is available
- `updateImuYawRate(...)`
   - Fuses IMU yaw-rate channel
- `getPose()`
   - Single fused output: $x, y, \theta, v_x, v_y, \omega, b_{imu}$
- `normalizeAngle(...)`
  - Keeps heading in valid wrapped range

Noise matrices:

- $\mathbf{Q}$: process noise (odometry drift model)
- $\mathbf{R}$: vision measurement noise
- $\mathbf{P}$: current state covariance (uncertainty)

Vision gating helpers:

- `setVisionMahalanobisGate(...)`
- `setEnableVisionGating(...)`
- `setVisionDistanceWeightGain(...)`
- `setSlipProcessNoiseGain(...)`
- `wasLastVisionAccepted()`
- `getLastVisionMahalanobisDistance()`
- `getLastVisionDistanceWeight()`
- `getLastSlipIndicator()`

## 2) Runtime fusion example

File:
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/examples/ExampleKalmanFilterOdometry.java`

Loop sequence:

1. Call PedroPathing update and compute odometry deltas
2. Call filter predict with those deltas
3. If AprilTags are visible, build map-frame pose from `Transformation`
4. Call filter update with that vision pose
5. Publish fused telemetry

---

## 3-second AprilTag warmup for initial pose

Before start, the example now performs a 3-second warmup window:

1. Continuously reads AprilTag detections
2. Converts detections to robot map-frame pose via `Transformation.getRobotPoseInMapFromMultipleTags(...)`
3. Averages pose samples
   - Position: arithmetic mean
   - Heading: circular mean

Circular heading mean:

$$
\bar{\theta} = \operatorname{atan2}\left(\frac{1}{N}\sum_{i=1}^{N}\sin\theta_i,
\frac{1}{N}\sum_{i=1}^{N}\cos\theta_i\right)
$$

This avoids wraparound errors near $\pm\pi$.

If no valid samples are collected, the code falls back to `STARTING_POSE`.

---

## Tuning guide

Builder parameters in `KalmanFilter.Builder`:

- `processNoiseXY`
- `processNoiseHeading`
- `processNoiseVelocity`
- `processNoiseOmega`
- `processNoiseImuBias`
- `visionNoiseXY`
- `visionNoiseHeading`
- `imuYawRateNoise`
- `initialUncertaintyXY`
- `initialUncertaintyHeading`
- `initialUncertaintyVelocity`
- `initialUncertaintyOmega`
- `initialUncertaintyImuBias`
- `visionMahalanobisGate`
- `visionDistanceWeightGain`
- `slipProcessNoiseGain`
- `velocityBlend`

Practical tuning rules:

1. **Filter lags behind odometry**
   - Increase process noise (`processNoiseXY`, `processNoiseHeading`)
2. **Filter trusts noisy tags too much**
   - Increase vision noise (`visionNoiseXY`, `visionNoiseHeading`)
   - Lower acceptance by decreasing `visionMahalanobisGate`
3. **Filter ignores good tag corrections**
   - Decrease vision noise
   - Increase `visionMahalanobisGate` if good measurements are being rejected
4. **Heading drift during poor tag visibility**
   - Decrease `imuYawRateNoise` so filter trusts IMU yaw-rate more
5. **Startup convergence is slow**
   - Reduce initial uncertainty if warmup init is already reliable

A good workflow is to tune on-field while watching:

- fused pose vs ground truth
- `positionUncertainty`
- `headingUncertainty`

---

## Coordinate and unit conventions

- Map frame origin: field left-bottom corner
- Filter state: meters/radians
- PedroPathing: inches/radians
- AprilTag transform pipeline: meters/radians

Conversion used:

$$
1\ \text{inch} = 0.0254\ \text{m}
$$

---

## Current limitations (intentional MVP)

This is a compact linear model for reliability and clarity:

- No delayed measurement handling
- No asynchronous timestamp alignment across sensors

Even with this simple model, the fusion usually outperforms odometry-only in long runs and vision-only under intermittent detection.