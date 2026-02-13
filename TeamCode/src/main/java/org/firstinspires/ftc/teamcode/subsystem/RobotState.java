package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.geometry.Pose;

/**
 * Robot state container for mutable runtime state.
 * Separates mutable state from immutable constants.
 */
public class RobotState {
    private RobotState() {}

    /**
     * Current robot pose - updated by path_following logic
     */
    public static Pose currentPose = null;

    /**
     * Shooter encoder calibration constant
     */
    public static int shooterEncoder = 10000;
}
