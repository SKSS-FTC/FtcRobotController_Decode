package org.firstinspires.ftc.teamcode.pedroPathing;

import android.sax.EndElementListener;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08,0,0.02,0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0, 0.2, 0));
    public static PathConstraints pathConstraints = new PathConstraints(0.99,   100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightUp")
            .rightRearMotorName("rightDown")
            .leftRearMotorName("leftDown")
            .leftFrontMotorName("leftUp")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(0.0029299753744086066)
            .strafeTicksToInches(0.00318307432688396)
            .turnTicksToInches(0.003168347588783974)
            .leftPodY(2.25)
            .rightPodY(-2.25)
            .strafePodX(-6.25)
            .leftEncoder_HardwareMapName("leftUp")
            .rightEncoder_HardwareMapName("rightUp")
            .strafeEncoder_HardwareMapName("leftDown")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD);
//            .IMU_HardwareMapName("imu")
//            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));

}
