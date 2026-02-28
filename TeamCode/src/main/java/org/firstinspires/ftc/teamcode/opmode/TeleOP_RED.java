package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.subsystem.RobotState.currentPose;

import android.util.Size;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.subsystem.AprilTagReader;
import org.firstinspires.ftc.teamcode.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Localizer;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Slider;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import Jama.Matrix;

@TeleOp(name = "TeleOP_RED", group = "Tests")
public class TeleOP_RED extends LinearOpMode {
    private Slider slider;
    private DriveTrain driveTrain;
    private Shooter shooter;
    private Intake intake;
    private Localizer localizer;
    private boolean wasLogging = false;
    private Pose STARTING_POSE = new Pose(0,0,0);

    @Override
    public void runOpMode() {
        slider = new Slider(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap);
        shooter = new Shooter(hardwareMap, Shooter.Alliance.RED);
//        shooter = new Shooter(hardwareMap, Shooter.Alliance.RED);
        intake = new Intake(hardwareMap);
//        if(currentPose != null){
//            STARTING_POSE = currentPose;
//        }
        localizer = new Localizer.Builder()
                .webcamName("Webcam 1")
                .startingPose(STARTING_POSE)
                .build();
        localizer.init(hardwareMap);

        intake.stop();

        telemetry.addData("Status", "Vision Portal initialized");
        telemetry.addData("Status", "AprilTag detection ready. Press START.");
        telemetry.update();

        waitForStart();
        localizer.getFollower().startTeleOpDrive();

        while (opModeIsActive()) {
            shooter.shooterAiming = false;
            localizer.update();
//            driveTrain.setFieldDrive(false);
//            driveTrain.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            // Get detections directly - VisionPortal handles frame processing automatically

            if (gamepad1.triangle) {
                shooter.shooterRotate = true;
            } else if (gamepad1.cross) {
                shooter.shooterRotate = false;
            }

            if (gamepad1.dpad_up) {
                intake.reserve();
            } else if (gamepad1.left_trigger > 0) {
                intake.intake();
                shooter.closeGate();
            } else if (gamepad1.right_trigger > 0.5) {
                shooter.openGate();
                intake.intake();
                intake.shoot();

            } else {
                intake.stop();
            }
            if(gamepad1.right_bumper){
                localizer.getFollower().setMaxPower(1);
            }else if (gamepad1.left_bumper){
                localizer.getFollower().setMaxPower(0.1);
            }else{
                localizer.getFollower().setMaxPower(0.4);
            }
//            if (gamepad1.x){
//                driveTrain.resetIMU();
//            }
//            if (gamepad1.dpad_left && gamepad1.b) {
//                slider.park();
//            } else {
//                slider.drive();
//            }
            intake.update();
            shooter.update(localizer.getPose(),-1*Math.toDegrees(localizer.getHeading()));
            telemetry.addData("kicker timer", intake.shootTimer.milliseconds());
            telemetry.addData("angular velocity", shooter.getShootVelocity());
            telemetry.addData("pose",localizer.getPose());
            telemetry.update();
            localizer.getFollower().setTeleOpDrive(gamepad1.left_stick_y*-1,gamepad1.left_stick_x*-1, gamepad1.right_stick_x*-1);
        }
    }

}
