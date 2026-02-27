package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.path.BluePath;

@Autonomous(name = "AutoPathing_BlueFar",preselectTeleOp = "blue")
public class Auto_BlueFar extends LinearOpMode {
    boolean PathGrabShoot1 = true;
    boolean PathGrabShoot2 = true;
    boolean PathGrabShoot3 = true;
    private boolean setPath = false;
    private BluePath bluePath;
    private Shooter shooter;
    private Intake intake;
    private enum FarPathState {none,FarScorePreload,FarGet_Ball1, FarShoot_Ball1, FarGet_Ball2, FarShoot_Ball2, FarGet_Ball3, FarShoot_Ball3, FarEndPath, finish};
    private FarPathState currentFarPathState = FarPathState.none;
    boolean OpmodeTimer = false;
    private Timer opmodeTimer;

    private void determinePath(int currentPath) {
        if (opmodeTimer.getElapsedTime() >= 25000) {
            currentFarPathState = FarPathState.FarEndPath;
            if (currentPath == 0) {
                if (PathGrabShoot1) {
                    currentFarPathState = FarPathState.FarGet_Ball1;
                }
            }
        }
        if (currentPath <= 1) {
            if (PathGrabShoot2) {
                currentFarPathState = FarPathState.FarGet_Ball2;
            }
        }
        if (currentPath <= 2) {
            if (PathGrabShoot3) {
                currentFarPathState = FarPathState.FarShoot_Ball3;
            }
            else {
                currentFarPathState = FarPathState.finish;
            }
        }
    }

    @Override
    public void runOpMode() {
        bluePath = new BluePath(hardwareMap);
        shooter = new Shooter(hardwareMap, Shooter.Alliance.BLUE);
        intake = new Intake(hardwareMap);
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        waitForStart();
        while (opModeIsActive()) {
            shooter.shooterAiming = true;
            if (!OpmodeTimer) {
                opmodeTimer.resetTimer();
                OpmodeTimer = true;
            }
            switch (currentFarPathState) {
                case none:
                    currentFarPathState = FarPathState.FarScorePreload;
                    setPath = true;

                case FarScorePreload:
                    if (setPath) {
                        bluePath.follower.followPath(bluePath.FarScorePreload);
                        setPath = false;
                    }
                    if (!bluePath.follower.isBusy()) {
                        determinePath(0);
                        setPath = true;
                    }

                case FarGet_Ball1:
                    if (setPath) {
                        bluePath.follower.followPath(bluePath.FarGet_Ball1);
                        setPath = false;
                    }
                    if (!bluePath.follower.isBusy()) {
                        currentFarPathState = FarPathState.FarShoot_Ball1;
                        intake.intake();
                        setPath = true;
                    }

                case FarShoot_Ball1:
                    if (setPath) {
                        bluePath.follower.followPath(bluePath.FarShoot_Ball1);
                        setPath = false;
                    }
                    if (!bluePath.follower.isBusy()) {
                        determinePath(1);
                        intake.stop();
                        setPath = true;
                    }

                case FarGet_Ball2:
                    if (setPath) {
                        bluePath.follower.followPath(bluePath.FarGet_Ball2);
                        setPath = false;
                    }
                    if (!bluePath.follower.isBusy()) {
                        currentFarPathState = FarPathState.FarShoot_Ball2;
                        intake.intake();
                        setPath = true;
                    }

                case FarShoot_Ball2:
                    if (setPath) {
                        bluePath.follower.followPath(bluePath.FarShoot_Ball2);
                        setPath = false;
                    }
                    if (!bluePath.follower.isBusy()) {
                        determinePath(2);
                        intake.stop();
                        setPath = true;
                    }

                case FarGet_Ball3:
                    if (setPath) {
                        bluePath.follower.followPath(bluePath.FarGet_Ball3);
                        setPath = false;
                    }
                    if (!bluePath.follower.isBusy()) {
                        currentFarPathState = FarPathState.FarShoot_Ball3;
                        intake.intake();
                        setPath = true;
                    }

                case FarShoot_Ball3:
                    if (setPath) {
                        bluePath.follower.followPath(bluePath.FarShoot_Ball3);
                        setPath = false;
                    }
                    if (!bluePath.follower.isBusy()) {
                        determinePath(3);
                        intake.stop();
                        setPath = true;
                    }

                case FarEndPath:
                    if (setPath) {
                        bluePath.follower.followPath(bluePath.FarEndPath);
                        setPath = false;
                    }
                    if (!bluePath.follower.isBusy()) {
                        currentFarPathState = FarPathState.finish;
                    }

                case finish:
                    if (gamepad1.triangle) {
                        currentFarPathState = FarPathState.none;
                        setPath = true;
                    }
            }
            bluePath.follower.update();
            shooter.update(new Pose(59.24382207578253, 97.67874794069192),  Math.toRadians(140));


            telemetry.addData("PathState", currentFarPathState);
            telemetry.addData("x", bluePath.follower.getPose().getX());
            telemetry.addData("y", bluePath.follower.getPose().getY());
            telemetry.addData("heading", bluePath.follower.getPose().getHeading());
            telemetry.update();
        }
    }
}