package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.path.BluePath;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

@Autonomous(name = "AutoPathing_BlueFar")
public class Auto_BlueFar extends LinearOpMode {
    boolean PathGrabShoot1 = true;
    boolean PathGrabShoot2 = true;
    boolean PathGrabShoot3 = true;
    private BluePath bluePath;
    private Shooter shooter;
    private Intake intake;
    private boolean setPath = false;
    private enum FarPathState {none, FarScorePreload, FarGet_Ball1, FarShoot_Ball1, FarGet_Ball2, FarShoot_Ball2, FarGet_Ball3, FarShoot_Ball3, FarEndPath, finish}

    ;
    private FarPathState currentFarPathState = FarPathState.none;
    boolean OpmodeTimer = false;
    private Timer opmodeTimer;

    private void determinePath(int currentPath) {
        if (opmodeTimer.getElapsedTime() >= 25000) {
            currentFarPathState = FarPathState.FarEndPath;
        }
        if (currentPath == 0 && PathGrabShoot1) {
            currentFarPathState = FarPathState.FarGet_Ball1;
        } else if (currentPath <= 1 && PathGrabShoot2) {
            //finish path 1
            currentFarPathState = FarPathState.FarGet_Ball2;
        }else if (currentPath <= 2 && PathGrabShoot3) {
            currentFarPathState = FarPathState.FarShoot_Ball3;
        }
        else {
            currentFarPathState = FarPathState.FarEndPath;
        }
    }

    @Override
    public void runOpMode() {
        bluePath = new BluePath(hardwareMap);
        shooter = new Shooter(hardwareMap, Shooter.Alliance.BLUE);
        intake = new Intake(hardwareMap);
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        bluePath.setFarStartPose();
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

                case FarScorePreload:
                    if (!setPath) {
                        bluePath.follower.followPath(bluePath.FarScorePreload);
                        setPath = true;
                    }
                    if (!bluePath.follower.isBusy()) {
                        setPath = false;
                        determinePath(0);
                    }

                case FarGet_Ball1:
                    if (!setPath) {
                        intake.intake();
                        bluePath.follower.followPath(bluePath.FarGet_Ball1);
                        setPath = true;
                    }
                    if (!bluePath.follower.isBusy()) {
                        setPath = false;
                        currentFarPathState = FarPathState.FarShoot_Ball1;
                        intake.stop();
                    }

                case FarShoot_Ball1:
                    if(!setPath) {
                        bluePath.follower.followPath(bluePath.FarShoot_Ball1);
                        setPath = true;
                    }
                    if (!bluePath.follower.isBusy()) {
                        setPath = false;
                        determinePath(1);
                    }

                case FarGet_Ball2:
                    if (!setPath){
                        bluePath.follower.followPath(bluePath.FarGet_Ball2);
                        setPath = true;
                    }
                    if (!bluePath.follower.isBusy()) {
                        currentFarPathState = FarPathState.FarShoot_Ball2;
                        intake.intake();
                        setPath = false;
                    }

                case FarShoot_Ball2:
                    if (!setPath) {
                        bluePath.follower.followPath(bluePath.FarShoot_Ball2);
                        setPath = true;
                    }
                    if (!bluePath.follower.isBusy()) {
                        determinePath(2);
                        setPath = false;
                        intake.stop();
                    }

                case FarGet_Ball3:
                    if (!setPath) {
                        bluePath.follower.followPath(bluePath.FarGet_Ball3);
                        setPath = true;
                    }
                    if (!bluePath.follower.isBusy()) {
                        currentFarPathState = FarPathState.FarShoot_Ball3;
                        intake.intake();
                        setPath = false;
                    }

                case FarShoot_Ball3:
                    if (!setPath) {
                        bluePath.follower.followPath(bluePath.FarShoot_Ball3);
                        setPath = true;
                    }
                    if (!bluePath.follower.isBusy()) {
                        determinePath(3);
                        intake.stop();
                        setPath = false;
                    }

                case FarEndPath:
                    if(!setPath) {
                        bluePath.follower.followPath(bluePath.FarEndPath);
                        setPath = true;
                    }
                    if (!bluePath.follower.isBusy()) {
                        currentFarPathState = FarPathState.finish;
                        setPath = false;
                    }

                case finish:
                    if (gamepad1.triangle) {
                        currentFarPathState = FarPathState.none;
                        //restart the path again
                    }
            }
            bluePath.update();
            shooter.update(new Pose(85, 97.67874794069192), Math.toRadians(35));


            telemetry.addData("FarPathState", currentFarPathState);
            telemetry.addData("x", bluePath.follower.getPose().getX());
            telemetry.addData("y", bluePath.follower.getPose().getY());
            telemetry.addData("heading", bluePath.follower.getPose().getHeading());
//            telemetry.addData("running path",bluePath.follower.)
            telemetry.update();
        }
    }
}