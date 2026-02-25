package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.path.RedPath;

@Autonomous(name = "AutoPathing_RedFar")
public class Auto_RedFar extends LinearOpMode {
    boolean PathGrabShoot1 = true;
    boolean PathGrabShoot2 = true;
    boolean PathGrabShoot3 = true;
    private RedPath redPath;

    private enum FarPathState {none, FarScorePreload, FarGet_Ball1, FarShoot_Ball1, FarGet_Ball2, FarShoot_Ball2, FarGet_Ball3, FarShoot_Ball3, FarEndPath, finish}

    ;
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
            //finish path 1
            if (PathGrabShoot2) {
                currentFarPathState = FarPathState.FarGet_Ball2;
            }
        }
        if (currentPath <= 2) {
            if (PathGrabShoot3) {
                currentFarPathState = FarPathState.FarShoot_Ball3;
            } else {
                currentFarPathState = FarPathState.finish;
            }
        }
    }

    @Override
    public void runOpMode() {
        redPath = new RedPath(hardwareMap);
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        waitForStart();
        while (opModeIsActive()) {
            if (!OpmodeTimer) {
                opmodeTimer.resetTimer();
                OpmodeTimer = true;
            }
            switch (currentFarPathState) {
                case none:
                    currentFarPathState = FarPathState.FarScorePreload;

                case FarScorePreload:
                    redPath.follower.followPath(redPath.FarScorePreload);
                    if (!redPath.follower.isBusy()) {
                        determinePath(0);
                    }

                case FarGet_Ball1:
                    redPath.follower.followPath(redPath.FarGet_Ball1);
                    if (!redPath.follower.isBusy()) {
                        currentFarPathState = FarPathState.FarShoot_Ball1;
                    }

                case FarShoot_Ball1:
                    redPath.follower.followPath(redPath.FarShoot_Ball1);
                    if (!redPath.follower.isBusy()) {
                        determinePath(1);
                    }

                case FarGet_Ball2:
                    redPath.follower.followPath(redPath.FarGet_Ball2);
                    if (!redPath.follower.isBusy()) {
                        currentFarPathState = FarPathState.FarShoot_Ball2;
                    }

                case FarShoot_Ball2:
                    redPath.follower.followPath(redPath.FarShoot_Ball2);
                    if (!redPath.follower.isBusy()) {
                        determinePath(2);
                    }

                case FarGet_Ball3:
                    redPath.follower.followPath(redPath.FarGet_Ball3);
                    if (!redPath.follower.isBusy()) {
                        currentFarPathState = FarPathState.FarShoot_Ball3;
                    }

                case FarShoot_Ball3:
                    redPath.follower.followPath(redPath.FarShoot_Ball3);
                    if (!redPath.follower.isBusy()) {
                        determinePath(3);
                    }

                case FarEndPath:
                    redPath.follower.followPath(redPath.FarEndPath);
                    if (!redPath.follower.isBusy()) {
                        currentFarPathState = FarPathState.finish;
                    }

                case finish:
                    if (gamepad1.triangle) {
                        currentFarPathState = FarPathState.none;
                        //restart the path again
                    }
            }
            redPath.follower.update();


            telemetry.addData("NearPathState", currentFarPathState);
            telemetry.addData("x", redPath.follower.getPose().getX());
            telemetry.addData("y", redPath.follower.getPose().getY());
            telemetry.addData("heading", redPath.follower.getPose().getHeading());
            telemetry.update();
        }
    }
}