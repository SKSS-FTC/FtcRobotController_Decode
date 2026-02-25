package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.path.RedPath;

@Autonomous(name = "AutoPathing_RedNear")
public class Auto_RedNear extends LinearOpMode {
    boolean PathGrabShoot1 = true;
    boolean PathGrabShoot2 = true;
    boolean PathGrabShoot3 = true;
    private RedPath redPath;
    private enum NearPathState {none,NearScorePreload,NearGet_Ball1, NearShoot_Ball1, NearGet_Ball2, NearShoot_Ball2, NearGet_Ball3, NearShoot_Ball3, NearEndPath, finish};
    private NearPathState currentNearPathState = NearPathState.none;
    boolean OpmodeTimer = false;
    private Timer opmodeTimer;

    private void determinePath(int currentPath) {
        if (opmodeTimer.getElapsedTime() >= 25000) {
            currentNearPathState = NearPathState.NearEndPath;
        if (currentPath == 0) {
            if (PathGrabShoot1) {
                currentNearPathState = NearPathState.NearGet_Ball1;
            }
            }
        }
        if (currentPath <= 1) {
            //finish path 1
            if (PathGrabShoot2) {
                currentNearPathState = NearPathState.NearGet_Ball2;
            }
        }
        if (currentPath <= 2) {
            if (PathGrabShoot3) {
                currentNearPathState = NearPathState.NearShoot_Ball3;
            }
            else {
                currentNearPathState = NearPathState.finish;
            }
        }
    }

    @Override
    public void runOpMode() {
        redPath = new RedPath(hardwareMap);
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        waitForStart();
        while(opModeIsActive()) {
            if (!OpmodeTimer) {
                opmodeTimer.resetTimer();
                OpmodeTimer = true;
            }
            switch (currentNearPathState) {
                case none:
                    currentNearPathState = NearPathState.NearScorePreload;

                case NearScorePreload:
                    redPath.follower.followPath(redPath.NearScorePreload);
                    if (!redPath.follower.isBusy()) {
                        determinePath(0);
                    }

                case NearGet_Ball1:
                    redPath.follower.followPath(redPath.NearGet_Ball1);
                    if (!redPath.follower.isBusy()) {
                        currentNearPathState = NearPathState.NearShoot_Ball1;
                    }

                case NearShoot_Ball1:
                    redPath.follower.followPath(redPath.NearShoot_Ball1);
                    if (!redPath.follower.isBusy()) {
                        determinePath(1);
                    }

                case NearGet_Ball2:
                    redPath.follower.followPath(redPath.NearGet_Ball2);
                    if (!redPath.follower.isBusy()) {
                        currentNearPathState = NearPathState.NearShoot_Ball2;
                    }

                case NearShoot_Ball2:
                    redPath.follower.followPath(redPath.NearShoot_Ball2);
                    if (!redPath.follower.isBusy()) {
                        determinePath(2);
                    }

                case NearGet_Ball3:
                    redPath.follower.followPath(redPath.NearGet_Ball3);
                    if (!redPath.follower.isBusy()) {
                        currentNearPathState = NearPathState.NearShoot_Ball3;
                    }

                case NearShoot_Ball3:
                    redPath.follower.followPath(redPath.NearShoot_Ball3);
                    if (!redPath.follower.isBusy()) {
                        determinePath(3);
                    }

                case NearEndPath:
                    redPath.follower.followPath(redPath.NearEndPath);
                    if (!redPath.follower.isBusy()) {
                        currentNearPathState = NearPathState.finish;
                    }

                case finish:
                    if (gamepad1.triangle) {
                        currentNearPathState = NearPathState.none;
                        //restart the path again
                    }
            }
            redPath.follower.update();


            telemetry.addData("NearPathState", currentNearPathState);
            telemetry.addData("x", redPath.follower.getPose().getX());
            telemetry.addData("y", redPath.follower.getPose().getY());
            telemetry.addData("heading", redPath.follower.getPose().getHeading());
            telemetry.update();
        }
    }
}