package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystem.path.BluePath;

@Autonomous(name = "AutoPathing_BlueNear")
public class AutoPathing_BlueNear extends OpMode {
    boolean PathGrabShoot1 = true;
    boolean PathGrabShoot2 = true;
    boolean PathGrabShoot3 = true;
    private BluePath bluePath;
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
    public void loop() {
        if (!OpmodeTimer) {
            opmodeTimer.resetTimer();
            OpmodeTimer = true;
        }
        switch (currentNearPathState) {
            case none:
                currentNearPathState = NearPathState.NearScorePreload;

            case NearScorePreload:
                bluePath.follower.followPath(bluePath.NearScorePreload);
                if (!bluePath.follower.isBusy()) {
                    determinePath(0);
                }

            case NearGet_Ball1:
                bluePath.follower.followPath(bluePath.NearGet_Ball1);
                if (!bluePath.follower.isBusy()) {
                    currentNearPathState = NearPathState.NearShoot_Ball1;
                }

            case NearShoot_Ball1:
                bluePath.follower.followPath(bluePath.NearShoot_Ball1);
                if (!bluePath.follower.isBusy()) {
                    determinePath(1);
                }

            case NearGet_Ball2:
                bluePath.follower.followPath(bluePath.NearGet_Ball2);
                if (!bluePath.follower.isBusy()) {
                    currentNearPathState = NearPathState.NearShoot_Ball2;
                }

            case NearShoot_Ball2:
                bluePath.follower.followPath(bluePath.NearShoot_Ball2);
                if (!bluePath.follower.isBusy()) {
                    determinePath(2);
                }

            case NearGet_Ball3:
                bluePath.follower.followPath(bluePath.NearGet_Ball3);
                if (!bluePath.follower.isBusy()) {
                    currentNearPathState = NearPathState.NearShoot_Ball3;
                }

            case NearShoot_Ball3:
                bluePath.follower.followPath(bluePath.NearShoot_Ball3);
                if (!bluePath.follower.isBusy()) {
                    determinePath(3);
                }

            case NearEndPath:
                bluePath.follower.followPath(bluePath.NearEndPath);
                if (!bluePath.follower.isBusy()) {
                    currentNearPathState = NearPathState.finish;
                }

            case finish:
                if (gamepad1.triangle) {
                    currentNearPathState = NearPathState.none;
                    //restart the path again
                }
        }
        bluePath.follower.update();


        telemetry.addData("NearPathState", currentNearPathState);
        telemetry.addData("x", bluePath.follower.getPose().getX());
        telemetry.addData("y", bluePath.follower.getPose().getY());
        telemetry.addData("heading", bluePath.follower.getPose().getHeading());
        telemetry.update();
    }
    @Override
    public void init() {
        bluePath = new BluePath(hardwareMap);
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
    }
}