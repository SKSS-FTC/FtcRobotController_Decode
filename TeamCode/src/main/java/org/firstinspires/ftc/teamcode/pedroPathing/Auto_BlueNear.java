package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.path.BluePath;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

@Autonomous(name = "AutoPathing_BlueNear")
public class Auto_BlueNear extends LinearOpMode {
    boolean PathGrabShoot1 = true;
    boolean PathGrabShoot2 = true;
    boolean PathGrabShoot3 = true;
    private BluePath bluePath;
    private Shooter shooter;
    private Intake intake;
    private boolean setPath = false;
    private enum NearPathState {none, NearScorePreload, NearGet_Ball1, NearShoot_Ball1, NearGet_Ball2, NearShoot_Ball2, NearGet_Ball3, NearShoot_Ball3, NearEndPath, finish}

    ;
    private NearPathState currentNearPathState = NearPathState.none;
    boolean OpmodeTimer = false;
    private Timer opmodeTimer;

    private void determinePath(int currentPath) {
        if (opmodeTimer.getElapsedTime() >= 25000) {
            currentNearPathState = NearPathState.NearEndPath;
        }
        if (currentPath == 0 && PathGrabShoot1) {
            currentNearPathState = NearPathState.NearGet_Ball1;
        } else if (currentPath <= 1 && PathGrabShoot2) {
            //finish path 1
            currentNearPathState = NearPathState.NearGet_Ball2;
        }else if (currentPath <= 2 && PathGrabShoot3) {
            currentNearPathState = NearPathState.NearShoot_Ball3;
        }
        else {
            currentNearPathState = NearPathState.NearEndPath;
        }
    }

    @Override
    public void runOpMode() {
        bluePath = new BluePath(hardwareMap);
        shooter = new Shooter(hardwareMap, Shooter.Alliance.BLUE);
        intake = new Intake(hardwareMap);
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        bluePath.setNearStartPose();
        waitForStart();
        while (opModeIsActive()) {
            if (!OpmodeTimer) {
                opmodeTimer.resetTimer();
                OpmodeTimer = true;
            }
            switch (currentNearPathState) {
                case none:
                    currentNearPathState = NearPathState.NearScorePreload;

                case NearScorePreload:
                    if (!setPath) {
                        bluePath.follower.followPath(bluePath.NearScorePreload);
                        setPath = true;
                    }
                    if (!bluePath.follower.isBusy()) {
                        setPath = false;
                        determinePath(0);
                    }

                case NearGet_Ball1:
                    if (!setPath) {
                        intake.intake();
                        bluePath.follower.followPath(bluePath.NearGet_Ball1);
                        setPath = true;
                    }
                    if (!bluePath.follower.isBusy()) {
                        setPath = false;
                        currentNearPathState = NearPathState.NearShoot_Ball1;
                        intake.stop();
                    }

                case NearShoot_Ball1:
                    if(!setPath) {
                        bluePath.follower.followPath(bluePath.NearShoot_Ball1);
                        setPath = true;
                    }
                    if (!bluePath.follower.isBusy()) {
                        setPath = false;
                        determinePath(1);
                    }

                case NearGet_Ball2:
                    if (!setPath){
                        bluePath.follower.followPath(bluePath.NearGet_Ball2);
                        setPath = true;
                    }
                    if (!bluePath.follower.isBusy()) {
                        currentNearPathState = NearPathState.NearShoot_Ball2;
                        intake.intake();
                        setPath = false;
                    }

                case NearShoot_Ball2:
                    if (!setPath) {
                        bluePath.follower.followPath(bluePath.NearShoot_Ball2);
                        setPath = true;
                    }
                    if (!bluePath.follower.isBusy()) {
                        determinePath(2);
                        setPath = false;
                        intake.stop();
                    }

                case NearGet_Ball3:
                    if (!setPath) {
                        bluePath.follower.followPath(bluePath.NearGet_Ball3);
                        setPath = true;
                    }
                    if (!bluePath.follower.isBusy()) {
                        currentNearPathState = NearPathState.NearShoot_Ball3;
                        intake.intake();
                        setPath = false;
                    }

                case NearShoot_Ball3:
                    if (!setPath) {
                        bluePath.follower.followPath(bluePath.NearShoot_Ball3);
                        setPath = true;
                    }
                    if (!bluePath.follower.isBusy()) {
                        determinePath(3);
                        intake.stop();
                        setPath = false;
                    }

                case NearEndPath:
                    if(!setPath) {
                        bluePath.follower.followPath(bluePath.NearEndPath);
                        setPath = true;
                    }
                    if (!bluePath.follower.isBusy()) {
                        currentNearPathState = NearPathState.finish;
                        setPath = false;
                    }

                case finish:
                    if (gamepad1.triangle) {
                        currentNearPathState = NearPathState.none;
                        //restart the path again
                    }
            }
            bluePath.update();
            shooter.update(new Pose(85, 97.67874794069192), Math.toRadians(35));


            telemetry.addData("NearPathState", currentNearPathState);
            telemetry.addData("x", bluePath.follower.getPose().getX());
            telemetry.addData("y", bluePath.follower.getPose().getY());
            telemetry.addData("heading", bluePath.follower.getPose().getHeading());
//            telemetry.addData("running path",bluePath.follower.)
            telemetry.update();
        }
    }
}