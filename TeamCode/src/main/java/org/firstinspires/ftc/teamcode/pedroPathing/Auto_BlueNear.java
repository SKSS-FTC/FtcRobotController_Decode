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
    private boolean setPath = false;
    private Shooter shooter;
    private BluePath bluePath;
    private Intake intake;
    private enum NearPathState {none,NearScorePreload,NearGet_Ball1, NearShoot_Ball1, NearGet_Ball2, NearShoot_Ball2, NearGet_Ball3, NearShoot_Ball3, NearEndPath, finish};
    private NearPathState currentNearPathState = NearPathState.none;
    boolean OpmodeTimer = false;
    private Timer opmodeTimer;

    private void determinePath(int currentPath) {
        if (opmodeTimer.getElapsedTime() >= 25000) {
            currentNearPathState = Auto_BlueNear.NearPathState.NearEndPath;
        }
        if (currentPath == 0 && PathGrabShoot1) {
            currentNearPathState = Auto_BlueNear.NearPathState.NearGet_Ball1;
        } else if (currentPath <= 1 && PathGrabShoot2) {
            //finish path 1
            currentNearPathState = Auto_BlueNear.NearPathState.NearGet_Ball2;
        }else if (currentPath <= 2 && PathGrabShoot3) {
            currentNearPathState = Auto_BlueNear.NearPathState.NearShoot_Ball3;
        }
        else {
            currentNearPathState = Auto_BlueNear.NearPathState.NearEndPath;
        }
    }

    @Override
    public void runOpMode() {
        bluePath = new BluePath(hardwareMap);
        shooter = new Shooter(hardwareMap, Shooter.Alliance.RED);
        intake = new Intake(hardwareMap);
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        waitForStart();
        while(opModeIsActive()) {
            shooter.shooterAiming = true;
            if (!OpmodeTimer) {
                opmodeTimer.resetTimer();
                OpmodeTimer = true;
            }
            switch (currentNearPathState) {
                case none:
                    currentNearPathState = NearPathState.NearScorePreload;
                    setPath = true;

                case NearScorePreload:
                    if (setPath) {
                        redPath.follower.followPath(redPath.NearScorePreload);
                        setPath = false;
                    }
                    if (!redPath.follower.isBusy()) {
                        determinePath(0);
                        setPath = true;
                    }

                case NearGet_Ball1:
                    if (setPath) {
                        redPath.follower.followPath(redPath.NearGet_Ball1);
                        setPath = false;
                    }
                    if (!redPath.follower.isBusy()) {
                        currentNearPathState = NearPathState.NearShoot_Ball1;
                        setPath = true;
                    }

                case NearShoot_Ball1:
                    if (setPath) {
                        redPath.follower.followPath(redPath.NearShoot_Ball1);
                        setPath = false;
                    }
                    if (!redPath.follower.isBusy()) {
                        determinePath(1);
                        setPath = true;
                    }

                case NearGet_Ball2:
                    if (setPath) {
                        redPath.follower.followPath(redPath.NearGet_Ball2);
                        setPath = false;
                    }
                    if (!redPath.follower.isBusy()) {
                        currentNearPathState = NearPathState.NearShoot_Ball2;
                        setPath = true;
                    }

                case NearShoot_Ball2:
                    if (setPath) {
                        redPath.follower.followPath(redPath.NearShoot_Ball2);
                        setPath = false;
                    }
                    if (!redPath.follower.isBusy()) {
                        determinePath(2);
                        setPath = true;
                    }

                case NearGet_Ball3:
                    if (setPath) {
                        redPath.follower.followPath(redPath.NearGet_Ball3);
                        setPath = false;
                    }
                    if (!redPath.follower.isBusy()) {
                        currentNearPathState = NearPathState.NearShoot_Ball3;
                        setPath = true;
                    }

                case NearShoot_Ball3:
                    if (setPath) {
                        redPath.follower.followPath(redPath.NearShoot_Ball3);
                        setPath = false;
                    }
                    if (!redPath.follower.isBusy()) {
                        determinePath(3);
                        setPath = true;
                    }

                case NearEndPath:
                    if (setPath) {
                        redPath.follower.followPath(redPath.NearEndPath);
                        setPath = false;
                    }
                    if (!redPath.follower.isBusy()) {
                        currentNearPathState = NearPathState.finish;
                    }

                case finish:
                    if (gamepad1.triangle) {
                        currentNearPathState = NearPathState.none;
                        setPath = true;
                    }
            }
            redPath.follower.update();
            shooter.update(new Pose(88,8),Math.toRadians(75));


            telemetry.addData("NearPathState", currentNearPathState);
            telemetry.addData("x", redPath.follower.getPose().getX());
            telemetry.addData("y", redPath.follower.getPose().getY());
            telemetry.addData("heading", redPath.follower.getPose().getHeading());
            telemetry.update();
        }
    }
}