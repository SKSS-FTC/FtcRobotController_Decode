package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.path.RedPath;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

@Autonomous(name = "AutoPathing_RedFar")
public class Auto_RedFar extends LinearOpMode {
    boolean PathGrabShoot1 = true;
    boolean PathGrabShoot2 = true;
    boolean PathGrabShoot3 = true;
    private RedPath redPath;
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
        redPath = new RedPath(hardwareMap);
        shooter = new Shooter(hardwareMap, Shooter.Alliance.RED);
        intake = new Intake(hardwareMap);
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        redPath.setFarStartPose();
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
                        redPath.follower.followPath(redPath.FarScorePreload);
                        setPath = true;
                    }
                    if (!redPath.follower.isBusy()) {
                        setPath = false;
                        determinePath(0);
                    }

                case FarGet_Ball1:
                    if (!setPath) {
                        intake.intake();
                        redPath.follower.followPath(redPath.FarGet_Ball1);
                        setPath = true;
                    }
                    if (!redPath.follower.isBusy()) {
                        setPath = false;
                        currentFarPathState = FarPathState.FarShoot_Ball1;
                        intake.stop();
                    }

                case FarShoot_Ball1:
                    if(!setPath) {
                        redPath.follower.followPath(redPath.FarShoot_Ball1);
                        setPath = true;
                    }
                    if (!redPath.follower.isBusy()) {
                        setPath = false;
                        determinePath(1);
                    }

                case FarGet_Ball2:
                    if (!setPath){
                        redPath.follower.followPath(redPath.FarGet_Ball2);
                        setPath = true;
                    }
                    if (!redPath.follower.isBusy()) {
                        currentFarPathState = FarPathState.FarShoot_Ball2;
                        intake.intake();
                        setPath = false;
                    }

                case FarShoot_Ball2:
                    if (!setPath) {
                        redPath.follower.followPath(redPath.FarShoot_Ball2);
                        setPath = true;
                    }
                    if (!redPath.follower.isBusy()) {
                        determinePath(2);
                        setPath = false;
                        intake.stop();
                    }

                case FarGet_Ball3:
                    if (!setPath) {
                        redPath.follower.followPath(redPath.FarGet_Ball3);
                        setPath = true;
                    }
                    if (!redPath.follower.isBusy()) {
                        currentFarPathState = FarPathState.FarShoot_Ball3;
                        intake.intake();
                        setPath = false;
                    }

                case FarShoot_Ball3:
                    if (!setPath) {
                        redPath.follower.followPath(redPath.FarShoot_Ball3);
                        setPath = true;
                    }
                    if (!redPath.follower.isBusy()) {
                        determinePath(3);
                        intake.stop();
                        setPath = false;
                    }

                case FarEndPath:
                    if(!setPath) {
                        redPath.follower.followPath(redPath.FarEndPath);
                        setPath = true;
                    }
                    if (!redPath.follower.isBusy()) {
                        currentFarPathState = FarPathState.finish;
                        setPath = false;
                    }

                case finish:
                    if (gamepad1.triangle) {
                        currentFarPathState = FarPathState.none;
                        //restart the path again
                    }
            }
            redPath.update();
//            shooter.update(new Pose(85, 97.67874794069192), Math.toRadians(35));


            telemetry.addData("NearPathState", currentFarPathState);
            telemetry.addData("x", redPath.follower.getPose().getX());
            telemetry.addData("y", redPath.follower.getPose().getY());
            telemetry.addData("heading", redPath.follower.getPose().getHeading());
//            telemetry.addData("running path",redPath.follower.)
            telemetry.update();
        }
    }
}