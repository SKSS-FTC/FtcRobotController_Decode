package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.path.BluePath;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Slider;

@Autonomous(name = "AutoPathing_BlueFar")
public class Auto_BlueFar extends LinearOpMode {
    boolean PathGrabShoot1 = true;
    boolean PathGrabShoot2 = true;
    boolean PathGrabShoot3 = true;
    private BluePath bluePath;
    private Shooter shooter;
    private Intake intake;
    private Slider slider;
    private boolean setPath = false;
    private enum FarPathState {none, FarScorePreload, FarGet_Ball1, FarShoot_Ball1, FarGet_Ball2, FarShoot_Ball2, FarGet_Ball3, FarShoot_Ball3, FarEndPath, finish}

    ;
    private FarPathState currentFarPathState = FarPathState.none;
    boolean shooterTimerBoolean = false;
    boolean OpmodeTimer = false;
    private Timer opmodeTimer;
    private Timer shooterTimer;

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
        shooterTimer = new Timer();
        opmodeTimer.resetTimer();
        shooterTimer.resetTimer();
        bluePath.setFarStartPose();
        slider.drive();
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
                        shooter.shooterRotate = true;
                        shooterTimerBoolean = false;
                    }
                    if (!bluePath.follower.isBusy()) {
                        if (!shooterTimerBoolean) {
                            shooterTimerBoolean = true;
                            shooterTimer.resetTimer();
                        }
                        if (shooterTimer.getElapsedTimeSeconds() < 0.1) {
                            shooter.openGate();
                        } else if (shooterTimer.getElapsedTimeSeconds() < 0.3) {
                            intake.shoot();
                        } else if (shooterTimer.getElapsedTimeSeconds() < 0.8) {
                            intake.shoot();
                        } else if (shooterTimer.getElapsedTimeSeconds() < 1.3) {
                            intake.shoot();
                        } else {
                            setPath = false;
                            determinePath(0);
                        }
                    }

                case FarGet_Ball1:
                    if (!setPath) {
                        intake.intake();
                        bluePath.follower.followPath(bluePath.FarGet_Ball1);
                        setPath = true;
                    }
                    if (!bluePath.follower.isBusy()) {
                        intake.stop();
                        setPath = false;
                        currentFarPathState = FarPathState.FarShoot_Ball1;
                        intake.stop();
                    }

                case FarShoot_Ball1:
                    if (!setPath) {
                        bluePath.follower.followPath(bluePath.FarShoot_Ball1);
                        setPath = true;
                        shooter.shooterRotate = true;
                        shooterTimerBoolean = false;
                    }
                    if (!bluePath.follower.isBusy()) {
                        if (!shooterTimerBoolean) {
                            shooterTimerBoolean = true;
                            shooterTimer.resetTimer();
                        }
                        if (shooterTimer.getElapsedTimeSeconds() < 0.1) {
                            shooter.openGate();
                        } else if (shooterTimer.getElapsedTimeSeconds() < 0.3) {
                            intake.shoot();
                        } else if (shooterTimer.getElapsedTimeSeconds() < 0.8) {
                            intake.shoot();
                        } else if (shooterTimer.getElapsedTimeSeconds() < 1.3) {
                            intake.shoot();
                        } else {
                            setPath = false;
                            shooter.shooterRotate = false;
                            determinePath(1);
                        }
                    }


                case FarGet_Ball2:
                    if (!setPath) {
                        intake.intake();
                        bluePath.follower.followPath(bluePath.FarGet_Ball2);
                        setPath = true;
                    }
                    if (!bluePath.follower.isBusy()) {
                        currentFarPathState = FarPathState.FarShoot_Ball2;
                        intake.stop();
                        setPath = false;
                    }

                case FarShoot_Ball2:
                    if (!setPath) {
                        bluePath.follower.followPath(bluePath.FarShoot_Ball2);
                        setPath = true;
                        shooter.shooterRotate = true;
                        shooterTimerBoolean = false;
                    }
                    if (!bluePath.follower.isBusy()) {
                        if (!shooterTimerBoolean) {
                            shooterTimerBoolean = true;
                            shooterTimer.resetTimer();
                        }
                        if (shooterTimer.getElapsedTimeSeconds() < 0.1) {
                            shooter.openGate();
                        } else if (shooterTimer.getElapsedTimeSeconds() < 0.3) {
                            intake.shoot();
                        } else if (shooterTimer.getElapsedTimeSeconds() < 0.8) {
                            intake.shoot();
                        } else if (shooterTimer.getElapsedTimeSeconds() < 1.3) {
                            intake.shoot();
                        } else {
                            setPath = false;
                            shooter.shooterRotate = false;
                            determinePath(2);
                        }
                    }

                case FarGet_Ball3:
                    if (!setPath) {
                        intake.intake();
                        bluePath.follower.followPath(bluePath.FarGet_Ball3);
                        setPath = true;
                    }
                    if (!bluePath.follower.isBusy()) {
                        currentFarPathState = FarPathState.FarShoot_Ball3;
                        intake.stop();
                        setPath = false;
                    }

                case FarShoot_Ball3:
                    if (!setPath) {
                        bluePath.follower.followPath(bluePath.FarShoot_Ball3);
                        setPath = true;
                        shooter.shooterRotate = true;
                        shooterTimerBoolean = false;
                    }
                    if (!bluePath.follower.isBusy()) {
                        if (!shooterTimerBoolean){
                            shooterTimerBoolean = true;
                            shooterTimer.resetTimer();
                        }
                        if (shooterTimer.getElapsedTimeSeconds()<0.1) {
                            shooter.openGate();
                        }else if (shooterTimer.getElapsedTimeSeconds()<0.3) {
                            intake.shoot();
                        }else if (shooterTimer.getElapsedTimeSeconds()<0.8) {
                            intake.shoot();
                        }
                        else if (shooterTimer.getElapsedTimeSeconds()<1.3) {
                            intake.shoot();
                        }else{
                            setPath = false;
                            shooter.shooterRotate = false;
                            determinePath(3);
                        }
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
            }
            bluePath.update();
            shooter.update(bluePath.FarShootPose,bluePath.FarShootPose.getHeading());


            telemetry.addData("FarPathState", currentFarPathState);
            telemetry.addData("x", bluePath.follower.getPose().getX());
            telemetry.addData("y", bluePath.follower.getPose().getY());
            telemetry.addData("heading", bluePath.follower.getPose().getHeading());
//            telemetry.addData("running path",bluePath.follower.)
            telemetry.update();
        }
    }
}