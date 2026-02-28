package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.path.BluePath;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Slider;

@Autonomous(name = "AutoPathing_BlueNear")
public class Auto_BlueNear extends LinearOpMode {
    boolean PathGrabShoot1 = true;
    boolean PathGrabShoot2 = true;
    boolean PathGrabShoot3 = true;
    private BluePath bluePath;
    private Shooter shooter;
    private Intake intake;
    private Slider slider;
    private boolean setPath = false;
    private enum NearPathState {none, NearScorePreload, NearGet_Ball1, NearShoot_Ball1, NearGet_Ball2, NearShoot_Ball2, NearGet_Ball3, NearShoot_Ball3, NearEndPath, finish}

    ;
    private NearPathState currentNearPathState = NearPathState.none;
    boolean shooterTimerBoolean = false;
    boolean OpmodeTimer = false;
    private Timer opmodeTimer;
    private Timer shooterTimer;

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
        shooterTimer = new Timer();
        opmodeTimer.resetTimer();
        shooterTimer.resetTimer();
        bluePath.setNearStartPose();
        slider.drive();
        waitForStart();
        while (opModeIsActive()) {
            shooter.shooterAiming = true;
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

                case NearGet_Ball1:
                    if (!setPath) {
                        intake.intake();
                        bluePath.follower.followPath(bluePath.NearGet_Ball1);
                        setPath = true;
                    }
                    if (!bluePath.follower.isBusy()) {
                        intake.stop();
                        setPath = false;
                        currentNearPathState = NearPathState.NearShoot_Ball1;
                        intake.stop();
                    }

                case NearShoot_Ball1:
                    if (!setPath) {
                        bluePath.follower.followPath(bluePath.NearShoot_Ball1);
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


                case NearGet_Ball2:
                    if (!setPath) {
                        intake.intake();
                        bluePath.follower.followPath(bluePath.NearGet_Ball2);
                        setPath = true;
                    }
                    if (!bluePath.follower.isBusy()) {
                        currentNearPathState = NearPathState.NearShoot_Ball2;
                        intake.stop();
                        setPath = false;
                    }

                case NearShoot_Ball2:
                    if (!setPath) {
                        bluePath.follower.followPath(bluePath.NearShoot_Ball2);
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

                case NearGet_Ball3:
                    if (!setPath) {
                        intake.intake();
                        bluePath.follower.followPath(bluePath.NearGet_Ball3);
                        setPath = true;
                    }
                    if (!bluePath.follower.isBusy()) {
                        currentNearPathState = NearPathState.NearShoot_Ball3;
                        intake.stop();
                        setPath = false;
                    }

                case NearShoot_Ball3:
                    if (!setPath) {
                        bluePath.follower.followPath(bluePath.NearShoot_Ball3);
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
            }
            bluePath.update();
            shooter.update(bluePath.NearShootPose,bluePath.NearShootPose.getHeading());


            telemetry.addData("NearPathState", currentNearPathState);
            telemetry.addData("x", bluePath.follower.getPose().getX());
            telemetry.addData("y", bluePath.follower.getPose().getY());
            telemetry.addData("heading", bluePath.follower.getPose().getHeading());
//            telemetry.addData("running path",bluePath.follower.)
            telemetry.update();
        }
    }
}