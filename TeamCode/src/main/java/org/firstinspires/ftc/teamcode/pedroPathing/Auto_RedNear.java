package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.path.RedPath;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Slider;

@Autonomous(name = "AutoPathing_RedNear")
public class Auto_RedNear extends LinearOpMode {
    boolean PathGrabShoot1 = true;
    boolean PathGrabShoot2 = true;
    boolean PathGrabShoot3 = true;
    private RedPath redPath;
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
        redPath = new RedPath(hardwareMap);
        shooter = new Shooter(hardwareMap, Shooter.Alliance.BLUE);
        intake = new Intake(hardwareMap);
        opmodeTimer = new Timer();
        shooterTimer = new Timer();
        opmodeTimer.resetTimer();
        shooterTimer.resetTimer();
        redPath.setNearStartPose();
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
                        redPath.follower.followPath(redPath.NearScorePreload);
                        setPath = true;
                        shooter.shooterRotate = true;
                        shooterTimerBoolean = false;
                    }
                    if (!redPath.follower.isBusy()) {
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
                        redPath.follower.followPath(redPath.NearGet_Ball1);
                        setPath = true;
                    }
                    if (!redPath.follower.isBusy()) {
                        intake.stop();
                        setPath = false;
                        currentNearPathState = NearPathState.NearShoot_Ball1;
                        intake.stop();
                    }

                case NearShoot_Ball1:
                    if (!setPath) {
                        redPath.follower.followPath(redPath.NearShoot_Ball1);
                        setPath = true;
                        shooter.shooterRotate = true;
                        shooterTimerBoolean = false;
                    }
                    if (!redPath.follower.isBusy()) {
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
                        redPath.follower.followPath(redPath.NearGet_Ball2);
                        setPath = true;
                    }
                    if (!redPath.follower.isBusy()) {
                        currentNearPathState = NearPathState.NearShoot_Ball2;
                        intake.stop();
                        setPath = false;
                    }

                case NearShoot_Ball2:
                    if (!setPath) {
                        redPath.follower.followPath(redPath.NearShoot_Ball2);
                        setPath = true;
                        shooter.shooterRotate = true;
                        shooterTimerBoolean = false;
                    }
                    if (!redPath.follower.isBusy()) {
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
                        redPath.follower.followPath(redPath.NearGet_Ball3);
                        setPath = true;
                    }
                    if (!redPath.follower.isBusy()) {
                        currentNearPathState = NearPathState.NearShoot_Ball3;
                        intake.stop();
                        setPath = false;
                    }

                case NearShoot_Ball3:
                    if (!setPath) {
                        redPath.follower.followPath(redPath.NearShoot_Ball3);
                        setPath = true;
                        shooter.shooterRotate = true;
                        shooterTimerBoolean = false;
                    }
                    if (!redPath.follower.isBusy()) {
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
                        redPath.follower.followPath(redPath.NearEndPath);
                        setPath = true;
                    }
                    if (!redPath.follower.isBusy()) {
                        currentNearPathState = NearPathState.finish;
                        setPath = false;
                    }

                case finish:
            }
            redPath.update();
            shooter.update(redPath.NearShootPose,redPath.NearShootPose.getHeading());


            telemetry.addData("NearPathState", currentNearPathState);
            telemetry.addData("x", redPath.follower.getPose().getX());
            telemetry.addData("y", redPath.follower.getPose().getY());
            telemetry.addData("heading", redPath.follower.getPose().getHeading());
//            telemetry.addData("running path",redPath.follower.)
            telemetry.update();
        }
    }
}