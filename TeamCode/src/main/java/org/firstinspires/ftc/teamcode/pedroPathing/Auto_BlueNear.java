package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.path.BluePath;

@Autonomous(name = "AutoPathing_BlueNear")
public class Auto_BlueNear extends LinearOpMode {
    boolean PathGrabShoot1 = true;
    boolean PathGrabShoot2 = true;
    boolean PathGrabShoot3 = true;
    private boolean setPath = false;
    private BluePath bluePath;
    private Shooter shooter;

    private enum NearPathState {none, NearScorePreload, NearGet_Ball1, NearShoot_Ball1, NearGet_Ball2, NearShoot_Ball2, NearGet_Ball3, NearShoot_Ball3, NearEndPath, finish}

    ;
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
            } else {
                currentNearPathState = NearPathState.finish;
            }
        }
    }

    @Override
    public void runOpMode() {
        bluePath = new BluePath(hardwareMap);
        shooter = new Shooter(hardwareMap, Shooter.Alliance.BLUE);
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
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
                    setPath = true;

                case NearScorePreload:
                    if (setPath) {
                        bluePath.follower.followPath(bluePath.NearScorePreload);
                        setPath = false;
                    }
                    if (!bluePath.follower.isBusy()) {
                        determinePath(0);
                        setPath = true;
                    }

                case NearGet_Ball1:
                    if (setPath) {
                        bluePath.follower.followPath(bluePath.NearGet_Ball1);
                        setPath = false;
                    }
                    if (!bluePath.follower.isBusy()) {
                        currentNearPathState = NearPathState.NearShoot_Ball1;
                        setPath = true;
                    }

                case NearShoot_Ball1:
                    if (setPath) {
                        bluePath.follower.followPath(bluePath.NearShoot_Ball1);
                        setPath = false;
                    }
                    if (!bluePath.follower.isBusy()) {
                        determinePath(1);
                        setPath = true;
                    }

                case NearGet_Ball2:
                    if (setPath) {
                        bluePath.follower.followPath(bluePath.NearGet_Ball2);
                        setPath = false;
                    }
                    if (!bluePath.follower.isBusy()) {
                        currentNearPathState = NearPathState.NearShoot_Ball2;
                        setPath = true;
                    }

                case NearShoot_Ball2:
                    if (setPath) {
                        bluePath.follower.followPath(bluePath.NearShoot_Ball2);
                        setPath = false;
                    }
                    if (!bluePath.follower.isBusy()) {
                        determinePath(2);
                        setPath = true;
                    }

                case NearGet_Ball3:
                    if (setPath) {
                        bluePath.follower.followPath(bluePath.NearGet_Ball3);
                        setPath = false;
                    }
                    if (!bluePath.follower.isBusy()) {
                        currentNearPathState = NearPathState.NearShoot_Ball3;
                        setPath = true;
                    }

                case NearShoot_Ball3:
                    if (setPath) {
                        bluePath.follower.followPath(bluePath.NearShoot_Ball3);
                        setPath = false;
                    }
                    if (!bluePath.follower.isBusy()) {
                        determinePath(3);
                        setPath = true;
                    }

                case NearEndPath:
                    if (setPath) {
                        bluePath.follower.followPath(bluePath.NearEndPath);
                        setPath = false;
                    }
                    if (!bluePath.follower.isBusy()) {
                        currentNearPathState = NearPathState.finish;
                    }

                case finish:
                    if (gamepad1.triangle) {
                        currentNearPathState = NearPathState.none;
                        setPath = true;
                    }
            }
            bluePath.follower.update();
            shooter.update(new Pose(56.37232289950577, 9.01153212520594), Math.toRadians(90));


            telemetry.addData("NearPathState", currentNearPathState);
            telemetry.addData("x", bluePath.follower.getPose().getX());
            telemetry.addData("y", bluePath.follower.getPose().getY());
            telemetry.addData("heading", bluePath.follower.getPose().getHeading());
            telemetry.update();
        }
    }
}