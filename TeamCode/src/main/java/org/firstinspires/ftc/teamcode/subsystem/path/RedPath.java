package org.firstinspires.ftc.teamcode.subsystem.path;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.RobotState;

public class RedPath {
    private final Pose FarStartPose = new Pose(122, 124.06919275123559, Math.toRadians(125));
    private final Pose NearStartPose = new Pose(88,9.01153212520594,Math.toRadians(90));
    private final Pose FarShootPose = new Pose(85, 97.67874794069192, Math.toRadians(35));
    private final Pose NearShootPose = new Pose(88,8, Math.toRadians(75));
    private final Pose EndPose = new Pose(91.29654036243821,59.820428336079075,Math.toRadians(0));
    private final Pose HumanPlayerPose = new Pose(12.001647446457977, 10.604612850082383, Math.toRadians(180));
    private final Pose GatePose = new Pose(16.03459637561778, 68.01482701812193, Math.toRadians(180));
    private final Pose FarPickUp1_start = new Pose(103.26029654036246,83.44481054365735,Math.toRadians(0));//From Up to Down
    private final Pose FarPickUp1_final = new Pose(128,83.44481054365735,Math.toRadians(0));//From Up to Down
    private final Pose FarPickUp2_start = new Pose(102.19110378912686,59.22075782537066,Math.toRadians(0));//From Up to Down
    private final Pose FarPickUp2_final = new Pose(129,59.22075782537066,Math.toRadians(0));//From Up to Down
    private final Pose FarPickUp3_start = new Pose(102.56013179571664,35.52553542009881,Math.toRadians(0));//From Up to Down
    private final Pose FarPickUp3_final = new Pose(129,35.52553542009881, Math.toRadians(0));//From Up to Down
    private final Pose NearPickUp1_start = new Pose(102.56013179571664,35.52553542009881,0);//From Down to Up
    private final Pose NearPickUp1_final = new Pose(129,35.52553542009881,0);//From Down to Up
    private final Pose NearPickUp2_start = new Pose(102.19110378912686,59.22075782537066,0);//From Down to Up
    private final Pose NearPickUp2_final = new Pose(129,59.22075782537066,0);//From Down to UP
    private final Pose NearPickUp3_start = new Pose(103.26029654036246,83.44481054365735,0);//From Down to Up
    private final Pose NearPickUp3_final = new Pose(128,83.44481054365735,0);//From Down to Up
    public PathChain NearScorePreload, NearGet_Ball1, NearShoot_Ball1, HumanPlayer_Ball, NearGet_Ball2, NearShoot_Ball2, NearGet_Ball3, NearShoot_Ball3;
    public PathChain FarScorePreload, FarGet_Ball1, FarShoot_Ball1,FarGet_Ball2, FarShoot_Ball2, FarGet_Ball3, FarShoot_Ball3;
    public PathChain FarEndPath, NearEndPath;
    public Follower follower;

    public RedPath(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
    }

    public void setNearStartPose(){
        follower.setStartingPose(NearStartPose);
    }
    public void setFarStartPose(){
        follower.setStartingPose(FarStartPose);
    }
    public void setTeleopStartPose(){follower.setStartingPose(RobotState.currentPose);}

    public double getX(){return follower.getPose().getX();}
    public double getY(){return follower.getPose().getY();}
    public double getHeading(){return follower.getPose().getHeading();}

    private void buildPaths(){
        NearScorePreload = follower.pathBuilder()
                .addPath(new BezierLine(NearStartPose, NearStartPose))
                .setLinearHeadingInterpolation(NearStartPose.getHeading(), NearShootPose.getHeading())
                .build();
        FarScorePreload = follower.pathBuilder()
                .addPath(new BezierLine(FarStartPose, FarShootPose))
                .setLinearHeadingInterpolation(FarPickUp1_start.getHeading(), FarPickUp1_final.getHeading())
                .build();

        HumanPlayer_Ball = follower.pathBuilder()
                .addPath(new BezierLine(NearShootPose, HumanPlayerPose))
                .setLinearHeadingInterpolation(NearShootPose.getHeading(), HumanPlayerPose.getHeading())
                .build();

        NearGet_Ball1 = follower.pathBuilder()
                .addPath(new BezierCurve(NearShootPose, NearPickUp1_start, NearPickUp1_final))
                .setLinearHeadingInterpolation(NearShootPose.getHeading(), NearPickUp1_final.getHeading())
                .build();
        NearShoot_Ball1 = follower.pathBuilder()
                .addPath(new BezierLine(NearPickUp1_final, NearShootPose))
                .setLinearHeadingInterpolation(NearPickUp1_final.getHeading(), NearShootPose.getHeading())
                .build();
        FarGet_Ball1 = follower.pathBuilder()
                .addPath(new BezierCurve(FarShootPose, FarPickUp1_start, FarPickUp1_final, GatePose))
                .setLinearHeadingInterpolation(FarShootPose.getHeading(), FarPickUp1_final.getHeading())
                .build();
        FarShoot_Ball1 = follower.pathBuilder()
                .addPath(new BezierLine(FarPickUp1_final, FarShootPose))
                .setLinearHeadingInterpolation(FarPickUp1_final.getHeading(), FarShootPose.getHeading())
                .build();

        NearGet_Ball2 = follower.pathBuilder()
                .addPath(new BezierCurve(NearShootPose, NearPickUp2_start, NearPickUp2_final))
                .setLinearHeadingInterpolation(NearShootPose.getHeading(), NearPickUp2_final.getHeading())
                .build();
        NearShoot_Ball2 = follower.pathBuilder()
                .addPath(new BezierLine(NearPickUp2_final, NearShootPose))
                .setLinearHeadingInterpolation(NearPickUp2_final.getHeading(), NearShootPose.getHeading())
                .build();
        FarGet_Ball2 = follower.pathBuilder()
                .addPath(new BezierCurve(FarShootPose, FarPickUp2_start, FarPickUp2_final, GatePose))
                .setLinearHeadingInterpolation(FarShootPose.getHeading(), FarPickUp2_final.getHeading())
                .build();
        FarShoot_Ball2 = follower.pathBuilder()
                .addPath(new BezierLine(FarPickUp2_final, FarShootPose))
                .setLinearHeadingInterpolation(FarPickUp2_final.getHeading(), FarShootPose.getHeading())
                .build();

        NearGet_Ball3 = follower.pathBuilder()
                .addPath(new BezierCurve(NearShootPose, NearPickUp3_start, NearPickUp3_final))
                .setLinearHeadingInterpolation(NearShootPose.getHeading(), NearPickUp3_start.getHeading())
                .build();
        NearShoot_Ball3 = follower.pathBuilder()
                .addPath(new BezierLine(NearPickUp3_final, NearShootPose))
                .setLinearHeadingInterpolation(NearPickUp3_final.getHeading(), NearShootPose.getHeading())
                .build();
        FarGet_Ball3 = follower.pathBuilder()
                .addPath(new BezierCurve(FarShootPose, FarPickUp3_start, FarPickUp3_final))
                .setLinearHeadingInterpolation(FarShootPose.getHeading(), FarPickUp3_final.getHeading())
                .build();
        FarShoot_Ball3 = follower.pathBuilder()
                .addPath(new BezierLine(FarPickUp3_final, FarShootPose))
                .setLinearHeadingInterpolation(FarPickUp3_final.getHeading(), FarShootPose.getHeading())
                .build();

        FarEndPath = follower.pathBuilder()
                .addPath(new BezierLine(FarShootPose, EndPose))
                .setLinearHeadingInterpolation(FarShootPose.getHeading(), EndPose.getHeading())
                .build();
        NearEndPath = follower.pathBuilder()
                .addPath(new BezierLine(NearShootPose, EndPose))
                .setLinearHeadingInterpolation(NearShootPose.getHeading(), EndPose.getHeading())
                .build();
    }
    public void update(){
        follower.update();
        RobotState.currentPose = follower.getPose();
    }
}