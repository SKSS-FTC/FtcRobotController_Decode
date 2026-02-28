package org.firstinspires.ftc.teamcode.subsystem.path;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.RobotState;

public class BluePath {
    public static Pose startingPose;
    private final Pose FarStartPose = new Pose(22.138385502471177, 124.06919275123559, Math.toRadians(140));
    private final Pose NearStartPose = new Pose(56.37232289950577,9.01153212520594, Math.toRadians(90));
    private final Pose FarShootPose = new Pose(59.24382207578253, 97.67874794069192,  Math.toRadians(140));
    private final Pose NearShootPose = new Pose(56.37232289950577,9.01153212520594, Math.toRadians(180));
    private final Pose EndPose = new Pose(36.309719934102134,10.846787479406943, Math.toRadians(90));
    private final Pose GatePose = new Pose(15.433278418451394,69.68039538714994, Math.toRadians(180));
    private final Pose HumanPlayerPose = new Pose(10.925864909390437,10.846787479406942, Math.toRadians(180));
    private final Pose FarPickUp1_start = new Pose(43.24052718286654,83.84184514003294, Math.toRadians(180));//From Up to Down
    private final Pose FarPickUp1_final = new Pose(16.051070840197696,83.60461285008238, Math.toRadians(180));//From Up to Down
    private final Pose FarPickUp2_start = new Pose(41.70345963756179,59.495881383855036, Math.toRadians(180));//From Up to Down
    private final Pose FarPickUp2_final = new Pose(18.05766062602966,59.258649093904445, Math.toRadians(180));//From Up to Down
    private final Pose FarPickUp3_start = new Pose(39.62932454695221,35.98023064250413, Math.toRadians(180));//From Up to Down
    private final Pose FarPickUp3_final = new Pose(14.828665568369027,35.742998352553556, Math.toRadians(180));//From Up to Down
    private final Pose NearPickUp1_start = new Pose(39.62932454695221,35.98023064250413,180);//From Down to Up
    private final Pose NearPickUp1_final = new Pose(14.828665568369027,35.742998352553556,180);//From Down to Up
    private final Pose NearPickUp2_start = new Pose(41.70345963756179,59.495881383855036,180);//From Down to Up
    private final Pose NearPickUp2_final = new Pose(18.05766062602966,59.258649093904445,180);//From Down to UP
    private final Pose NearPickUp3_start = new Pose(43.24052718286654,83.84184514003294,180);//From Down to Up
    private final Pose NearPickUp3_final = new Pose(16.051070840197696,83.60461285008238,180);//From Down to Up
    public PathChain NearScorePreload, NearGet_Ball1, NearShoot_Ball1, HumanPlayer_Ball, NearGet_Ball2, NearShoot_Ball2, NearGet_Ball3, NearShoot_Ball3;
    public PathChain FarScorePreload, FarGet_Ball1, FarShoot_Ball1,FarGet_Ball2, FarShoot_Ball2, FarGet_Ball3, FarShoot_Ball3;
    public PathChain NearEndPath, FarEndPath;
    public Follower follower;

    public BluePath(HardwareMap hardwareMap) {
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
                .build();

        NearGet_Ball1.setHeadingInterpolator(
                HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0,
                0.2,
                        HeadingInterpolator.linear(NearShootPose.getHeading(),NearPickUp1_start.getHeading())),
                new HeadingInterpolator.PiecewiseNode(
                        0.2,
                        1,
                        HeadingInterpolator.constant(NearPickUp1_final.getHeading()))
                )
        );
        NearShoot_Ball1 = follower.pathBuilder()
                .addPath(new BezierLine(NearPickUp1_final, NearShootPose))
                .setLinearHeadingInterpolation(NearPickUp1_final.getHeading(), NearShootPose.getHeading())
                .build();
        FarGet_Ball1 = follower.pathBuilder()
                .addPath(new BezierCurve(FarShootPose, FarPickUp1_start, FarPickUp1_final, GatePose))
                .build();

        FarGet_Ball1.setHeadingInterpolator(
                HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0,
                0.2,
                        HeadingInterpolator.linear(FarShootPose.getHeading(),FarPickUp1_start.getHeading())),
                new HeadingInterpolator.PiecewiseNode(
                        0.2,
                        1,
                        HeadingInterpolator.constant(FarPickUp1_final.getHeading()))
                )
        );
        FarShoot_Ball1 = follower.pathBuilder()
                .addPath(new BezierLine(FarPickUp1_final, FarShootPose))
                .setLinearHeadingInterpolation(FarPickUp1_final.getHeading(), FarShootPose.getHeading())
                .build();

        NearGet_Ball2 = follower.pathBuilder()
                .addPath(new BezierCurve(NearShootPose, NearPickUp2_start, NearPickUp2_final))
                .build();

        NearGet_Ball2.setHeadingInterpolator(
                HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0,
                0.2,
                        HeadingInterpolator.linear(NearShootPose.getHeading(),NearPickUp2_start.getHeading())),
                new HeadingInterpolator.PiecewiseNode(
                        0.2,
                        1,
                        HeadingInterpolator.constant(NearPickUp2_final.getHeading()))
                )
        );
        NearShoot_Ball2 = follower.pathBuilder()
                .addPath(new BezierLine(NearPickUp2_final, NearShootPose))
                .setLinearHeadingInterpolation(NearPickUp2_final.getHeading(), NearShootPose.getHeading())
                .build();
        FarGet_Ball2 = follower.pathBuilder()
                .addPath(new BezierCurve(FarShootPose, FarPickUp2_start, FarPickUp2_final, GatePose))
                .build();

        FarGet_Ball2.setHeadingInterpolator(
                HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0,
                0.2,
                        HeadingInterpolator.linear(FarShootPose.getHeading(),FarPickUp2_start.getHeading())),
                new HeadingInterpolator.PiecewiseNode(
                        0.2,
                        1,
                        HeadingInterpolator.constant(FarPickUp2_final.getHeading()))
                )
        );
        FarShoot_Ball2 = follower.pathBuilder()
                .addPath(new BezierLine(FarPickUp2_final, FarShootPose))
                .setLinearHeadingInterpolation(FarPickUp2_final.getHeading(), FarShootPose.getHeading())
                .build();

        NearGet_Ball3 = follower.pathBuilder()
                .addPath(new BezierCurve(NearShootPose, NearPickUp3_start, NearPickUp3_final))
                .build();

        NearGet_Ball3.setHeadingInterpolator(
                HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0,
                0.2,
                        HeadingInterpolator.linear(NearShootPose.getHeading(),NearPickUp3_start.getHeading())),
                new HeadingInterpolator.PiecewiseNode(
                        0.2,
                        1,
                        HeadingInterpolator.constant(NearPickUp3_final.getHeading()))
                )
        );
        NearShoot_Ball3 = follower.pathBuilder()
                .addPath(new BezierLine(NearPickUp3_final, NearShootPose))
                .setLinearHeadingInterpolation(NearPickUp3_final.getHeading(), NearShootPose.getHeading())
                .build();
        FarGet_Ball3 = follower.pathBuilder()
                .addPath(new BezierCurve(FarShootPose, FarPickUp3_start, FarPickUp3_final))
                .build();

        FarGet_Ball3.setHeadingInterpolator(
                HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0,
                0.2,
                        HeadingInterpolator.linear(FarShootPose.getHeading(),FarPickUp3_start.getHeading())),
                new HeadingInterpolator.PiecewiseNode(
                        0.2,
                        1,
                        HeadingInterpolator.constant(FarPickUp3_final.getHeading()))
                )
        );
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