package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous (name="square", group = "Autonomous")
public class square extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(71.4, 21.8, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose topright = new Pose(109.5, 83.7, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose topleft = new Pose(33.5, 83.7, Math.toRadians(270)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose bottomleft = new Pose(33.5, 35.1, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose bottomright = new Pose(109.5, 35.1, Math.toRadians(90)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private Path startpoint;
    private PathChain tophorizontal, leftvertical, bottomhorizontal, rightvertical;


    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        startpoint = new Path(new BezierLine(startPose, topright));
        startpoint.setLinearHeadingInterpolation(startPose.getHeading(), topright.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        tophorizontal = follower.pathBuilder()
                .addPath(new BezierLine(topright, topleft))
                .setLinearHeadingInterpolation(topright.getHeading(), topleft.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        leftvertical = follower.pathBuilder()
                .addPath(new BezierLine(topleft, bottomleft))
                .setLinearHeadingInterpolation(topleft.getHeading(), bottomleft.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        bottomhorizontal = follower.pathBuilder()
                .addPath(new BezierLine(bottomleft, bottomright))
                .setLinearHeadingInterpolation(bottomleft.getHeading(), bottomright.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        rightvertical = follower.pathBuilder()
                .addPath(new BezierLine(bottomright,topright))
                .setLinearHeadingInterpolation(bottomright.getHeading(), topleft.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(startpoint);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(tophorizontal, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(leftvertical, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(bottomhorizontal, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(rightvertical, true);
                    setPathState(1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}