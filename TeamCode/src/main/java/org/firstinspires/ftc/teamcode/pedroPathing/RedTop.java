package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name="RedTop", group = "Autonomous")
public class RedTop extends OpMode {
    private Follower follower;
    public double milliseconds;
    public double distance;
    private DcMotor outtakeMotor;
    private DcMotor intakeMotor;
    private DcMotor intake2;
    private Servo kicker;
    private Servo rightBarrier;
    public Timer pathTimer, opmodeTimer, actionTimer;
    private int pathState;
    private final Pose startPose = new Pose(35.543, 134.752, Math.toRadians(270)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(64.613, 83.396, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(27, 83.227, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose control1=new Pose(81.340,79.914);
    private final Pose pickup2Pose = new Pose(28.002, 59.16, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(24, 36, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1,grabPickup2, scorePickup2, grabPickup3, scorePickup3;


    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setTangentHeadingInterpolation()
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setTangentHeadingInterpolation()
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setTangentHeadingInterpolation()
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                actionTimer.resetTimer();
                pathState=1;;
                break;
            case 1:
                outtakeMotor.setPower(0.75);
                kicker.setPosition(0);
                rightBarrier.setPosition(0.6);

                if (milliseconds>=5000){
                    outtakeMotor.setPower(0);
                    intake2.setPower(0);
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup1, true);
                        kicker.setPosition(1);
                        rightBarrier.setPosition(0.1);
                        actionTimer.resetTimer();
                        setPathState(2);

                    }

                }

                else if (milliseconds>=4500){
                    intake2.setPower(1);
                    intakeMotor.setPower(-1);
                }

                else if (milliseconds>=4000){
                    intake2.setPower(0);
                    intakeMotor.setPower(0);
                }
                else if (milliseconds>=3100){
                    intakeMotor.setPower(-1);
                    intake2.setPower(1);
                }

                break;
            case 2:
                if (distance<=25) {
                    intakeMotor.setPower(-1);
                    follower.setMaxPowerScaling(0.25);

                    if (!follower.isBusy()) {
                        follower.followPath(scorePickup1, true);
                        intakeMotor.setPower(0);
                        follower.setMaxPowerScaling((1));
                        actionTimer.resetTimer();
                        setPathState(3);
                    }
                }
                break;
            case 3:
                outtakeMotor.setPower(0.75);
                //change to open gate position
                rightBarrier.setPosition(0.6);
                kicker.setPosition(0);

                if (milliseconds>=4300){
                    outtakeMotor.setPower(0);
                    intake2.setPower(0);
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup2, true);
                        kicker.setPosition(1);
                        rightBarrier.setPosition(0.1);
                        actionTimer.resetTimer();
                        setPathState(4);

                    }

                }

                else if (milliseconds>=3400){
                    intake2.setPower(1);
                    intakeMotor.setPower(-1);
                }

                else if (milliseconds>=2600){
                    intake2.setPower(0);
                    intakeMotor.setPower(0);
                }
                else if (milliseconds>=2100){
                    intakeMotor.setPower(-1);
                    intake2.setPower(1);
                }
                break;
            case 4:
                if (distance<=25) {
                    intakeMotor.setPower(-1);
                    follower.setMaxPowerScaling(0.25);

                    if (!follower.isBusy()) {
                        follower.followPath(scorePickup2, true);
                        intakeMotor.setPower(0);
                        follower.setMaxPowerScaling((1));
                        actionTimer.resetTimer();
                        setPathState(5);
                    }
                }
                break;
            case 5:
                outtakeMotor.setPower(0.75);
                //change to open gate position
                rightBarrier.setPosition(0.6);
                kicker.setPosition(0);

                //change timing
                if (milliseconds>=4600){
                    outtakeMotor.setPower(0);
                    intake2.setPower(0);
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup3, true);
                        kicker.setPosition(1);
                        rightBarrier.setPosition(0.1);
                        actionTimer.resetTimer();
                        setPathState(6);

                    }

                }
                //change timing
                else if (milliseconds>=4200){
                    intake2.setPower(1);
                    intakeMotor.setPower(-1);
                }
                //change timing
                else if (milliseconds>=3400){
                    intake2.setPower(0);
                    intakeMotor.setPower(0);
                }
                //change timing
                else if (milliseconds>=2500){
                    intakeMotor.setPower(-1);
                    intake2.setPower(1);
                }
                break;
            case 6:
                //need to go to shooting area
                if (!follower.isBusy()) {
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    setPathState(-1);
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
        milliseconds=actionTimer.getElapsedTime();
        distance=follower.getDistanceRemaining();
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("time",milliseconds);
        telemetry.addData("Distance",distance);
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer=new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        outtakeMotor=hardwareMap.get(DcMotor.class,"outtakeMotor");
        intakeMotor=hardwareMap.get(DcMotor.class,"intakeMotor");
        intake2=hardwareMap.get(DcMotor.class,"intake2");
        kicker=hardwareMap.get(Servo.class,"kicker");
        rightBarrier=hardwareMap.get(Servo.class,"rightBarrier");


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
        pathTimer.resetTimer();

        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}