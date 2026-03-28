package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "RedEnc4", group = "Autonomous")
public class RedEnc4 extends OpMode {
    public double TPS;
    private Follower follower;
    public double totalTime;
    public double distance;
    private DcMotorEx outtakeMotor;
    private DcMotor intakeMotor;
    private DcMotor intake2;
    private Servo kicker;
    private Servo rightBarrier;
    public Timer pathTimer, opmodeTimer, actionTimer;
    private int pathState;
    private boolean isStopped = false;

    // REV Core Hex Motor specifications
    private final double OUTTAKE_TARGET_VELOCITY = 30; // ticks/sec
    private final double OUTTAKE_HOLD_POWER = 0.65;

    private final Pose startPose = new Pose(133.752, 134.752, Math.toRadians(270));
    private final Pose scorePose = new Pose(117, 94.396, Math.toRadians(50));
    private final Pose pickup1Pose = new Pose(151.9, 98.029, Math.toRadians(0));
    // Original control points (restored)
    private final Pose control1 = new Pose(73.465, 57.191, Math.toRadians(0));
    private final Pose control2 = new Pose(85.685, 43.519, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(156, 71.7, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(154.6, 43.461, Math.toRadians(0));
    private final Pose finalPose = new Pose(140, 81, Math.toRadians(0));

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;
    private Path moveToFinal;

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
                .addPath(new BezierCurve(scorePose, control1, pickup2Pose))
                .setTangentHeadingInterpolation()
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, control2, pickup3Pose))
                .setTangentHeadingInterpolation()
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        moveToFinal = new Path(new BezierLine(scorePose, finalPose));
        moveToFinal.setLinearHeadingInterpolation(scorePose.getHeading(), finalPose.getHeading());
    }

    public void autonomousPathUpdate() {
        if (isStopped) return;
        double stateTime = pathTimer.getElapsedTime();

        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                double threshold1 = 0.42;
                boolean motorAtSpeed1 = TPS >= OUTTAKE_TARGET_VELOCITY * threshold1;
                outtakeMotor.setPower(OUTTAKE_HOLD_POWER);
                follower.setMaxPowerScaling(0.8);
                kicker.setPosition(0);
                rightBarrier.setPosition(0.6);

                if (stateTime >= 5500 && motorAtSpeed1) {
                    outtakeMotor.setPower(0);
                    intake2.setPower(0);
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup1, true);
                        kicker.setPosition(0.6);
                        rightBarrier.setPosition(0.2);
                        setPathState(2);
                    }
                } else if (stateTime >= 5000 && motorAtSpeed1) {
                    intakeMotor.setPower(1);
                    if (TPS > 20) intake2.setPower(1);
                } else if (stateTime >= 3800) {
                    intake2.setPower(0);
                    intakeMotor.setPower(0);
                } else if (stateTime >= 3100 && motorAtSpeed1) {
                    intakeMotor.setPower(1);
                    if (TPS > 20) intake2.setPower(1);
                }
                break;

            case 2:
                if (distance <= 35) {
                    intakeMotor.setPower(1);
                    follower.setMaxPowerScaling(0.41);
                    if (!follower.isBusy()) {
                        follower.followPath(scorePickup1, true);
                        intakeMotor.setPower(0);
                        follower.setMaxPowerScaling(1);
                        setPathState(3);
                    }
                }
                break;

            case 3:
                double threshold2 = 0.42;
                boolean motorAtSpeed2 = TPS >= OUTTAKE_TARGET_VELOCITY * threshold2;
                outtakeMotor.setPower(OUTTAKE_HOLD_POWER);
                follower.setMaxPowerScaling(0.8);
                rightBarrier.setPosition(0.6);
                kicker.setPosition(0);

                if (stateTime >= 4600 && motorAtSpeed2) {
                    outtakeMotor.setPower(0);
                    intake2.setPower(0);
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup2, true);
                        kicker.setPosition(0.6);
                        rightBarrier.setPosition(0.2);
                        setPathState(4);
                    }
                } else if (stateTime >= 3600 && motorAtSpeed2) {
                    intakeMotor.setPower(1);
                    if (TPS > 20) intake2.setPower(1);
                } else if (stateTime >= 2500) {
                    intake2.setPower(0);
                    intakeMotor.setPower(0);
                } else if (stateTime >= 2100 && motorAtSpeed2) {
                    intakeMotor.setPower(1);
                    if (TPS > 20) intake2.setPower(1);
                }
                break;

            case 4:
                if (distance <= 35) {
                    follower.setMaxPowerScaling(0.45);
                    if (TPS > 20) intakeMotor.setPower(1);
                    if (!follower.isBusy()) {
                        follower.followPath(scorePickup2, true);
                        intakeMotor.setPower(0);
                        follower.setMaxPowerScaling(1);
                        setPathState(5);
                    }
                }
                break;

            case 5:
                double threshold3 = 0.28;
                boolean motorAtSpeed3 = TPS >= OUTTAKE_TARGET_VELOCITY * threshold3;
                outtakeMotor.setPower(OUTTAKE_HOLD_POWER);
                follower.setMaxPowerScaling(0.8);
                rightBarrier.setPosition(0.6);
                kicker.setPosition(0);

                boolean proceed = (stateTime >= 5100 && motorAtSpeed3) || (stateTime >= 6000);
                if (proceed) {
                    outtakeMotor.setPower(0);
                    intake2.setPower(0);
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup3, true);
                        kicker.setPosition(0.6);
                        rightBarrier.setPosition(0.2);
                        setPathState(6);
                    }
                } else if (stateTime >= 4200 && motorAtSpeed3) {
                    intakeMotor.setPower(1);
                    if (TPS > 20) intake2.setPower(1);
                } else if (stateTime >= 3200) {
                    intake2.setPower(0);
                    intakeMotor.setPower(0);
                } else if (stateTime >= 2500 && motorAtSpeed3) {
                    intakeMotor.setPower(1);
                    if (TPS > 20) intake2.setPower(1);
                }
                break;

            case 6:
                if (distance <= 35) {
                    follower.setMaxPowerScaling(0.45);
                    if (!follower.isBusy()) {
                        follower.followPath(scorePickup3, true);
                        intakeMotor.setPower(0);
                        follower.setMaxPowerScaling(1);
                        setPathState(7);
                    }
                }
                break;

            case 7:
                double threshold4 = 0.35;
                boolean motorAtSpeed4 = TPS >= OUTTAKE_TARGET_VELOCITY * threshold4;
                outtakeMotor.setPower(0.7);
                rightBarrier.setPosition(0.6);
                kicker.setPosition(0);

                proceed = (stateTime >= 5200 && motorAtSpeed4) || (stateTime >= 6200);
                if (proceed) {
                    outtakeMotor.setPower(0);
                    intake2.setPower(0);
                    if (!follower.isBusy()) {
                        kicker.setPosition(0.6);
                        rightBarrier.setPosition(0.2);
                        intakeMotor.setPower(0);
                        setPathState(8);
                    }
                } else if (stateTime >= 4400 && motorAtSpeed4) {
                    intakeMotor.setPower(1);
                    if (TPS > 20) intake2.setPower(1);
                } else if (stateTime >= 3800) {
                    intake2.setPower(0);
                    intakeMotor.setPower(0);
                } else if (stateTime >= 3000 && motorAtSpeed4) {
                    intakeMotor.setPower(1);
                    if (TPS > 20) intake2.setPower(1);
                }
                break;

            case 8:
                follower.followPath(moveToFinal);
                setPathState(9);
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.breakFollowing();
                    setPathState(-1);
                } else if (stateTime >= 5000) {
                    follower.breakFollowing();
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        if (!isStopped && opmodeTimer.getElapsedTime() >= 29500) {
            isStopped = true;
            if (outtakeMotor != null) outtakeMotor.setPower(0);
            if (intakeMotor != null) intakeMotor.setPower(0);
            if (intake2 != null) intake2.setPower(0);
            if (follower != null) follower.breakFollowing();
            pathState = -1;
        }

        if (isStopped) {
            telemetry.addData("Status", "CUTOFF - Time limit reached");
            telemetry.addData("Runtime (ms)", opmodeTimer.getElapsedTime());
            telemetry.update();
            return;
        }

        totalTime = opmodeTimer.getElapsedTime();
        distance = follower.getDistanceRemaining();
        TPS = outtakeMotor.getVelocity();

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("state time", pathTimer.getElapsedTime());
        telemetry.addData("Distance", distance);
        telemetry.addData("Outtake TPS", TPS);
        telemetry.addData("Outtake Target", OUTTAKE_TARGET_VELOCITY);
        telemetry.addData("TPS %", (TPS / OUTTAKE_TARGET_VELOCITY) * 100);
        telemetry.addData("Runtime (ms)", totalTime);
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        kicker = hardwareMap.get(Servo.class, "kicker");
        rightBarrier = hardwareMap.get(Servo.class, "rightBarrier");

        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Initialized – motor off");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        actionTimer.resetTimer();
        isStopped = false;
        setPathState(0);
    }

    @Override
    public void stop() {
        if (outtakeMotor != null) outtakeMotor.setPower(0);
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (intake2 != null) intake2.setPower(0);
    }
}