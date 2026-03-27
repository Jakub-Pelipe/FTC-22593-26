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

@Autonomous(name = "BlueEnc", group = "Autonomous")
public class BlueEnc extends OpMode {
    public double TPS;
    private Follower follower;
    public double milliseconds;
    public double distance;
    private DcMotorEx outtakeMotor;
    private DcMotor intakeMotor;
    private DcMotor intake2;
    private Servo kicker;
    private Servo rightBarrier;
    public Timer pathTimer, opmodeTimer, actionTimer;
    private int pathState;

    // REV Core Hex Motor specifications
    private final double OUTTAKE_TARGET_VELOCITY = 30; // ticks/sec (adjust as needed)
    private final double OUTTAKE_HOLD_POWER = 0.65;

    private final Pose startPose = new Pose(34.543, 134.752, Math.toRadians(270));
    private final Pose scorePose = new Pose(55.613, 77.396, Math.toRadians(140));
    private final Pose pickup1Pose = new Pose(20.4, 85.029, Math.toRadians(180));
    private final Pose control1 = new Pose(68.465, 57.191);
    private final Pose control2 = new Pose(77.685, 30.519);
    private final Pose pickup2Pose = new Pose(14.4, 60.561, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(14.4, 33.604, Math.toRadians(180));

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

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
    }

    public void autonomousPathUpdate() {
        // Precompute speed condition once
        boolean motorAtSpeed = TPS >= OUTTAKE_TARGET_VELOCITY * 0.45;

        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                actionTimer.resetTimer();
                pathState = 1;
                break;

            case 1:
                outtakeMotor.setPower(OUTTAKE_HOLD_POWER);
                follower.setMaxPowerScaling(0.8);
                kicker.setPosition(0);
                rightBarrier.setPosition(0.6);

                if (milliseconds >= 5500 && motorAtSpeed) {
                    outtakeMotor.setPower(0);
                    intake2.setPower(0);
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup1, true);
                        kicker.setPosition(0.6);
                        rightBarrier.setPosition(0.2);
                        actionTimer.resetTimer();
                        setPathState(2);
                    }
                } else if (milliseconds >= 5000 && motorAtSpeed) {
                    intakeMotor.setPower(1);
                    if (TPS > 20) intake2.setPower(1);
                } else if (milliseconds >= 3800) {
                    intake2.setPower(0);
                    intakeMotor.setPower(0);
                } else if (milliseconds >= 3100 && motorAtSpeed) {
                    intakeMotor.setPower(1);
                    if (TPS > 20) intake2.setPower(1);
                }
                break;

            case 2:
                if (distance <= 35) {
                    intakeMotor.setPower(1);
                    follower.setMaxPowerScaling(0.25);
                    if (!follower.isBusy()) {
                        follower.followPath(scorePickup1, true);
                        intakeMotor.setPower(0);
                        follower.setMaxPowerScaling(1);
                        actionTimer.resetTimer();
                        setPathState(3);
                    }
                }
                break;

            case 3:
                outtakeMotor.setPower(OUTTAKE_HOLD_POWER);
                follower.setMaxPowerScaling(0.8);
                rightBarrier.setPosition(0.6);
                kicker.setPosition(0);

                if (milliseconds >= 4600 && motorAtSpeed) {
                    outtakeMotor.setPower(0);
                    intake2.setPower(0);
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup2, true);
                        kicker.setPosition(0.6);
                        rightBarrier.setPosition(0.2);
                        actionTimer.resetTimer();
                        setPathState(4);
                    }
                } else if (milliseconds >= 3600 && motorAtSpeed) {
                    intakeMotor.setPower(1);
                    if (TPS > 20) intake2.setPower(1);
                } else if (milliseconds >= 2500) {
                    intake2.setPower(0);
                    intakeMotor.setPower(0);
                } else if (milliseconds >= 2100 && motorAtSpeed) {
                    intakeMotor.setPower(1);
                    if (TPS > 20) intake2.setPower(1);
                }
                break;

            case 4:
                if (distance <= 35) {
                    follower.setMaxPowerScaling(0.35);
                    if (TPS > 20) intakeMotor.setPower(1);
                    if (!follower.isBusy()) {
                        follower.followPath(scorePickup2, true);
                        intakeMotor.setPower(0);
                        follower.setMaxPowerScaling(1);
                        actionTimer.resetTimer();
                        setPathState(5);
                    }
                }
                break;

            case 5:
                outtakeMotor.setPower(getOuttakeHoldPower());
                follower.setMaxPowerScaling(0.8);
                rightBarrier.setPosition(0.6);
                kicker.setPosition(0);

                boolean proceed = (milliseconds >= 5100 && motorAtSpeed) || (milliseconds >= 6000);
                if (proceed) {
                    outtakeMotor.setPower(0);
                    intake2.setPower(0);
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup3, true);
                        kicker.setPosition(0.6);
                        rightBarrier.setPosition(0.2);
                        actionTimer.resetTimer();
                        setPathState(6);
                    }
                } else if (milliseconds >= 4200 && motorAtSpeed) {
                    intakeMotor.setPower(1);
                    if (TPS > 20) intake2.setPower(1);
                } else if (milliseconds >= 3200) {
                    intake2.setPower(0);
                    intakeMotor.setPower(0);
                } else if (milliseconds >= 2500 && motorAtSpeed) {
                    intakeMotor.setPower(1);
                    if (TPS > 20) intake2.setPower(1);
                }
                break;

            case 6:
                if (distance <= 35) {
                    follower.setMaxPowerScaling(0.35);
                    if (TPS > 20) intakeMotor.setPower(1);
                    if (!follower.isBusy()) {
                        follower.followPath(scorePickup3, true);
                        intakeMotor.setPower(0);
                        follower.setMaxPowerScaling(1);
                        actionTimer.resetTimer();
                        setPathState(7);
                    }
                }
                break;

            case 7:
                outtakeMotor.setPower(0.7);
                rightBarrier.setPosition(0.6);
                kicker.setPosition(0);

                proceed = (milliseconds >= 5200 && motorAtSpeed) || (milliseconds >= 6200);
                if (proceed) {
                    outtakeMotor.setPower(0);
                    intake2.setPower(0);
                    if (!follower.isBusy()) {
                        kicker.setPosition(0.6);
                        rightBarrier.setPosition(0.2);
                        actionTimer.resetTimer();
                        intakeMotor.setPower(0);
                        setPathState(-1);
                    }
                } else if (milliseconds >= 4400 && motorAtSpeed) {
                    intakeMotor.setPower(1);
                    if (TPS > 20) intake2.setPower(1);
                } else if (milliseconds >= 3800) {
                    intake2.setPower(0);
                    intakeMotor.setPower(0);
                } else if (milliseconds >= 3000 && motorAtSpeed) {
                    intakeMotor.setPower(1);
                    if (TPS > 20) intake2.setPower(1);
                }
                break;
        }
    }

    private double getOuttakeHoldPower() {
        return OUTTAKE_HOLD_POWER;
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        milliseconds = actionTimer.getElapsedTime();
        distance = follower.getDistanceRemaining();

        TPS = outtakeMotor.getVelocity();

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("time", milliseconds);
        telemetry.addData("Distance", distance);
        telemetry.addData("Outtake TPS", TPS);
        telemetry.addData("Outtake Target", OUTTAKE_TARGET_VELOCITY);
        telemetry.addData("TPS %", (TPS / OUTTAKE_TARGET_VELOCITY) * 100);
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

        // Configure outtake motor but DO NOT start it
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Important: No setVelocity or setPower here – motor stays off until needed

        // Configure intake motors – use REVERSE so positive power runs inward
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
        setPathState(0);
    }

    @Override
    public void stop() {
        if (outtakeMotor != null) outtakeMotor.setPower(0);
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (intake2 != null) intake2.setPower(0);
    }
}