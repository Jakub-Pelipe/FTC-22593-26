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

@Autonomous(name = "RedEnc5", group = "Autonomous")
public class RedEnc5 extends OpMode {
    // Motor constants – adjusted for REV Core Hex Motor
    private static final double OUTTAKE_CPR = 28;           // Counts per revolution of the output shaft
    private static final double OUTTAKE_RPM_TARGET = 250;   // Desired outtake speed in RPM
    private static final double OUTTAKE_HOLD_POWER = 0.8;   // Power to maintain speed when running
    // Precompute target velocity in ticks/sec for efficiency
    private static final double OUTTAKE_TARGET_VELOCITY = (OUTTAKE_RPM_TARGET * OUTTAKE_CPR) / 60.0;

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
    private boolean outtakeStarted = false;  // Track if outtake has been started

    // Pose definitions
    private final Pose startPose = new Pose(133.752, 134.543, Math.toRadians(270));
    private final Pose scorePose = new Pose(92, 91.396, Math.toRadians(40));
    private final Pose pickup1Pose = new Pose(140.6, 97.029, Math.toRadians(0));
    private final Pose control1 = new Pose(68.465, 57.191);
    private final Pose control2 = new Pose(75.685, 43.519);
    private final Pose pickup2Pose = new Pose(145.6, 71.761, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(129.6, 27.304, Math.toRadians(0));

    // Path definitions
    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    // Helper method to set outtake motor to a constant RPM
    private void setOuttakeSpeed(double rpm) {
        double ticksPerSecond = (rpm * OUTTAKE_CPR) / 60.0;
        outtakeMotor.setVelocity(ticksPerSecond);
    }

    private void startOuttake() {
        if (!outtakeStarted) {
            setOuttakeSpeed(OUTTAKE_RPM_TARGET);
            outtakeMotor.setPower(OUTTAKE_HOLD_POWER);
            outtakeStarted = true;
        }
    }

    private void stopOuttake() {
        outtakeMotor.setPower(0);
        outtakeStarted = false;
    }

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
        // Precompute target velocity and current speed once per update
        double currentVelocity = outtakeMotor.getVelocity();
        boolean atTargetSpeed = currentVelocity >= OUTTAKE_TARGET_VELOCITY * 0.85;

        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                actionTimer.resetTimer();
                pathState = 1;
                break;

            case 1:
                startOuttake();
                kicker.setPosition(0);
                rightBarrier.setPosition(0.6);

                if (milliseconds >= 5500) {
                    stopOuttake();
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup1, true);
                        kicker.setPosition(0.6);
                        rightBarrier.setPosition(0.2);
                        actionTimer.resetTimer();
                        setPathState(2);
                    }
                } else if (milliseconds >= 5000 && atTargetSpeed) {
                    intake2.setPower(1);
                } else if (milliseconds >= 3800) {
                    intake2.setPower(0);
                } else if (milliseconds >= 3100 && atTargetSpeed) {
                    intake2.setPower(1);
                }
                break;

            case 2:
                if (distance <= 35) {
                    intake2.setPower(1);
                    follower.setMaxPowerScaling(0.25);
                    if (!follower.isBusy()) {
                        follower.followPath(scorePickup1, true);
                        follower.setMaxPowerScaling(1);
                        actionTimer.resetTimer();
                        setPathState(3);
                    }
                }
                break;

            case 3:
                startOuttake();
                rightBarrier.setPosition(0.6);
                kicker.setPosition(0);

                if (milliseconds >= 4600) {
                    stopOuttake();
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup2, true);
                        kicker.setPosition(0.6);
                        rightBarrier.setPosition(0.2);
                        actionTimer.resetTimer();
                        setPathState(4);
                    }
                } else if (milliseconds >= 3600 && atTargetSpeed) {
                    intake2.setPower(1);
                } else if (milliseconds >= 2500) {
                    intake2.setPower(0);
                } else if (milliseconds >= 2100 && atTargetSpeed) {
                    intake2.setPower(1);
                }
                break;

            case 4:
                if (distance <= 35) {
                    intake2.setPower(1);
                    follower.setMaxPowerScaling(0.35);
                    if (!follower.isBusy()) {
                        follower.followPath(scorePickup2, true);
                        follower.setMaxPowerScaling(1);
                        actionTimer.resetTimer();
                        setPathState(5);
                    }
                }
                break;

            case 5:
                startOuttake();
                rightBarrier.setPosition(0.6);
                kicker.setPosition(0);

                if (milliseconds >= 5100) {
                    stopOuttake();
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup3, true);
                        kicker.setPosition(0.6);
                        rightBarrier.setPosition(0.2);
                        actionTimer.resetTimer();
                        setPathState(6);
                    }
                } else if (milliseconds >= 4200 && atTargetSpeed) {
                    intake2.setPower(1);
                } else if (milliseconds >= 3200) {
                    intake2.setPower(0);
                } else if (milliseconds >= 2500 && atTargetSpeed) {
                    intake2.setPower(1);
                }
                break;

            case 6:
                if (distance <= 35) {
                    intake2.setPower(1);
                    follower.setMaxPowerScaling(0.35);
                    if (!follower.isBusy()) {
                        follower.followPath(scorePickup3, true);
                        follower.setMaxPowerScaling(1);
                        actionTimer.resetTimer();
                        setPathState(7);
                    }
                }
                break;

            case 7:
                startOuttake();
                rightBarrier.setPosition(0.6);
                kicker.setPosition(0);

                if (milliseconds >= 5200) {
                    stopOuttake();
                    if (!follower.isBusy()) {
                        kicker.setPosition(0.6);
                        rightBarrier.setPosition(0.2);
                        actionTimer.resetTimer();
                        setPathState(-1);
                    }
                } else if (milliseconds >= 4400 && atTargetSpeed) {
                    intake2.setPower(1);
                } else if (milliseconds >= 3800) {
                    intake2.setPower(0);
                } else if (milliseconds >= 3000 && atTargetSpeed) {
                    intake2.setPower(1);
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
        milliseconds = actionTimer.getElapsedTime();
        distance = follower.getDistanceRemaining();

        follower.update();
        autonomousPathUpdate();

        double currentVelocity = outtakeMotor.getVelocity();
        double currentRPM = (currentVelocity / OUTTAKE_CPR) * 60.0;

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("time", milliseconds);
        telemetry.addData("Distance", distance);
        telemetry.addData("Outtake RPM", currentRPM);
        telemetry.addData("Outtake Target RPM", OUTTAKE_RPM_TARGET);
        telemetry.addData("Outtake Velocity %", (currentVelocity / OUTTAKE_TARGET_VELOCITY) * 100);
        telemetry.addData("Outtake Started", outtakeStarted);
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Hardware mapping
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        kicker = hardwareMap.get(Servo.class, "kicker");
        rightBarrier = hardwareMap.get(Servo.class, "rightBarrier");

        // Configure outtake motor - DO NOT start it here
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // IMPORTANT: Do NOT set velocity or power here - motor should stay off until start()

        // Configure intake motors
        if (intakeMotor != null) {
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Ready to start");
        telemetry.addData("Outtake Motor", "Configured - OFF until start");
        telemetry.addData("Target RPM", OUTTAKE_RPM_TARGET);
        telemetry.addData("Target Velocity (ticks/s)", OUTTAKE_TARGET_VELOCITY);
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        actionTimer.resetTimer();

        // Reset outtake state
        outtakeStarted = false;

        // Reset outtake motor configuration
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure motor is off at start
        outtakeMotor.setPower(0);

        setPathState(0);
    }

    @Override
    public void stop() {
        if (outtakeMotor != null) {
            outtakeMotor.setPower(0);
        }
        if (intakeMotor != null) {
            intakeMotor.setPower(0);
        }
        if (intake2 != null) {
            intake2.setPower(0);
        }
    }
}