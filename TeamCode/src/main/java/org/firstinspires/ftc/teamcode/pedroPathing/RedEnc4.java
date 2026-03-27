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
    // 28 pulses per revolution (from encoder), 288 RPM max speed
    // Ticks per revolution = 28 * 4 (quadrature) = 112 ticks per revolution
    // Max velocity in ticks per second = (288 RPM / 60) * 112 = 537.6 ticks/second
    private final double OUTTAKE_TARGET_VELOCITY = 500; // Set slightly below max for reliability
    private final double OUTTAKE_HOLD_POWER = 0.8; // Power to maintain speed when running
    private final double OUTTAKE_IDLE_POWER = 0.3; // Power to keep motor engaged but not overspeed

    private final Pose startPose = new Pose(133.752, 134.543, Math.toRadians(270));
    private final Pose scorePose = new Pose(92, 91.396, Math.toRadians(40));
    private final Pose pickup1Pose = new Pose(140.6, 97.029, Math.toRadians(0));
    private final Pose control1 = new Pose(68.465, 57.191);
    private final Pose control2 = new Pose(75.685, 43.519);
    private final Pose pickup2Pose = new Pose(145.6, 71.761, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(129.6, 27.304, Math.toRadians(0));

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
        switch (pathState) {
            case 0:
                // Start moving to score preload
                follower.followPath(scorePreload);
                actionTimer.resetTimer();
                pathState = 1;
                break;

            case 1:
                // Score preload and prepare for first pickup
                outtakeMotor.setPower(OUTTAKE_HOLD_POWER);
                kicker.setPosition(0);
                rightBarrier.setPosition(0.6);

                if (milliseconds >= 5500 && TPS >= OUTTAKE_TARGET_VELOCITY * 0.85) {
                    outtakeMotor.setPower(0);
                    intake2.setPower(0);
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup1, true);
                        kicker.setPosition(0.6);
                        rightBarrier.setPosition(0.2);
                        actionTimer.resetTimer();
                        setPathState(2);
                    }
                } else if (milliseconds >= 5000 && TPS >= OUTTAKE_TARGET_VELOCITY * 0.85) {
                    intakeMotor.setPower(-1);
                    if (TPS > 20) {
                        intake2.setPower(1);
                    }
                } else if (milliseconds >= 3800) {
                    intake2.setPower(0);
                    intakeMotor.setPower(0);
                } else if (milliseconds >= 3100 && TPS >= OUTTAKE_TARGET_VELOCITY * 0.85) {
                    intakeMotor.setPower(-1);
                    if (TPS > 20) {
                        intake2.setPower(1);
                    }
                }
                break;

            case 2:
                // Grab first pickup
                if (distance <= 35) {
                    intakeMotor.setPower(-1);
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
                // Score first pickup
                outtakeMotor.setPower(OUTTAKE_HOLD_POWER);
                rightBarrier.setPosition(0.6);
                kicker.setPosition(0);

                if (milliseconds >= 4600 && TPS >= OUTTAKE_TARGET_VELOCITY * 0.85) {
                    outtakeMotor.setPower(0);
                    intake2.setPower(0);
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup2, true);
                        kicker.setPosition(0.6);
                        rightBarrier.setPosition(0.2);
                        actionTimer.resetTimer();
                        setPathState(4);
                    }
                } else if (milliseconds >= 3600 && TPS >= OUTTAKE_TARGET_VELOCITY * 0.85) {
                    intakeMotor.setPower(-1);
                    if (TPS > 20) {
                        intake2.setPower(1);
                    }
                } else if (milliseconds >= 2500) {
                    intake2.setPower(0);
                    intakeMotor.setPower(0);
                } else if (milliseconds >= 2100 && TPS >= OUTTAKE_TARGET_VELOCITY * 0.85) {
                    intakeMotor.setPower(-1);
                    if (TPS > 20) {
                        intake2.setPower(1);
                    }
                }
                break;

            case 4:
                // Grab second pickup
                if (distance <= 35) {
                    follower.setMaxPowerScaling(0.35);
                    if (TPS > 20) {
                        intakeMotor.setPower(-1);

                        if (!follower.isBusy()) {
                            follower.followPath(scorePickup2, true);
                            intakeMotor.setPower(0);
                            follower.setMaxPowerScaling(1);
                            actionTimer.resetTimer();
                            setPathState(5);
                        }
                    }
                }
                break;

            case 5:
                // Score second pickup
                outtakeMotor.setPower(OUTTAKE_HOLD_POWER);
                rightBarrier.setPosition(0.6);
                kicker.setPosition(0);

                if (milliseconds >= 5100 && TPS >= OUTTAKE_TARGET_VELOCITY * 0.85) {
                    outtakeMotor.setPower(0);
                    intake2.setPower(0);
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup3, true);
                        kicker.setPosition(0.6);
                        rightBarrier.setPosition(0.2);
                        actionTimer.resetTimer();
                        setPathState(6);
                    }
                } else if (milliseconds >= 4200 && TPS >= OUTTAKE_TARGET_VELOCITY * 0.85) {
                    intakeMotor.setPower(-1);
                    if (TPS > 20) {
                        intake2.setPower(1);
                    }
                } else if (milliseconds >= 3200) {
                    intake2.setPower(0);
                    intakeMotor.setPower(0);
                } else if (milliseconds >= 2500 && TPS >= OUTTAKE_TARGET_VELOCITY * 0.85) {
                    intakeMotor.setPower(-1);
                    if (TPS > 20) {
                        intake2.setPower(1);
                    }
                }
                break;

            case 6:
                // Grab third pickup
                if (distance <= 35) {
                    follower.setMaxPowerScaling(0.35);
                    if (TPS > 20) {
                        intakeMotor.setPower(-1);

                        if (!follower.isBusy()) {
                            follower.followPath(scorePickup3, true);
                            intakeMotor.setPower(0);
                            follower.setMaxPowerScaling(1);
                            actionTimer.resetTimer();
                            setPathState(7);
                        }
                    }
                }
                break;

            case 7:
                // Score third pickup
                outtakeMotor.setPower(0.7);
                rightBarrier.setPosition(0.6);
                kicker.setPosition(0);

                if (milliseconds >= 5200 && TPS >= OUTTAKE_TARGET_VELOCITY * 0.85) {
                    outtakeMotor.setPower(0);
                    intake2.setPower(0);
                    if (!follower.isBusy()) {
                        kicker.setPosition(0.6);
                        rightBarrier.setPosition(0.2);
                        actionTimer.resetTimer();
                        intakeMotor.setPower(0);
                        setPathState(-1);
                    }
                } else if (milliseconds >= 4400 && TPS >= OUTTAKE_TARGET_VELOCITY * 0.85) {
                    intakeMotor.setPower(-1);
                    if (TPS > 20) {
                        intake2.setPower(1);
                    }
                } else if (milliseconds >= 3800) {
                    intake2.setPower(0);
                    intakeMotor.setPower(0);
                } else if (milliseconds >= 3000 && TPS >= OUTTAKE_TARGET_VELOCITY * 0.85) {
                    intakeMotor.setPower(-1);
                    if (TPS > 20) {
                        intake2.setPower(1);
                    }
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

        // Update TPS continuously - get the actual current velocity
        TPS = outtakeMotor.getVelocity();

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
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

        // Initialize motors
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");

        // Initialize servos
        kicker = hardwareMap.get(Servo.class, "kicker");
        rightBarrier = hardwareMap.get(Servo.class, "rightBarrier");

        // Configure REV Core Hex Motor for outtake
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD); // Adjust if needed
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor.setVelocity(OUTTAKE_TARGET_VELOCITY);
        outtakeMotor.setPower(OUTTAKE_HOLD_POWER);

        // Configure intake motors
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE); // Adjust based on your setup
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE); // Adjust based on your setup
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {
        // Display initialization status
        telemetry.addData("Status", "Initializing...");
        telemetry.addData("Outtake Motor", "Configured");
        telemetry.addData("Target Velocity", OUTTAKE_TARGET_VELOCITY);
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        actionTimer.resetTimer();

        // Ensure outtake motor is ready before starting
        outtakeMotor.setVelocity(OUTTAKE_TARGET_VELOCITY);
        outtakeMotor.setPower(OUTTAKE_HOLD_POWER);

        setPathState(0);
    }

    @Override
    public void stop() {
        // Stop all motors when opmode ends
        if (outtakeMotor != null) outtakeMotor.setPower(0);
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (intake2 != null) intake2.setPower(0);

        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}