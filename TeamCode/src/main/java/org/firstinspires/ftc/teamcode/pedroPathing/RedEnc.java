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
import com.qualcomm.robotcore.hardware.DcMotorEx;     // Import DcMotorEx
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "RedEnc", group = "Autonomous")
public class RedEnc extends OpMode {
    // Motor constants – adjust to your hardware
    private static final double OUTTAKE_CPR = 28;           // Counts per revolution of the output shaft (e.g., 28 for REV/NeveRest with no extra gearbox)
    private static final double OUTTAKE_RPM_TARGET = 300;   // Desired outtake speed in RPM

    private Follower follower;
    public double milliseconds;
    public double distance;
    private DcMotorEx outtakeMotor;    // Use DcMotorEx for velocity control
    private DcMotor intake2;
    private Servo kicker;
    private Servo rightBarrier;
    public Timer pathTimer, opmodeTimer, actionTimer;
    private int pathState;

    // Pose definitions (unchanged)
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
        setOuttakeSpeed(OUTTAKE_RPM_TARGET);
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
                    outtakeMotor.setPower(0);
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup1, true);
                        kicker.setPosition(0.6);
                        rightBarrier.setPosition(0.2);
                        actionTimer.resetTimer();
                        setPathState(2);
                    }
                } else if (milliseconds >= 5000) {
                    intake2.setPower(1);
                } else if (milliseconds >= 3800) {
                    intake2.setPower(0);
                } else if (milliseconds >= 3100) {
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
                    outtakeMotor.setPower(0);
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup2, true);
                        kicker.setPosition(0.6);
                        rightBarrier.setPosition(0.2);
                        actionTimer.resetTimer();
                        setPathState(4);
                    }
                } else if (milliseconds >= 3600) {
                    intake2.setPower(1);
                } else if (milliseconds >= 2500) {
                    intake2.setPower(0);
                } else if (milliseconds >= 2100) {
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
                    outtakeMotor.setPower(0);
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup3, true);
                        kicker.setPosition(0.6);
                        rightBarrier.setPosition(0.2);
                        actionTimer.resetTimer();
                        setPathState(6);
                    }
                } else if (milliseconds >= 4200) {
                    intake2.setPower(1);
                } else if (milliseconds >= 3200) {
                    intake2.setPower(0);
                } else if (milliseconds >= 2500) {
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
                    outtakeMotor.setPower(0);
                    if (!follower.isBusy()) {
                        kicker.setPosition(0.6);
                        rightBarrier.setPosition(0.2);
                        actionTimer.resetTimer();
                        setPathState(-1);
                    }
                } else if (milliseconds >= 4400) {
                    intake2.setPower(1);
                } else if (milliseconds >= 3800) {
                    intake2.setPower(0);
                } else if (milliseconds >= 3000) {
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

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("time", milliseconds);
        telemetry.addData("Distance", distance);
        telemetry.addData("Outtake velocity (ticks/s)", outtakeMotor.getVelocity());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Hardware mapping – note: outtakeMotor is cast to DcMotorEx
        outtakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "outtakeMotor");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        kicker = hardwareMap.get(Servo.class, "kicker");
        rightBarrier = hardwareMap.get(Servo.class, "rightBarrier");

        // Configure outtake motor for encoder control (official REV method)
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // intake2 can be left as default (RUN_WITHOUT_ENCODER) if you only need power control
        // But if you want, you can also set it up similarly:
        // intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // default

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}