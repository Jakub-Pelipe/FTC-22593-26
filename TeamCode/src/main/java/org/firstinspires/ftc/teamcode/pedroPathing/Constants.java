package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// propably something important ^
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(6);
// ASSUMED MASS OF BOT 5(KG) NEEEEED TO CHANGE^
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    // DRIVE TRAIN CONFIG FOR DIRECTION + POWER
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.8)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("rearRight")
            .leftRearMotorName("rearLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(51.646130659448815)
            .yVelocity(43.85592098686639);




// ACCORDING TO THE MANUEL WE MAY HAVE TO "REVERSE" THE MOTOR DIRECTION ^

// THIS IS USED TO GET OUR LOCATION WE USE PINPOINT
public static PinpointConstants localizerConstants = new PinpointConstants()
        .forwardPodY(8.202756)
        .strafePodX(8.202756)
        .distanceUnit(DistanceUnit.INCH)
        .hardwareMapName("pinpoint") // MAKE SURE TO ADD NAME TO DRIVE HUB
        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD) // WE ASSUME WE HAVE THIS MAY NEED TO DOUBLE CHECK
        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED); //untested cz biyon slow TO DO
// this is not a duplicate as some believed


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)










                .build();

        }

    }


