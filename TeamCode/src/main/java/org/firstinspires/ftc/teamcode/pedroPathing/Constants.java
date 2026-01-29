package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
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


public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(15.4) //kg

            .forwardZeroPowerAcceleration(-58.52125)
            .lateralZeroPowerAcceleration(-115.77)

            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.002, 0.05))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.005, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.0003, 1, 0.01))

            .centripetalScaling(0.000005);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.975)
            .leftFrontMotorName("flWheel")
            .rightFrontMotorName("frWheel")
            .leftRearMotorName("blWheel")
            .rightRearMotorName("brWheel")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)

            .xVelocity(88.3642)
            .yVelocity(66.46)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0)
            .strafePodX(4.7)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.8, 1);


    public static Follower createFollower(HardwareMap hardwareMap)
    {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
