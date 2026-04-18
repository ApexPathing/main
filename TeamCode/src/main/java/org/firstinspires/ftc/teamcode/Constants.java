package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import drivetrains.constants.MecanumConstants;
import drivetrains.constants.TankConstants;
import localizers.constants.OTOSConstants;
import localizers.constants.PinpointConstants;
import followers.constants.P2PFollowerConstants;
import util.Angle;
import util.Distance;
import util.Pose;

/**
 * Constants file for testing
 * @author Dylan B. - 18597 RoboClovers - Delta
 * @author Xander Haemel -31616 - 404 Not Found
 */
@SuppressWarnings("unused")
public class Constants {
    public static MecanumConstants driveConstants = new MecanumConstants()
            .setFrontLeftMotorName("frontLeftMotor")
            .setBackLeftMotorName("backLeftMotor")
            .setFrontRightMotorName("frontRightMotor")
            .setBackRightMotorName("backRightMotor")
            .setFrontRightReversed(true)
            .setBackRightReversed(true)
            .setBrakeMode(true)
            .setRobotCentric(true)
            .setMaxPower(1.0);

    /** Example tank drive constants
    public static TankConstants tankDriveConstants = new TankConstants()
            .setFourMotorDrive(true)
            .setFrontLeftMotorName("leftFront")
            .setBackLeftMotorName("leftRear")
            .setFrontRightMotorName("rightFront")
            .setBackRightMotorName("rightRear")
            .setFrontRightReversed(true)
            .setBackRightReversed(true)
            .setBrakeMode(true)
            .setRobotCentric(true)
            .setMaxPower(0.5);
     **/

    /**
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .setName("pinpoint")
            .setDistanceUnit(DistanceUnit.INCH)
            .setAngleUnit(AngleUnit.DEGREES)
            .setXOffset(-3.31) // In distanceUnit // TODO: Auto offset tuner
            .setYOffset(-6.61) // In distanceUnit // TODO: Auto offset tuner
            .setXPodDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .setYPodDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    **/

    public static OTOSConstants localizerConstants = new OTOSConstants()
            .setName("otos")
            .setOffset(new Pose(227, -16, 0, Distance.Units.MILLIMETERS, Angle.Units.DEGREES))
            .setLinearScalar(1.05)
            .setHeadingScalar(1.0);

    public static P2PFollowerConstants followerConstants = new P2PFollowerConstants()
            .setTranslationalGain(0.05)
            .setTranslationalD(0)
            .setHeadingGain(0.35)
            .setHeadingD(0)
            .setTranslationalTolerance(1.0) // Inches
            .setHeadingTolerance(3.0) // Degrees
            .setMaxPower(0.5) // Power limits can be overwritten by the drivetrain's power limits, these are specifically for following
            .setMinPower(0.05); //TODO: tune FIRST (this is basically kS, so just increase until robot barely moves)
}