package org.firstinspires.ftc.teamcode;

import core.ApexBuilder;
import drivetrains.constants.DrivetrainConstants;
import drivetrains.constants.MecanumConstants;
import localizers.constants.LocalizerConstants;
import localizers.constants.OTOSConstants;
import followers.constants.FollowerConstants;
import followers.constants.P2PFollowerConstants;
import util.Angle;
import util.Distance;
import util.Pose;

/**
 * This class extends {@link ApexBuilder} and provides the specific constants for the drivetrain,
 * localizer, and follower that we want to use in our OpMode. In this example, we are using a
 * mecanum drivetrain, an OTOS localizer, and a point-to-point follower. You can modify the values in
 * the setDrivetrainConstants(), setLocalizerConstants(), and setFollowerConstants() methods to fit
 * your robot's hardware and tuning preferences.
 *
 * @author Dylan B. 18597 RoboClovers - Delta
 */
public class Constants extends ApexBuilder {
    @Override
    public DrivetrainConstants setDrivetrainConstants() { // Any DrivetrainConstants
        return new MecanumConstants()
                .setFrontLeftMotorName("frontLeftMotor")
                .setBackLeftMotorName("backLeftMotor")
                .setFrontRightMotorName("frontRightMotor")
                .setBackRightMotorName("backRightMotor")
                .setFrontRightReversed(true)
                .setBackRightReversed(true)
                .setBrakeMode(true)
                .setRobotCentric(true)
                .setMaxPower(1.0);
    }

    @Override
    public LocalizerConstants setLocalizerConstants() { // Any LocalizerConstants
        return new OTOSConstants()
                .setName("otos")
                .setOffset(new Pose(227, -16, 0, Distance.Units.MILLIMETERS, Angle.Units.DEGREES))
                .setLinearScalar(1.05)
                .setHeadingScalar(1.0);
    }

    @Override
    public FollowerConstants setFollowerConstants() { // Any FollowerConstants
        return new P2PFollowerConstants()
                .setTranslationalGain(0.05)
                .setTranslationalD(0)
                .setHeadingGain(1)
                .setHeadingD(0.05)
                .setTranslationalTolerance(67) // Inches
                .setHeadingTolerance(3.0) // Degrees
                .setMaxPower(0.5) // Specifically for following, not drivetrain max power
                .setMinPower(0.04);
    }
}

/* Tank drivetrain constants
new TankConstants()
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
 */

/* Swerve drivetrain constants
new SwerveConstants()
        .setFrontLeftModuleConstants(
                new SwerveModuleConstants()
                        .setMotorName("frontLeftMotor")
                        .setServoName("flServo")
                        .setEncoderName("flEncoder")
                        .setMotorReversed(false)
        )
        .setFrontRightModuleConstants(
                new SwerveModuleConstants()
                        .setMotorName("frontRightMotor")
                        .setServoName("frServo")
                        .setEncoderName("frEncoder")
                        .setMotorReversed(true)
        )
        .setBackLeftModuleConstants(
                new SwerveModuleConstants()
                        .setMotorName("backLeftMotor")
                        .setServoName("blServo")
                        .setEncoderName("blEncoder")
                        .setMotorReversed(false)
        )
        .setBackRightModuleConstants(
                new SwerveModuleConstants()
                        .setMotorName("backRightMotor")
                        .setServoName("brServo")
                        .setEncoderName("brEncoder")
                        .setMotorReversed(true)
        )
        .setMaxPower(1.0)
        .setTrackWidth(Distance.fromMm(300))
        .setWheelbase(Distance.fromMm(300))
        .setRobotCentric(true);
*/

/* Pinpoint constants
new PinpointConstants()
        .setName("pinpoint")
        .setDistanceUnit(DistanceUnit.INCH)
        .setAngleUnit(AngleUnit.DEGREES)
        .setXOffset(-3.31) // In distanceUnit
        .setYOffset(-6.61) // In distanceUnit
        .setXPodDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
        .setYPodDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
        .setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
*/