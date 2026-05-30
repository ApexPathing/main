package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import controllers.PDSController;
import core.ApexBuilder;
import drivetrains.constants.DrivetrainConstants;
import drivetrains.constants.MecanumConstants;
import localizers.constants.LocalizerConstants;
import localizers.constants.PinpointConstants;
import followers.constants.FollowerConstants;
import followers.constants.P2PFollowerConstants;
import geometry.Angle;
import geometry.Dist;

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
        return new PinpointConstants()
                .setName("pinpoint")
                .setDistanceUnit(DistanceUnit.INCH)
                .setAngleUnit(AngleUnit.DEGREES)
                .setXOffset(0.0) // In distanceUnit
                .setYOffset(0.0) // In distanceUnit
                .setXPodDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                .setYPodDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                .setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    }

    @Override
    public FollowerConstants setFollowerConstants() { // Any FollowerConstants
        return new P2PFollowerConstants()
                .setAxialCoeffs(new PDSController.PDSCoefficients(0.0, 0.0, 0.0, 0.0))
                .setStrafeCoeffs(new PDSController.PDSCoefficients(0.0, 0.0, 0.0, 0.0))
                .setHeadingCoeffs(new PDSController.PDSCoefficients(0.0, 0.0, 0.0, 0.0))
                .setHeadingTolerance(Angle.fromDeg(2.0))
                .setAxialTolerance(Dist.fromIn(1.5))
                .setStrafeTolerance(Dist.fromIn(1.5))
                .setMaxAxialPower(1)
                .setStrafeTolerance(Dist.fromIn(1.5))
                .setMaxStrafePower(1)
                .setMaxTurnPower(1);
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
SwerveConstants()
                .setFrontLeftModuleConstants(
                        new SwerveModuleConstants()
                                .setMotorName("frontLeftMotor")
                                .setServoName("flServo")
                                .setEncoderName("flEncoder")
                                .setMotorReversed(false)
                                .setModuleAngleOffset(0) //degrees
                                .setMaxEncoderVoltage(3.3)
                                .setSteering_kP_val(0.1)
                )
                .setFrontRightModuleConstants(
                        new SwerveModuleConstants()
                                .setMotorName("frontRightMotor")
                                .setServoName("frServo")
                                .setEncoderName("frEncoder")
                                .setMotorReversed(true)
                                .setModuleAngleOffset(0) //degrees
                                .setMaxEncoderVoltage(3.3)
                                .setSteering_kP_val(0.1)
                )
                .setBackLeftModuleConstants(
                        new SwerveModuleConstants()
                                .setMotorName("backLeftMotor")
                                .setServoName("blServo")
                                .setEncoderName("blEncoder")
                                .setMotorReversed(false)
                                .setModuleAngleOffset(0) //degrees
                                .setMaxEncoderVoltage(3.3)
                                .setSteering_kP_val(0.1)
                )
                .setBackRightModuleConstants(
                        new SwerveModuleConstants()
                                .setMotorName("backRightMotor")
                                .setServoName("brServo")
                                .setEncoderName("brEncoder")
                                .setMotorReversed(true)
                                .setModuleAngleOffset(0) //degrees
                                .setMaxEncoderVoltage(3.3)
                                .setSteering_kP_val(0.1)
                )
                .setMaxPower(1.0)
                .setTrackWidth(Dist.fromMm(0))
                .setWheelbase(Dist.fromMm(0))
                .setRobotCentric(true);
    }*/

/* Kiwi drivetrain constants
return new KiwiConstants()
                .setFrontRightMotorName("frMotor")
                .setBackMotorName("bMotor")
                .setFrontLeftMotorName("flMotor")
                .setMaxPower(1.0)
                .setRobotCentric(true);
*/

/* OTOS Constants
new OTOSConstants() // Tuned for Dylan + Mikey strafer chassis with OTOS, don't change these
    .setName("otos")
    .setOffset(new Pose(227, -16, 0, Distance.Units.MILLIMETERS, Angle.Units.DEGREES))
    .setLinearScalar(1.05)
    .setHeadingScalar(1.0);
*/