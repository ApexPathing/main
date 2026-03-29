package Tests;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import Drivetrains.Constants.MecanumConstants;
import Localizers.Constants.PinpointConstants;

/**
 * Constants file for testing
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class Constants {
    public static MecanumConstants driveConstants = new MecanumConstants()
            .setLeftFrontMotorName("leftFront")
            .setLeftRearMotorName("leftRear")
            .setRightFrontMotorName("rightFront")
            .setRightRearMotorName("rightRear")
            .setLeftFrontDirection(DcMotorSimple.Direction.FORWARD)
            .setLeftRearDirection(DcMotorSimple.Direction.FORWARD)
            .setRightFrontDirection(DcMotorSimple.Direction.REVERSE)
            .setRightRearDirection(DcMotorSimple.Direction.REVERSE)
            .setUseBrakingMode(true)
            .setRobotCentric(true)
            .setMaxPower(0.5);

    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .setName("pinpoint")
            .setDistanceUnit(DistanceUnit.INCH)
            .setAngleUnit(AngleUnit.DEGREES)
            .setXOffset(-3.31) // In distanceUnit
            .setYOffset(-6.61) // In distanceUnit
            .setXPodDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .setYPodDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
}
