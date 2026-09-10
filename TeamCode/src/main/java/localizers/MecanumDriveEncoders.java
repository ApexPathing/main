package localizers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.json.JSONObject;

import geometry.Angle;
import geometry.Pose;

/**
 * A localizer that uses 4 drive encoders and an IMU
 *
 * @author Topher F. - 23571 alum
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class MecanumDriveEncoders extends BaseLocalizer<MecanumDriveEncoders.Constants> {
    private final OdometryPod frontLeft, frontRight, backLeft, backRight;
    private final IMU imu;

    private double correction = 0.0;

    public MecanumDriveEncoders(Constants constants, HardwareMap hardwareMap) {
        super(constants);

        frontLeft = new OdometryPod(hardwareMap, constants.frontLeftName, config.ticksPerInch);
        frontRight = new OdometryPod(hardwareMap, constants.frontRightName, config.ticksPerInch);
        if (constants.backLeftName != null && constants.backRightName != null) {
            backLeft = new OdometryPod(hardwareMap, constants.backLeftName, config.ticksPerInch);
            backRight = new OdometryPod(hardwareMap, constants.backRightName, config.ticksPerInch);
        } else {
            backLeft = null;
            backRight = null;
        }

        imu = hardwareMap.get(IMU.class, constants.imuName);
        imu.initialize(new IMU.Parameters(constants.hubOrientation));
    }

    @Override
    public void update() {
        frontLeft.update();
        frontRight.update();
        if (backLeft != null) {
            backLeft.update();
            backRight.update();
        }
        double blDelta = 0;
        double brDelta = 0;
        if (backLeft != null) {
            blDelta = backLeft.getDeltaInches();
            brDelta = backRight.getDeltaInches();
        }

        int encoderCount = backLeft == null ? 2 : 4;
        double forward = (frontLeft.getDeltaInches() + frontRight.getDeltaInches() +
                blDelta + brDelta) / encoderCount;
        double strafe = backLeft == null ? 0.0 :
                (-frontLeft.getDeltaInches() + frontRight.getDeltaInches() +
                        blDelta - brDelta) / 4.0;

        double oldYaw = pose.getHeading(geometry.AngleUnit.RAD);
        double currentYaw = Angle.normalize(
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - correction
        );
        double deltaYaw = Angle.wrap(currentYaw - oldYaw);
        pose = integrateArc(pose, forward, strafe, deltaYaw, currentYaw);

        calculate(UpdateType.BOTH);
    }

    /** Returns raw wheel encoder positions in front-left, front-right, back-left, back-right order. */
    public int[] getEncoderTicks() {
        return new int[] {frontLeft.getTicks(), frontRight.getTicks(),
                backLeft == null ? 0 : backLeft.getTicks(),
                backRight == null ? 0 : backRight.getTicks()};
    }

    @Override
    public void setPose(Pose newPose) {
        correction = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) -
                newPose.getHeading(geometry.AngleUnit.RAD);
        pose = newPose;
        frontLeft.reset();
        frontRight.reset();
        if (backLeft != null) {
            backLeft.reset();
            backRight.reset();
        }
        resetKinematicEstimate(newPose);
    }

    public static class Constants implements BaseLocalizerConstants<Constants> {
        public String frontLeftName;
        public String frontRightName;
        public String backLeftName;
        public String backRightName;
        public String imuName;
        public RevHubOrientationOnRobot hubOrientation;
        public double ticksPerInch = 1.0;

        @Override
        public JSONObject getCalibrationValues() {
            try { return new JSONObject().put("ticksPerInch", ticksPerInch); }
            catch (Exception e) { throw new IllegalStateException(e); }
        }

        @Override
        public void applyCalibrationValues(JSONObject values) {
            ticksPerInch = CalibrationJson.positive(values, "ticksPerInch", ticksPerInch);
        }

        @Override
        public BaseLocalizer<?> build(HardwareMap hardwareMap) {
            if (frontLeftName == null || frontRightName == null) {
                throw new IllegalArgumentException(
                        "You must call setFrontLeftName and setFrontRightName to set the names " +
                                "of the motors that the encoders are attached to. For " +
                                "drivetrains that are not 2 wheel tank, you must also call " +
                                "setBackLeftName and setBackRightName."
                );
            }

            if (imuName == null) {
                throw new IllegalArgumentException(
                        "You must call setIMUName to set the name of the control hub IMU."
                );
            }

            if (this.hubOrientation == null) {
                // noinspection ConstantExpression
                throw new IllegalArgumentException(
                        "You must call setHubOrientation to set the orientation of the control " +
                                "hub on the robot."
                );
            }

            return new MecanumDriveEncoders(this, hardwareMap);
        }

        /** Sets the name of the motor that the front left encoder is attached to. */
        public Constants setFrontLeftName(String name) {
            this.frontLeftName = name;
            return this;
        }

        /** Sets the name of the motor that the front right encoder is attached to. */
        public Constants setFrontRightName(String name) {
            this.frontRightName = name;
            return this;
        }

        /**
         * Sets the name of the motor that the back left encoder is attached to. For 2 wheel tank
         * drivetrains, don't set the back encoders, only the front.
         */
        public Constants setBackLeftName(String name) {
            this.backLeftName = name;
            return this;
        }

        /**
         * Sets the name of the motor that the back right encoder is attached to. For 2 wheel tank
         * drivetrains, don't set the back encoders, only the front.
         */
        public Constants setBackRightName(String name) {
            this.backRightName = name;
            return this;
        }

        /** Sets the name of the control hub IMU that is used to measure heading */
        public Constants setIMUName(String name) {
            this.imuName = name;
            return this;
        }

        /** Sets the orientation of the control hub on the robot. */
        public Constants setHubOrientation(RevHubOrientationOnRobot orientation) {
            this.hubOrientation = orientation;
            return this;
        }

        /** Sets the number of encoder ticks per inch of travel. */
        public Constants setTicksPerInch(double ticksPerInch) {
            this.ticksPerInch = ticksPerInch;
            return this;
        }
    }
}
