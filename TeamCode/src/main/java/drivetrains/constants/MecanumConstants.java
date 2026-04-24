package drivetrains.constants;

import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import drivetrains.Drivetrain;
import drivetrains.Mecanum;
import util.MotorMetaData;

/**
 * Mecanum drivetrain constants class
 *
 * @author Xander Haemel - 31616 404 Not Found
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class MecanumConstants extends DrivetrainConstants {
    // Motors
    public MotorMetaData flData = new MotorMetaData("front_left_drive");
    public MotorMetaData blData = new MotorMetaData("back_left_drive");
    public MotorMetaData frData = new MotorMetaData("front_right_drive");
    public MotorMetaData brData = new MotorMetaData("back_right_drive");

    // Miscellaneous constants
    public double maxPower = 1.0; // 0 to 1, max power to apply to the motors
    public double maxCurrent = -1.0; // Max total motor current in amps, negative for no limit
    public boolean robotCentric = true; // Whether to use robot-centric controls (true) or field-centric controls (false) in TeleOp

    /** Constructor for the MecanumConstants class */
    public MecanumConstants() {}

    @Override
    public Drivetrain build(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, this);
    }

    /**
     * Sets the left front motor name. Default: "front_left_drive"
     * @param name the name of the left front motor
     * @return this instance for chaining
     */
    public MecanumConstants setFrontLeftMotorName(String name) {
        this.flData.setName(name);
        return this;
    }

    /**
     * Sets the left rear motor name. Default: "back_left_drive"
     * @param name the name of the left rear motor
     * @return this instance for chaining
     */
    public MecanumConstants setBackLeftMotorName(String name) {
        this.blData.setName(name);
        return this;
    }

    /**
     * Sets the right front motor name. Default: "front_right_drive"
     * @param name the name of the right front motor
     * @return this instance for chaining
     */
    public MecanumConstants setFrontRightMotorName(String name) {
        this.frData.setName(name);
        return this;
    }

    /**
     * Sets the right rear motor name. Default: "back_right_drive"
     * @param name the name of the right rear motor
     * @return this instance for chaining
     */
    public MecanumConstants setBackRightMotorName(String name) {
        this.brData.setName(name);
        return this;
    }

    /**
     * Default direction is FORWARD.
     * @param reversed whether the front left motor is reversed
     * @return this instance for chaining
     */
    public MecanumConstants setFrontLeftReversed(boolean reversed) {
        this.flData.setDirection(reversed ? Direction.REVERSE : Direction.FORWARD);
        return this;
    }

    /**
     * Default direction is FORWARD.
     * @param reversed whether the back left motor is reversed
     * @return this instance for chaining
     */
    public MecanumConstants setBackLeftReversed(boolean reversed) {
        this.blData.setDirection(reversed ? Direction.REVERSE : Direction.FORWARD);
        return this;
    }

    /**
     * Default direction is FORWARD.
     * @param reversed whether the front right motor is reversed
     * @return this instance for chaining
     */
    public MecanumConstants setFrontRightReversed(boolean reversed) {
        this.frData.setDirection(reversed ? Direction.REVERSE : Direction.FORWARD);
        return this;
    }

    /**
     * Default direction is FORWARD.
     * @param reversed whether the back right motor is reversed
     * @return this instance for chaining
     */
    public MecanumConstants setBackRightReversed(boolean reversed) {
        this.brData.setDirection(reversed ? Direction.REVERSE : Direction.FORWARD);
        return this;
    }

    /**
     * Sets whether to use braking mode. Default: true (brake mode).
     * @param brakeMode true for brake mode, false for float mode
     * @return this instance for chaining
     */
    public MecanumConstants setBrakeMode(boolean brakeMode) {
        this.flData.setBrakeMode(brakeMode ? ZeroPowerBehavior.BRAKE : ZeroPowerBehavior.FLOAT);
        this.blData.setBrakeMode(brakeMode ? ZeroPowerBehavior.BRAKE : ZeroPowerBehavior.FLOAT);
        this.frData.setBrakeMode(brakeMode ? ZeroPowerBehavior.BRAKE : ZeroPowerBehavior.FLOAT);
        this.brData.setBrakeMode(brakeMode ? ZeroPowerBehavior.BRAKE : ZeroPowerBehavior.FLOAT);
        return this;
    }

    /**
     * Sets the maximum power.
     * @param maxPower the max power (0 to 1) to apply to the motors
     * @return this instance for chaining
     */
    public MecanumConstants setMaxPower(double maxPower) {
        this.maxPower = Math.max(0.0, Math.min(maxPower, 1.0)); // Ensure maxPower is between 0 and 1
        return this;
    }

    /**
     * Sets the maximum total motor current allowed in amps. Set to a negative value for no limit.
     * @param amps is the current limit in amps
     * @return this instance for chaining
     */
    public MecanumConstants setMaxCurrent(double amps){
        this.maxCurrent = amps;
        return this;
    }

    /**
     * Sets whether to use robot-centric or field-centric controls in TeleOp.
     * @param robotCentric true for robot-centric controls, false for field-centric controls
     * @return this instance for chaining
     */
    public MecanumConstants setRobotCentric(boolean robotCentric) {
        this.robotCentric = robotCentric;
        return this;
    }
}