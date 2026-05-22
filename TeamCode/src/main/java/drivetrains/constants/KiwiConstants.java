package drivetrains.constants;

import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import drivetrains.Drivetrain;
import drivetrains.Kiwi;
import util.MotorMetaData;

/**
 * Kiwi drivetrain constants class
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class KiwiConstants extends DrivetrainConstants {
    // Motors
    public MotorMetaData frMotorData = new MotorMetaData("frMotor");
    public MotorMetaData bMotorData = new MotorMetaData("bMotor");
    public MotorMetaData flMotorData = new MotorMetaData("flMotor");

    // Miscellaneous constants
    public double maxPower = 1.0; // 0 to 1, max power to apply to the motors
    public double maxCurrent = -1.0; // Max total motor current in amps, negative for no limit
    public boolean robotCentric = true; // Whether to use robot-centric controls (true) or field-centric controls (false) in TeleOp

    /** Constructor for the MecanumConstants class */
    public KiwiConstants() {}

    @Override
    public Drivetrain build(HardwareMap hardwareMap) { return new Kiwi(hardwareMap, this); }

    /**
     * Sets the first drive motor name. Default: "frMotor"
     * @param name the name of the first drive motor
     * @return this instance for chaining
     */
    public KiwiConstants setFrontRightMotorName(String name) {
        this.frMotorData.setName(name);
        return this;
    }

    /**
     * Sets the second drive motor name. Default: "bMotor"
     * @param name the name of the second drive motor
     * @return this instance for chaining
     */
    public KiwiConstants setBackMotorName(String name) {
        this.bMotorData.setName(name);
        return this;
    }

    /**
     * Sets the third drive motor name. Default: "flMotor"
     * @param name the name of the third drive motor
     * @return this instance for chaining
     */
    public KiwiConstants setFrontLeftMotorName(String name) {
        this.flMotorData.setName(name);
        return this;
    }

    /**
     * Default direction is FORWARD.
     * @param reversed whether the first drive motor is reversed
     * @return this instance for chaining
     */
    public KiwiConstants setFrontRightMotorReversed(boolean reversed) {
        this.frMotorData.setDirection(reversed ? Direction.REVERSE : Direction.FORWARD);
        return this;
    }

    /**
     * Default direction is FORWARD.
     * @param reversed whether the second drive motor is reversed
     * @return this instance for chaining
     */
    public KiwiConstants setBackMotorReversed(boolean reversed) {
        this.bMotorData.setDirection(reversed ? Direction.REVERSE : Direction.FORWARD);
        return this;
    }

    /**
     * Default direction is FORWARD.
     * @param reversed whether the third drive motor is reversed
     * @return this instance for chaining
     */
    public KiwiConstants setFrontLeftMotorReversed(boolean reversed) {
        this.flMotorData.setDirection(reversed ? Direction.REVERSE : Direction.FORWARD);
        return this;
    }

    /**
     * Sets whether to use braking mode. Default: true (brake mode).
     * @param brakeMode true for brake mode, false for float mode
     * @return this instance for chaining
     */
    public KiwiConstants setBrakeMode(boolean brakeMode) {
        this.frMotorData.setBrakeMode(brakeMode ? ZeroPowerBehavior.BRAKE : ZeroPowerBehavior.FLOAT);
        this.bMotorData.setBrakeMode(brakeMode ? ZeroPowerBehavior.BRAKE : ZeroPowerBehavior.FLOAT);
        this.flMotorData.setBrakeMode(brakeMode ? ZeroPowerBehavior.BRAKE : ZeroPowerBehavior.FLOAT);
        return this;
    }

    /**
     * Sets the maximum power.
     * @param maxPower the max power (0 to 1) to apply to the motors
     * @return this instance for chaining
     */
    public KiwiConstants setMaxPower(double maxPower) {
        this.maxPower = Math.max(0.0, Math.min(maxPower, 1.0)); // Ensure maxPower is between 0 and 1
        return this;
    }

    /**
     * Sets the maximum total motor current allowed in amps. Set to a negative value for no limit.
     * @param amps is the current limit in amps
     * @return this instance for chaining
     */
    public KiwiConstants setMaxCurrent(double amps){
        this.maxCurrent = amps;
        return this;
    }

    /**
     * Sets whether to use robot-centric or field-centric controls in TeleOp.
     * @param robotCentric true for robot-centric controls, false for field-centric controls
     * @return this instance for chaining
     */
    public KiwiConstants setRobotCentric(boolean robotCentric) {
        this.robotCentric = robotCentric;
        return this;
    }

    /**
     * @return The user-defined MotorMetaData for the first drive motor.
     */
    public MotorMetaData getFrontRightMotorData() {
        return frMotorData;
    }

    /**
     * @return The user-defined MotorMetaData for the second drive motor.
     */
    public MotorMetaData getBackMotorData() {
        return bMotorData;
    }

    /**
     * @return The user-defined MotorMetaData for the third drive motor.
     */
    public MotorMetaData getFrontLeftMotorData() {
        return flMotorData;
    }
}