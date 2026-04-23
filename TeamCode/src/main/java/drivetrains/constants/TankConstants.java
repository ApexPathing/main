package drivetrains.constants;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import drivetrains.Tank;
import util.MotorMetaData;

/**
 * Tank drivetrain constants class
 * @author Xander Haemel - 31616- 404 Not Found
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class TankConstants extends DrivetrainConstants {
    // Motors
    public MotorMetaData flData = new MotorMetaData("front_left_drive");
    public MotorMetaData blData = new MotorMetaData("back_left_drive");
    public MotorMetaData frData = new MotorMetaData("front_right_drive");
    public MotorMetaData brData = new MotorMetaData("back_right_drive");

    // Tunable constants TODO: USE THESE
    public final double forwardVelocity = 60; // Inches per second, 60 ips default
    public final double headingVelocity = 6.82; // Radians per second, 1rps (2pi rad) default

    // Miscellaneous constants
    public double maxPower = 1.0; // 0 to 1, max power to apply to the motors
    public boolean fourMotor = true; // Four motor tank (true) or two motor tank (false)
    public boolean brakeMode = false; // Brake mode (true) or coast mode (false) for the motors
    public boolean feedForward = true; // Whether to use feedforward in the velocity controller TODO: USE THIS
    public boolean robotCentric = true; // Whether to use robot-centric controls (true) or field-centric controls (false) in TeleOp

    /**
     * Constructor for the TankConstants class
     */
    public TankConstants() {}

    @Override
    public Tank build(HardwareMap hardwareMap) {
        return new Tank(hardwareMap, this);
    }

    /**
     * Sets the left front motor name. Default: "front_left_drive"
     * @param name the name of the left front motor
     * @return this instance for chaining
     */
    public TankConstants setFrontLeftMotorName(String name) {
        this.flData.setName(name);
        return this;
    }
    /**
     * Sets the left rear motor name. Default: "back_left_drive"
     * @param name the name of the left rear motor
     * @return this instance for chaining
     */
    public TankConstants setBackLeftMotorName(String name) {
        this.blData.setName(name);
        return this;
    }
    /**
     * Sets the right front motor name. Default: "front_right_drive"
     * @param name the name of the right front motor
     * @return this instance for chaining
     */
    public TankConstants setFrontRightMotorName(String name) {
        this.frData.setName(name);
        return this;
    }
    /**
     * Sets the right rear motor name. Default: "back_right_drive"
     * @param name the name of the right rear motor
     * @return this instance for chaining
     */
    public TankConstants setBackRightMotorName(String name) {
        this.brData.setName(name);
        return this;
    }

    /**
     * Default direction is FORWARD.
     * @param reversed whether the front left motor is reversed
     * @return this instance for chaining
     */
    public TankConstants setFrontLeftReversed(boolean reversed) {
        this.flData.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        return this;
    }
    /**
     * Default direction is FORWARD.
     * @param reversed whether the back left motor is reversed
     * @return this instance for chaining
     */
    public TankConstants setBackLeftReversed(boolean reversed) {
        this.blData.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        return this;
    }
    /**
     * Default direction is FORWARD.
     * @param reversed whether the front right motor is reversed
     * @return this instance for chaining
     */
    public TankConstants setFrontRightReversed(boolean reversed) {
        this.frData.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        return this;
    }
    /**
     * Default direction is FORWARD.
     * @param reversed whether the back right motor is reversed
     * @return this instance for chaining
     */
    public TankConstants setBackRightReversed(boolean reversed) {
        this.brData.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        return this;
    }

    /**
     * Sets the maximum power.
     * @param maxPower the max power (0 to 1) to apply to the motors
     * @return this instance for chaining
     */
    public TankConstants setMaxPower(double maxPower) {
        this.maxPower = Math.max(0.0, Math.min(maxPower, 1.0)); // Ensure maxPower is between 0 and 1
        return this;
    }

    /**
     * Sets whether to use four motor (true) or two motor tank drive (false). Default: true
     * @param fourMotor whether to use four motor tank drive
     * @return this instance for chaining
     */
    public TankConstants setFourMotorDrive(boolean fourMotor) {
        this.fourMotor = fourMotor;
        return this;
    }

    /**
     * Sets whether to use brake mode. Default: true (brake mode).
     * @param brakeMode true for brake mode, false for float mode
     * @return this instance for chaining
     */
    public TankConstants setBrakeMode(boolean brakeMode) {
        this.flData.setBrakeMode(brakeMode ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        this.blData.setBrakeMode(brakeMode ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        this.frData.setBrakeMode(brakeMode ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        this.brData.setBrakeMode(brakeMode ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        return this;
    }

    /**
     * Sets whether to use feedforward in the velocity controller.
     * @param useFeedForward true to use feedforward, false otherwise
     * @return this instance for chaining
     */
    public TankConstants setUseFeedForward(boolean useFeedForward) {
        this.feedForward = useFeedForward;
        return this;
    }

    /**
     * Sets whether to use robot-centric or field-centric controls in TeleOp.
     * @param robotCentric true for robot-centric controls, false for field-centric controls
     * @return this instance for chaining
     */
    public TankConstants setRobotCentric(boolean robotCentric) {
        this.robotCentric = robotCentric;
        return this;
    }
}
