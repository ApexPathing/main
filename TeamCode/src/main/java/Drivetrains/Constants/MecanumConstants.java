package Drivetrains.Constants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Mecanum Constants class
 * @author Xander Haemel - 31616 404 Not Found
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class MecanumConstants {
    // Motor names
    public String leftFrontMotorName = "font_left_drive";
    public String leftRearMotorName = "back_left_drive";
    public String rightFrontMotorName = "front_right_drive";
    public String rightRearMotorName = "back_right_drive";

    // Motor directions
    public DcMotorSimple.Direction leftFrontDirection = DcMotorSimple.Direction.REVERSE;
    public DcMotorSimple.Direction leftRearDirection = DcMotorSimple.Direction.REVERSE;
    public DcMotorSimple.Direction rightFrontDirection = DcMotorSimple.Direction.FORWARD;
    public DcMotorSimple.Direction rightRearDirection = DcMotorSimple.Direction.FORWARD;

    // Tuned values TODO: USE THESE
    public double xVelocity = 60; // Inches per second
    public double yVelocity = 60; // Inches per second

    // Miscellaneous constants
    public double maxPower = 1.0; // 0 to 1, max power to apply to the motors
    public boolean useBrakingMode = false; // Brake mode = true, Float mode = false
    public boolean useFeedForward = true; // Whether to use feedforward in the velocity controller TODO: USE THIS
    public boolean robotCentric = true; // Whether to use robot-centric controls (true) or field-centric controls (false) in TeleOp

    /**
     * Constructor for the MecanumConstants class
     * Default constants are derived from the BasicOmniOpMode_Linear SDK sample with untuned values
     */
    public MecanumConstants() {}

    /**
     * Sets the left front motor name.
     * @param leftFrontMotorName the name of the left front motor
     * @return this instance for chaining
     */
    public MecanumConstants setLeftFrontMotorName(String leftFrontMotorName) {
        this.leftFrontMotorName = leftFrontMotorName;
        return this;
    }

    /**
     * Sets the left rear motor name.
     * @param leftRearMotorName the name of the left rear motor
     * @return this instance for chaining
     */
    public MecanumConstants setLeftRearMotorName(String leftRearMotorName) {
        this.leftRearMotorName = leftRearMotorName;
        return this;
    }

    /**
     * Sets the right front motor name.
     * @param rightFrontMotorName the name of the right front motor
     * @return this instance for chaining
     */
    public MecanumConstants setRightFrontMotorName(String rightFrontMotorName) {
        this.rightFrontMotorName = rightFrontMotorName;
        return this;
    }

    /**
     * Sets the right rear motor name.
     * @param rightRearMotorName the name of the right rear motor
     * @return this instance for chaining
     */
    public MecanumConstants setRightRearMotorName(String rightRearMotorName) {
        this.rightRearMotorName = rightRearMotorName;
        return this;
    }

    /**
     * Sets the left front motor direction.
     * @param leftFrontDirection the direction of the left front motor
     * @return this instance for chaining
     */
    public MecanumConstants setLeftFrontDirection(DcMotorSimple.Direction leftFrontDirection) {
        this.leftFrontDirection = leftFrontDirection;
        return this;
    }

    /**
     * Sets the left rear motor direction.
     * @param leftRearDirection the direction of the left rear motor
     * @return this instance for chaining
     */
    public MecanumConstants setLeftRearDirection(DcMotorSimple.Direction leftRearDirection) {
        this.leftRearDirection = leftRearDirection;
        return this;
    }

    /**
     * Sets the right front motor direction.
     * @param rightFrontDirection the direction of the right front motor
     * @return this instance for chaining
     */
    public MecanumConstants setRightFrontDirection(DcMotorSimple.Direction rightFrontDirection) {
        this.rightFrontDirection = rightFrontDirection;
        return this;
    }

    /**
     * Sets the right rear motor direction.
     * @param rightRearDirection the direction of the right rear motor
     * @return this instance for chaining
     */
    public MecanumConstants setRightRearDirection(DcMotorSimple.Direction rightRearDirection) {
        this.rightRearDirection = rightRearDirection;
        return this;
    }

    /**
     * Sets the X velocity value from tuning.
     * @param xVelocity the X velocity in inches per second
     * @return this instance for chaining
     */
    public MecanumConstants setXVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
        return this;
    }

    /**
     * Sets the Y velocity value from tuning.
     * @param yVelocity the Y velocity in inches per second
     * @return this instance for chaining
     */
    public MecanumConstants setYVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
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
     * Sets whether to use braking mode.
     * @param useBrakingMode true for brake mode, false for float mode
     * @return this instance for chaining
     */
    public MecanumConstants setUseBrakingMode(boolean useBrakingMode) {
        this.useBrakingMode = useBrakingMode;
        return this;
    }

    /**
     * Sets whether to use feedforward in the velocity controller.
     * @param useFeedForward true to use feedforward, false otherwise
     * @return this instance for chaining
     */
    public MecanumConstants setUseFeedForward(boolean useFeedForward) {
        this.useFeedForward = useFeedForward;
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
