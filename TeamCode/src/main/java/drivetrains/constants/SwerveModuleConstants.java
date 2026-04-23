package drivetrains.constants;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import drivetrains.Swerve;
import drivetrains.SwerveModule;
import util.MotorMetaData;

/**
 * Swerve module constants class.
 * This is meant to be used for a {@link Swerve} drivetrain inside of {@link SwerveConstants}
 *
 * @author Dylan B. 18597 RoboClovers - Delta
 */
public class SwerveModuleConstants {
    // Hardware
    public MotorMetaData motorData = new MotorMetaData();
    public String servoName = "flServo";
    public String encoderName = "flEncoder"; // 0–3.3V maps to 0–360 degrees

    // Tuned values (these are set by the main SwerveConstants class because all modules should have the same tuning)
    public double steeringPGain = 0.015; // Proportional gain for steering correction

    /**
     * Constructor for the SwerveConstants class
     */
    public SwerveModuleConstants() {}

    /**
     * Builds a SwerveModule using this SwerveModuleConstants instance.
     * @param hardwareMap the hardware map to use for initializing the module
     * @return a new SwerveModule instance configured with this SwerveModuleConstants
     */
    public SwerveModule build(HardwareMap hardwareMap) {
        return new SwerveModule(hardwareMap, this);
    }

    /**
     * Sets the module's motor name. Default: "front_left_drive"
     * @param name the name of the modules' motor
     * @return this instance for chaining
     */
    public SwerveModuleConstants setMotorName(String name) {
        this.motorData.setName(name);
        return this;
    }

    /**
     * Default direction is FORWARD.
     * @param reversed whether the module's motor is reversed
     * @return this instance for chaining
     */
    public SwerveModuleConstants setMotorReversed(boolean reversed) {
        this.motorData.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        return this;
    }

    /**
     * Sets whether to use braking mode.
     * @param brakeMode true for brake mode, false for float mode
     * @return this instance for chaining
     */
    public SwerveModuleConstants setBrakeMode(boolean brakeMode) {
        this.motorData.setBrakeMode(brakeMode ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        return this;
    }

    /**
     * Sets the module's servo name.
     * @param name the name of the module's servo
     * @return this instance for chaining
     */
    public SwerveModuleConstants setServoName(String name) {
        this.servoName = name;
        return this;
    }

    /**
     * Sets the module's encoder name.
     * @param name the name of the module's encoder
     * @return this instance for chaining
     */
    public SwerveModuleConstants setEncoderName(String name) {
        this.encoderName = name;
        return this;
    }
}