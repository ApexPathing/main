package drivetrains.constants;

import com.qualcomm.robotcore.hardware.HardwareMap;

import drivetrains.Drivetrain;

/**
 * Abstract base class for drivetrain constants.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public abstract class DrivetrainConstants {
    /**
    * Constructor for the DrivetrainConstants class
    */
    public DrivetrainConstants() {}

    /**
     * Builds a Drivetrain instance based on the constants defined in this class.
     * This method must be implemented by subclasses to return the appropriate type of Drivetrain.
     * @param hardwareMap the HardwareMap to use for initializing the Drivetrain's motors and sensors
     * @return a Drivetrain instance configured with the constants defined in this class
     */
    abstract public Drivetrain build(HardwareMap hardwareMap);
}