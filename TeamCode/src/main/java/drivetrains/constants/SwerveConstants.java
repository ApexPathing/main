package drivetrains.constants;

import com.qualcomm.robotcore.hardware.HardwareMap;

import drivetrains.Swerve;
import util.Distance;

/**
 * Swerve drivetrain constants class
 *
 * @author Xander Haemel 31616 404 Not Found
 * @author Sohum Arora 22985 Paraducks
 * @author Dylan B. 18597 RoboClovers - Delta
 */
public class SwerveConstants extends DrivetrainConstants {
    // Swerve module constants
    public SwerveModuleConstants flModuleConstants = new SwerveModuleConstants();
    public SwerveModuleConstants blModuleConstants = new SwerveModuleConstants();
    public SwerveModuleConstants frModuleConstants = new SwerveModuleConstants();
    public SwerveModuleConstants brModuleConstants = new SwerveModuleConstants();

    // Distances and spacing
    public Distance wheelbase = Distance.fromMm(300); // Front pod to back pod spacing
    public Distance trackWidth = Distance.fromMm(300); // Left pod to right pod spacing
    public double diagonalDistance; // Inches
    public double wheelbaseRatio; // Wheelbase / diagonal distance
    public double trackWidthRatio; // Track width / diagonal distance

    // Misc
    public double maxPower = 1.0; // Max power to apply to any module's motor, from 0 to 1
    public boolean robotCentric = true; // Whether to use robot-centric controls (true) or field-centric controls (false) in TeleOp

    /** Constructor for the SwerveConstants class */
    public SwerveConstants() {}

    @Override
    public Swerve build(HardwareMap hardwareMap) {
        this.diagonalDistance = this.getDiagonalDistance();
        this.wheelbaseRatio = this.getWheelbaseRatio();
        this.trackWidthRatio = this.getTrackWidthRatio();

        return new Swerve(hardwareMap, this);
    }

    /** @return the diagonal distance between the wheelbase and trackwidth in inches. */
    private double getDiagonalDistance() { return this.wheelbase.hypotenuse(this.trackWidth).getIn(); }

    /** @return the ratio between the wheelbase and diagonal distance. */
    public double getWheelbaseRatio() { return this.wheelbase.getIn() / this.diagonalDistance; }

    /** @return the ratio between the trackwidth and diagonal distance. */
    public double getTrackWidthRatio() { return this.trackWidth.getIn() / this.diagonalDistance; }

    /**
     * Sets the front left module constants.
     * @param constants the {@link SwerveModuleConstants} to use for the front left module
     * @return this instance for chaining
     */
    public SwerveConstants setFrontLeftModuleConstants(SwerveModuleConstants constants) {
        this.flModuleConstants = constants;
        return this;
    }

    /**
     * Sets the back left module constants.
     * @param constants the {@link SwerveModuleConstants} to use for the back left module
     * @return this instance for chaining
     */
    public SwerveConstants setBackLeftModuleConstants(SwerveModuleConstants constants) {
        this.blModuleConstants = constants;
        return this;
    }

    /**
     * Sets the front right module constants.
     * @param constants the {@link SwerveModuleConstants} to use for the front right module
     * @return this instance for chaining
     */
    public SwerveConstants setFrontRightModuleConstants(SwerveModuleConstants constants) {
        this.frModuleConstants = constants;
        return this;
    }

    /**
     * Sets the back right module constants.
     * @param constants the {@link SwerveModuleConstants} to use for the back right module
     * @return this instance for chaining
     */
    public SwerveConstants setBackRightModuleConstants(SwerveModuleConstants constants) {
        this.brModuleConstants = constants;
        return this;
    }

    /**
     * Sets the wheelbase spacing and updates diagonal distance.
     * @param wheelbase the front pod to back pod spacing
     * @return this instance for chaining
     */
    public SwerveConstants setWheelbase(Distance wheelbase) {
        this.wheelbase = wheelbase;
        return this;
    }

    /**
     * Sets the trackwidth spacing and updates diagonal distance.
     * @param trackWidth the left pod to right pod spacing
     * @return this instance for chaining
     */
    public SwerveConstants setTrackWidth(Distance trackWidth) {
        this.trackWidth = trackWidth;
        return this;
    }

    /**
     * Sets the maximum power to apply to any module's drive motor.
     * @param maxPower the maximum power, from 0 to 1
     * @return this instance for chaining
     */
    public SwerveConstants setMaxPower(double maxPower) {
        this.maxPower = maxPower;
        return this;
    }

    /**
     * Sets whether to use robot-centric controls (true) or field-centric controls (false) in TeleOp.
     * @param robotCentric whether to use robot-centric controls (true) or field-centric controls (false) in TeleOp
     * @return this instance for chaining
     */
    public SwerveConstants setRobotCentric(boolean robotCentric) {
        this.robotCentric = robotCentric;
        return this;
    }

    /**
    * Sets the steering proportional gain.
    * @param kP the proportional gain for steering correction
    * @return this instance for chaining
    */
    public SwerveConstants setSteeringPGain(double kP) {
        this.flModuleConstants.steeringPGain = kP;
        this.blModuleConstants.steeringPGain = kP;
        this.frModuleConstants.steeringPGain = kP;
        this.brModuleConstants.steeringPGain = kP;
        return this;
    }
}