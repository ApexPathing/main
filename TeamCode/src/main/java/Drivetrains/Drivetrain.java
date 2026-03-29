package Drivetrains;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Abstract class implemented by all drivetrain classes
 * @author ohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public abstract class Drivetrain {
    /**
     * Sets the zero power behavior for all drivetrain motors
     * @param behavior the zero power behavior to set for all drivetrain motors
     */
    protected abstract void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior);

    /**
     * Applies a deadzone to the input value. If the absolute value of the input is less than 0.05,
     * it returns 0. Otherwise, it returns the original value.
     * @param value the input value to apply the deadzone to
     * @return the value after applying the deadzone
     */
    protected static double deadzone(double value) {
        return Math.abs(value) < 0.05 ? 0.0 : value;
    }

    /**
     * Update the brake mode for all drivetrain motors
     * @param brake true for brake mode, false for float mode
     */
    public void setBrakeMode(boolean brake) {
        if (brake) {
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    /**
     * Moves the robot using the provided drive, strafe, and turn vectors.
     * The values are normalized and applied to the motors according to the mecanum drive formulas.
     * @param drive the forward/backward movement vector (positive for forward, negative for backward)
     * @param strafe the left/right movement vector (positive for right, negative for left)
     * @param turn the rotation vector (positive for clockwise, negative for counterclockwise)
     */
    public abstract void moveWithVectors(double drive, double strafe, double turn);

    /**
     * Drives the robot using the provided joystick inputs and robot heading. The joystick inputs are adjusted
     * for field-centric or robot-centric control based on the constants, and a deadzone is applied to prevent drift.
     * @param x the left/right joystick input (positive for right, negative for left)
     * @param y the forward/backward joystick input (positive for forward, negative for backward)
     * @param turn the rotation joystick input (positive for clockwise, negative for counterclockwise)
     * @param robotHeading the current heading of the robot in radians, not used for robot centric control
     */
    public abstract void drive(double x, double y, double turn, double robotHeading);

    /**
     * Stop all drivetrain actuators
     */
    public abstract void stop();

    /**
     * Uses the telemetry object to display relevant drivetrain information such as motor powers and velocities.
     */
    public abstract void debug(Telemetry telemetry);
}
