package drivetrains;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Abstract class implemented by all drivetrain classes
 *
 * @author ohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public abstract class Drivetrain {
    /**
     * Applies a deadzone to the input value. If the absolute value of the input is less than 0.03,
     * it returns 0. Otherwise, it returns the original value.
     * @param value the input value to apply the deadzone to
     * @return the value after applying the deadzone
     */
    protected static double deadzone(double value) {
        return Math.abs(value) < 0.03 ? 0.0 : value;
    }

    /**
     * @return whether the drivetrain should use robot-centric controls (true) or field-centric controls (false)
     */
    protected abstract boolean isRobotCentric();

    /**
     * Moves the robot using the provided drive, strafe, and turn vectors.
     * The values are normalized and applied to the motors according to the mecanum drive formulas.
     * @param x the forward/backward movement vector (positive for forward, negative for backward)
     * @param y the left/right movement vector (positive for left, negative for right)
     * @param turn the rotation vector (positive for counterclockwise, negative for clockwise)
     */
    public abstract void moveWithVectors(double x, double y, double turn);

    /**
     * Drives the robot with provided joystick inputs and the robot's current heading. This method
     * is meant for field-centric control. If you are using robot-centric control, the robotHeading
     * parameter will be ignored, you can use the other drive method that doesn't require the
     * robot's heading.
     * @param x the forward/backward joystick input (positive for forward, negative for backward)
     * @param y the left/right joystick input (positive for left, negative for right)
     * @param turn the rotation joystick input (positive for counterclockwise, negative for clockwise)
     * @param robotHeading the current heading of the robot in radians, not used for robot centric control
     */
    public void drive(double x, double y, double turn, double robotHeading) {
        double adjX, adjY, adjTurn;
        if (isRobotCentric()) {
            adjX = deadzone(x);
            adjY = deadzone(y);
        } else {
            double cos = Math.cos(-robotHeading);
            double sin = Math.sin(-robotHeading);
            adjX = deadzone(x * cos - y * sin);
            adjY = deadzone(x * sin + y * cos);
        }
        adjTurn = deadzone(turn);
        moveWithVectors(adjX, adjY, adjTurn);
    }

    /**
     * Drives the robot with provided joystick inputs. This method is meant for robotic-centric
     * control. If you are using field-centric control, you have to use the other drive method that
     * requires the robot's current heading to be passed in as a parameter.
     * @param x the forward/backward joystick input (positive for forward, negative for backward)
     * @param y the left/right joystick input (positive for left, negative for right)
     * @param turn the rotation joystick input (positive for counterclockwise, negative for clockwise)
     */
    public void drive(double x, double y, double turn) { drive(x, y, turn, 0); }

    /**
     * Stop all drivetrain actuators
     */
    public abstract void stop();

    public abstract void debug(Telemetry telemetry);

    @NonNull
    @Override
    public String toString() {
        return "The drivetrain type didn't implement toString()!";
    }
}
