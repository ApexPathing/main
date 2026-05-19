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
     * Applies a deadzone to the input value. If the absolute value of the input is less than 0.05,
     * it returns 0. Otherwise, it returns the original value.
     * @param value the input value to apply the deadzone to
     * @return the value after applying the deadzone
     */
    protected static double deadzone(double value) {
        return Math.abs(value) < 0.05 ? 0.0 : value;
    }

    /**
     * @return whether the drivetrain should use robot-centric controls (true) or field-centric controls (false)
     */
    protected abstract boolean isRobotCentric();

    /**
     * Moves the robot using the provided drive, strafe, and turn vectors.
     * The values are normalized and applied to the motors according to the mecanum drive formulas.
     * @param drive the forward/backward movement vector (positive for forward, negative for backward)
     * @param strafe the left/right movement vector (positive for right, negative for left)
     * @param turn the rotation vector (positive for clockwise, negative for counterclockwise)
     */
    public abstract void moveWithVectors(double drive, double strafe, double turn);

    /**
     * Drives the robot with provided joystick inputs and the robot's current heading. This method
     * is meant for field-centric control. If you are using robot-centric control, the robotHeading
     * parameter will be ignored, you can use the other drive method that doesn't require the
     * robot's heading.
     * @param x the left/right joystick input (positive for right, negative for left)
     * @param y the forward/backward joystick input (positive for forward, negative for backward)
     * @param turn the rotation joystick input (positive for clockwise, negative for counterclockwise)
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
        moveWithVectors(adjY, adjX, adjTurn);
    }

    /**
     * Drives the robot with provided joystick inputs. This method is meant for robotic-centric
     * control. If you are using field-centric control, you have to use the other drive method that
     * requires the robot's current heading to be passed in as a parameter.
     * @param x the left/right joystick input (positive for right, negative for left)
     * @param y the forward/backward joystick input (positive for forward, negative for backward)
     * @param turn the rotation joystick input (positive for clockwise, negative for counterclockwise)
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
