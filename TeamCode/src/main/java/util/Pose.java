package util;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

/**
 * Class representing a 2D pose with vector (x, y) and heading component. Provides methods for pose
 * arithmetic, comparison, and utility functions such as mirroring and conversion to FTC SDK Pose2D.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class Pose {
    private final Vector position;
    private Angle heading;
    public Angle.Units angleUnit;

    // region Constructors and factory methods

    /**
     * Constructor for {@link Pose} class with manual x, y, and heading values in specified units
     * @param x the x component of the position in the specified unit
     * @param y the y component of the position in the specified unit
     * @param heading the heading of the pose as an angle value in the specified unit
     * @param posUnit the {@link Distance.Units} of the x and y components of the position
     * @param angleUnit the {@link Angle.Units} of the heading angle
     * @param mirror whether the pose should be mirrored across the y axis or not
     */
    public Pose(double x, double y, double heading, Distance.Units posUnit, Angle.Units angleUnit, boolean mirror) {
        this.position = new Vector(x, y, posUnit);
        this.heading = Angle.from(angleUnit, heading);
        this.angleUnit = angleUnit;
        if (mirror) { this.mirror(); }
    }

    /**
     * Constructor for {@link Pose} class with manual x, y, and heading values in specified units without mirroring
     * @param x the x component of the position in the specified unit
     * @param y the y component of the position in the specified unit
     * @param heading the heading of the pose as an angle value in the specified unit
     * @param posUnit the {@link Distance.Units} of the x and y components of the position
     * @param angleUnit the {@link Angle.Units} of the heading angle
     */
    public Pose(double x, double y, double heading, Distance.Units posUnit, Angle.Units angleUnit) {
        this(x, y, heading, posUnit, angleUnit, false);
    }

    /**
     * Constructor for {@link Pose} class with manual x, y, and heading values in default units
     * @param x the x component of the position in inches
     * @param y the y component of the position in inches
     * @param heading the heading of the pose as an angle value in radians
     */
    public Pose(double x, double y, double heading) {
        this(x, y, heading, Distance.Units.INCHES, Angle.Units.RADIANS, false);
    }

    /**
     * Factory method to create a {@link Pose} from an FTC SDK {@link Pose2D} object with specified units and mirroring
     * @param pose2D the FTC SDK Pose2D object
     * @param posUnit the desired distance unit for the new pose
     * @param angleUnit the desired angle unit for the new pose
     * @param mirror whether the pose should be mirrored across the y axis
     * @return a new Pose object matching the FTC SDK Pose2D
     */
    public static Pose fromPose2D(Pose2D pose2D, Distance.Units posUnit, Angle.Units angleUnit, boolean mirror) {
        return new Pose(
                pose2D.getX(DistanceUnit.INCH),
                pose2D.getY(DistanceUnit.INCH),
                pose2D.getHeading(AngleUnit.RADIANS),
                Distance.Units.INCHES, Angle.Units.RADIANS, mirror
        );
    }

    /**
     * Constructor for a {@link Pose} with an undefined/infinite heading in default units.
     * Useful for representing raw coordinates where rotation does not matter.
     * @param x the x component in inches
     * @param y the y component in inches
     */
    public Pose(double x, double y) {
        this (x, y, Double.POSITIVE_INFINITY);
    }

    /**
     * Constructor for a {@link Pose} with an undefined/infinite heading in specified units.
     * @param unit the {@link Distance.Units} of the x and y components
     * @param x the x component
     * @param y the y component
     */
    public Pose(Distance.Units unit, double x, double y) {
        // BUG FIX: Replaced 'null' angleUnit with Angle.Units.RADIANS to prevent a NullPointerException
        // when Angle.from() attempts to switch on the provided enum.
        this(x, y, Double.POSITIVE_INFINITY, unit, Angle.Units.RADIANS);
    }

    /**
     * Factory method to create a {@link Pose} at (0, 0, 0) in default units
     * (inches and radians) with no mirroring
     * @return a new {@link Pose} object representing the origin pose with 0 heading
     */
    public static Pose zero() {
        return new Pose(0, 0, 0);
    }

    // endregion

    // region Getters

    /** @return the x coordinate in the current distance unit */
    public double getX() { return this.position.getX(); }

    /** @return the y coordinate in the current distance unit */
    public double getY() { return this.position.getY(); }

    /** @return the full 2D position as a {@link Vector} */
    public Vector toVec() { return this.position; }

    /** @return the heading in the current angle unit */
    public double getHeading() { return this.heading.get(this.angleUnit); }

    /** @return the position component as a {@link Vector} */
    public Vector getPositionComponent() { return this.position; }

    /** @return the x component as a strongly-typed {@link Distance} object */
    public Distance getXComponent() { return this.position.getXComponent(); }

    /** @return the y component as a strongly-typed {@link Distance} object */
    public Distance getYComponent() { return this.position.getYComponent(); }

    /** @return the heading component as a strongly-typed {@link Angle} object */
    public Angle getHeadingComponent() { return this.heading; }

    /** @return the current unit of measurement for distances */
    public Distance.Units getDistanceUnit() { return this.position.getUnit(); }

    /** @return the current unit of measurement for angles */
    public Angle.Units getAngleUnit() { return this.angleUnit; }

    // endregion

    // region Setters

    /** @param x the new x coordinate in the current distance unit */
    public void setX(double x) { this.position.setX(x); }

    /** @param y the new y coordinate in the current distance unit */
    public void setY(double y) { this.position.setY(y); }

    /** @param heading the new heading in the current angle unit */
    public void setHeading(double heading) { this.heading = Angle.from(this.angleUnit, heading); }

    /** @param unit the new distance unit to use for this pose */
    public void setDistanceUnit(Distance.Units unit) { this.position.setUnit(unit); }

    /** @param unit the new angle unit to use for this pose */
    public void setAngleUnit(Angle.Units unit) { this.angleUnit = unit; }

    // endregion

    // region Arithmetic methods

    /**
     * Add 2 {@link Pose}s together, this pose is treated as the base pose
     * @param other the other pose to add to this pose
     * @return a new pose representing the sum of this pose and the other pose
     */
    public Pose add(Pose other) {
        return new Pose(
                this.getX() + other.getXComponent().get(this.getDistanceUnit()),
                this.getY() + other.getYComponent().get(this.getDistanceUnit()),
                this.getHeading() + other.getHeadingComponent().get(this.angleUnit),
                this.getDistanceUnit(), this.angleUnit, false
        );
    }

    /**
     * Subtract another {@link Pose} from this pose, this pose is treated as the base pose
     * @param other the other pose to subtract from this pose
     * @return a new pose representing the difference between this pose and the other pose
     */
    public Pose subtract(Pose other) {
        return new Pose(
                this.getX() - other.getXComponent().get(this.getDistanceUnit()),
                this.getY() - other.getYComponent().get(this.getDistanceUnit()),
                this.getHeading() - other.getHeadingComponent().get(this.angleUnit),
                this.getDistanceUnit(), this.angleUnit, false
        );
    }

    /**
     * Multiply this pose (in the original units) by a scalar value
     * @param scalar the scalar value to multiply this pose by
     * @return a new scaled pose
     */
    public Pose multiply(double scalar) {
        return new Pose(
                this.getX() * scalar,
                this.getY() * scalar,
                this.getHeading() * scalar,
                this.getDistanceUnit(), this.angleUnit, false
        );
    }

    /**
     * Multiply this pose (in the original units) by another pose, this pose is treated as the base
     * pose and the other pose is treated as a relative pose to be multiplied by this pose
     * @param other the other pose to multiply this pose by
     * @return a new pose representing the product of this pose and the other pose
     */
    public Pose multiply(Pose other) {
        return new Pose(
                this.getX() * other.getXComponent().get(this.getDistanceUnit()),
                this.getY() * other.getYComponent().get(this.getDistanceUnit()),
                this.getHeading() * other.getHeadingComponent().get(this.angleUnit),
                this.getDistanceUnit(), this.angleUnit, false
        );
    }

    /**
     * Divide this pose (in the original units) by a scalar value
     * @param scalar the scalar value to divide this pose by
     * @return a new scaled pose
     */
    public Pose divide(double scalar) {
        return new Pose(
                this.getX() / scalar,
                this.getY() / scalar,
                this.getHeading() / scalar,
                this.getDistanceUnit(), this.angleUnit, false
        );
    }

    /**
     * Divide this pose (in the original units) by another pose, this pose is treated as the base
     * pose and the other pose is treated as a relative pose to be divided by this pose
     * @param other the other pose to divide this pose by
     * @return a new pose representing the quotient of this pose and the other pose
     */
    public Pose divide(Pose other) {
        return new Pose(
                this.getX() / other.getXComponent().get(this.getDistanceUnit()),
                this.getY() / other.getYComponent().get(this.getDistanceUnit()),
                this.getHeading() / other.getHeadingComponent().get(this.angleUnit),
                this.getDistanceUnit(), this.angleUnit, false
        );
    }

    // endregion

    // region Comparison methods

    /**
     * @param other the pose to compare against
     * @return true if this pose is exactly equal to another pose, false otherwise
     */
    public boolean equals(Pose other) {
        return this.getX() == other.getXComponent().get(this.getDistanceUnit()) &&
                this.getY() == other.getYComponent().get(this.getDistanceUnit()) &&
                this.getHeading() == other.getHeadingComponent().get(this.angleUnit);
    }

    /**
     * Check if this pose is within a certain distance and angle tolerance of another pose, this
     * pose is treated as the base pose
     * @param other the pose to compare this pose to
     * @param distTolerance the distance tolerance in the units of this pose's position components
     * @param angleTolerance the angle tolerance in the units of this pose's heading component
     * @return true if this pose is within the specified distance and angle tolerance of the other pose, false otherwise
     */
    public boolean isNear(Pose other, double distTolerance, double angleTolerance) {
        return Math.abs(this.getX() - other.getXComponent().get(this.getDistanceUnit())) < distTolerance &&
                Math.abs(this.getY() - other.getYComponent().get(this.getDistanceUnit())) < distTolerance &&
                Math.abs(this.getHeading() - other.getHeadingComponent().get(this.angleUnit)) < angleTolerance;
    }

    /**
     * Get the distance between this pose and another pose, this pose is treated as the base pose
     * @param other the other pose to get the distance to
     * @return the distance between this pose and the other pose in this pose's distance unit
     */
    public double distanceTo(Pose other) {
        return Math.hypot(
                this.getX() - other.getXComponent().get(this.getDistanceUnit()),
                this.getY() - other.getYComponent().get(this.getDistanceUnit())
        );
    }

    // endregion

    // region Utility methods

    /**
     * Mirror the pose in place across the y-axis.
     * Alters both the position's x-coordinate and the heading angle.
     */
    public void mirror() {
        this.position.getXComponent().mirror();
        this.heading.mirror();
    }

    /**
     * Create a deep copy of this pose.
     * @return a new {@link Pose} object with identical properties
     */
    public Pose copy() {
        return new Pose(
                this.getX(), this.getY(), this.getHeading(),
                this.getDistanceUnit(), this.angleUnit, false
        );
    }

    /**
     * Convert this {@link Pose} to an FTC SDK {@link Pose2D} object
     * @return a new {@link Pose2D} object representing this pose in inches and radians
     */
    public Pose2D toPose2D() {
        return new Pose2D(
                DistanceUnit.INCH,
                this.getXComponent().getIn(),
                this.getYComponent().getIn(),
                AngleUnit.RADIANS,
                this.getHeadingComponent().getRad()
        );
    }

    @Override
    @NonNull
    public String toString() {
        return String.format(Locale.ENGLISH,
                "X=%.1f, Y=%.1f %s, Heading=%.1f %s",
                getX(), getY(), position.getUnit(), getHeading(), angleUnit
        );
    }



    // endregion
}