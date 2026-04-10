package util;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

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
     */
    public Pose(double x, double y, double heading, Distance.Units posUnit, Angle.Units angleUnit, boolean mirror) {
        this.position = new Vector(x, y, posUnit);
        this.heading = Angle.from(angleUnit, heading);
        this.angleUnit = angleUnit;
        if (mirror) { this.mirror(); }
    }

    /**
     * A default constructor to create and {@link Pose} object.
     * @param x distance (inches)
     * @param y distance (inches)
     * @param heading angle (radians)
     * @return
     */
    public static Pose defaultPose(double x, double y, double heading) {
        return new Pose(x, y, heading, Distance.Units.INCHES, Angle.Units.RADIANS, false);
    }

    /** Factory method to create a {@link Pose} from an FTC SDK {@link Pose2D} object with specified units and mirroring */
    public static Pose fromPose2D(Pose2D pose2D, Distance.Units posUnit, Angle.Units angleUnit, boolean mirror) {
        return new Pose(
                pose2D.getX(DistanceUnit.INCH),
                pose2D.getY(DistanceUnit.INCH),
                pose2D.getHeading(AngleUnit.RADIANS),
                Distance.Units.INCHES, Angle.Units.RADIANS, mirror
        );
    }
    // endregion

    // TODO: Javadocs
    // region Getters
    public double getX() { return this.position.getX(); }
    public double getY() { return this.position.getY(); }
    public double getHeading() { return this.heading.get(this.angleUnit); }

    public Vector getPositionComponent() { return this.position; }
    public Distance getXComponent() { return this.position.x; }
    public Distance getYComponent() { return this.position.y; }
    public Angle getHeadingComponent() { return this.heading; }

    public Distance.Units getDistanceUnit() { return this.position.getUnit(); }
    public Angle.Units getAngleUnit() { return this.angleUnit; }
    // endregion

    // region Setters
    public void setX(double x) { this.position.setX(x); }
    public void setY(double y) { this.position.setY(y); }
    public void setHeading(double heading) { this.heading = Angle.from(this.angleUnit, heading); }

    public void setDistanceUnit(Distance.Units unit) { this.position.setUnit(unit); }
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
    /** @return true if this pose is equal to another pose, false otherwise */
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

    // TODO: Add utility methods and Javadocs for them
    // region Utility methods
    /** Mirror the pose in place across the y-axis */
    public void mirror() {
        this.position.x.mirror();
        this.heading.mirror();
    }

    public Pose copy() {
        return new Pose(
                this.getX(), this.getY(), this.getHeading(),
                this.getDistanceUnit(), this.angleUnit, false
        );
    }

    /**
     * Convert this {@link Pose} to an FTC SDK {@link Pose2D} object
     * @return a new {@link Pose2D} object representing this pose
     */
    public Pose2D toPose2D() {
        return new Pose2D(
                DistanceUnit.INCH,
                this.getXComponent().getIn(),
                this.getYComponent().getIn(),
                AngleUnit.RADIANS,
                this.getHeadingComponent().getDeg()
        );
    }

    @Override
    @NonNull
    public String toString() {
        return String.format(
                "Pose(x=%s %s, y=%s %s, heading=%s %s)",
                getX(), position.getUnit(), getY(), position.getUnit(), getHeading(), angleUnit
        );
    }
    // endregion
}
