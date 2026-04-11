package util;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


/**
 * @author Sohum Arora 22985 Paraducks
 */
public class Pose {
    private final Vector position;
    private Angle heading;
    public Angle.Units angleUnit;

    public Pose(double x, double y, double heading, Distance.Units posUnit, Angle.Units angleUnit, boolean mirror) {
        this.position = new Vector(x, y, posUnit);
        this.heading = Angle.from(angleUnit, heading);
        this.angleUnit = angleUnit;
        if (mirror) { this.mirror(); }
    }

    public Pose(double x, double y, double heading) {
        this.position = new Vector(x, y, Distance.Units.INCHES);
        this.heading = Angle.from(Angle.Units.RADIANS, heading);
        this.angleUnit = Angle.Units.RADIANS;
    }

    public static Pose fromPose2D(Pose2D pose2D, Distance.Units posUnit, Angle.Units angleUnit, boolean mirror) {
        return new Pose(
                pose2D.getX(DistanceUnit.INCH),
                pose2D.getY(DistanceUnit.INCH),
                pose2D.getHeading(AngleUnit.RADIANS),
                Distance.Units.INCHES, Angle.Units.RADIANS, mirror
        );
    }

    public double getX() { return this.position.getX(); }
    public double getY() { return this.position.getY(); }
    public double getHeading() { return this.heading.get(this.angleUnit); }

    public Vector getPositionComponent() { return this.position; }
    public Distance getXComponent() { return this.position.x; }
    public Distance getYComponent() { return this.position.y; }
    public Angle getHeadingComponent() { return this.heading; }

    public Distance.Units getDistanceUnit() { return this.position.getUnit(); }
    public Angle.Units getAngleUnit() { return this.angleUnit; }

    public void setX(double x) { this.position.setX(x); }
    public void setY(double y) { this.position.setY(y); }
    public void setHeading(double heading) { this.heading = Angle.from(this.angleUnit, heading); }

    public void setDistanceUnit(Distance.Units unit) { this.position.setUnit(unit); }
    public void setAngleUnit(Angle.Units unit) { this.angleUnit = unit; }

    public Pose add(Pose other) {
        return new Pose(
                this.getX() + other.getXComponent().get(this.getDistanceUnit()),
                this.getY() + other.getYComponent().get(this.getDistanceUnit()),
                this.getHeading() + other.getHeadingComponent().get(this.angleUnit),
                this.getDistanceUnit(), this.angleUnit, false
        );
    }

    public Pose subtract(Pose other) {
        return new Pose(
                this.getX() - other.getXComponent().get(this.getDistanceUnit()),
                this.getY() - other.getYComponent().get(this.getDistanceUnit()),
                this.getHeading() - other.getHeadingComponent().get(this.angleUnit),
                this.getDistanceUnit(), this.angleUnit, false
        );
    }

    public Pose multiply(double scalar) {
        return new Pose(
                this.getX() * scalar,
                this.getY() * scalar,
                this.getHeading() * scalar,
                this.getDistanceUnit(), this.angleUnit, false
        );
    }

    public Pose multiply(Pose other) {
        return new Pose(
                this.getX() * other.getXComponent().get(this.getDistanceUnit()),
                this.getY() * other.getYComponent().get(this.getDistanceUnit()),
                this.getHeading() * other.getHeadingComponent().get(this.angleUnit),
                this.getDistanceUnit(), this.angleUnit, false
        );
    }

    public Pose divide(double scalar) {
        return new Pose(
                this.getX() / scalar,
                this.getY() / scalar,
                this.getHeading() / scalar,
                this.getDistanceUnit(), this.angleUnit, false
        );
    }

    public Pose divide(Pose other) {
        return new Pose(
                this.getX() / other.getXComponent().get(this.getDistanceUnit()),
                this.getY() / other.getYComponent().get(this.getDistanceUnit()),
                this.getHeading() / other.getHeadingComponent().get(this.angleUnit),
                this.getDistanceUnit(), this.angleUnit, false
        );
    }

    public boolean equals(Pose other) {
        return this.getX() == other.getXComponent().get(this.getDistanceUnit()) &&
                this.getY() == other.getYComponent().get(this.getDistanceUnit()) &&
                this.getHeading() == other.getHeadingComponent().get(this.angleUnit);
    }

    public boolean isNear(Pose other, double distTolerance, double angleTolerance) {
        return Math.abs(this.getX() - other.getXComponent().get(this.getDistanceUnit())) < distTolerance &&
                Math.abs(this.getY() - other.getYComponent().get(this.getDistanceUnit())) < distTolerance &&
                Math.abs(this.getHeading() - other.getHeadingComponent().get(this.angleUnit)) < angleTolerance;
    }

    public double distanceTo(Pose other) {
        return Math.hypot(
                this.getX() - other.getXComponent().get(this.getDistanceUnit()),
                this.getY() - other.getYComponent().get(this.getDistanceUnit())
        );
    }

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
        return String.format(
                "Pose(x=%s %s, y=%s %s, heading=%s %s)",
                getX(), position.getUnit(), getY(), position.getUnit(), getHeading(), angleUnit
        );
    }
    public static double normalize(double angle) {
        angle = angle % (2 * Math.PI);
        if (angle > Math.PI) angle -= 2 * Math.PI;
        if (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}