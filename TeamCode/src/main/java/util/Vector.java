package util;

import androidx.annotation.NonNull;

public class Vector {
    Distance x;
    Distance y;
    Distance.Units unit;

    // region Constructors abd factory methods
    /**
     * Constructor for the {@link Vector} class
     * @param x the x component of the vector
     * @param y the y component of the vector
     * @param unit the input and output unit
     */
    public Vector(double x, double y, Distance.Units unit) {
        this.x = Distance.from(unit, x); this.y = Distance.from(unit, y);
        this.unit = unit;
    }

    /**
     * Constructor for the {@link Vector} class with the default unit of inches
     * @param x the x component of the vector
     * @param y the y component of the vector
     */
    public Vector(double x, double y) { this(x, y, Distance.Units.INCHES); }

    /**
     * Constructor for {@link Vector} class with {@link Distance} objects
     * @param x the x {@link Distance} of the vector
     * @param y the y {@link Distance} of the vector
     */
    public Vector(Distance x, Distance y) { this.x = x; this.y = y; }

    /**
     * Default constructor for {@link Vector} class, initializes to (0, 0) in inches
     */
    public Vector() { this(new Distance(), new Distance()); }

    /** Factory method to create a {@link Vector} from a {@link Pose} */
    public static Vector fromPose(Pose pose) { return new Vector(pose.getX(), pose.getY()); }
    // endregion

    // TODO: Getter and setter Javadocs
    // region Getters
    public double getX() { return this.x.get(this.unit); }
    public double getY() { return this.y.get(this.unit); }

    public Distance getXComponent() { return this.x; }
    public Distance getYComponent() { return this.y; }

    public Distance.Units getUnit() { return this.unit; }
    // endregion

    // region Setters
    public void setX(double x) { this.x = Distance.from(this.unit, x); }
    public void setY(double y) { this.y = Distance.from(this.unit, y); }

    public void setUnit(Distance.Units unit) { this.unit = unit; }
    // endregion

    // region Arithmetic methods
    // TODO
    // endregion

    // region Geometric methods
    /**
     * Rotates the vector by a given angle in radians.
     * @param angle The angle to rotate the vector by, in radians.
     */
    public void rotate(double angle) {
        double cosA = Math.cos(angle);
        double sinA = Math.sin(angle);
        double x = this.x.get(Distance.Units.INCHES);
        double y = this.y.get(Distance.Units.INCHES);

        this.x = new Distance((x * cosA) - (y * sinA));
        this.y = new Distance((x * sinA) + (y * cosA));
    }

    // TODO
    // endregion

    // TODO: add utility functions and Javadocs for them
    // region Utility methods
    /** Return a copy of this {@link Vector} */
    public Vector copy() { return new Vector(this.x, this.y); }

    @Override
    @NonNull
    public String toString() { return String.format("Vector(x: %s in, y: %s in)", this.x, this.y); }
    // endregion
}