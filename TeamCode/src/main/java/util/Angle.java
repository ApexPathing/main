package util;

import androidx.annotation.NonNull;

/**
 * Class to represent an angle consisting of formats for both radians and degrees.
 * Provides methods for conversion between the two units. The angle is automatically normalized
 * to the range [0, 2π) for radians and [0, 360) for degrees.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class Angle {
    /** Enum to represent the different angle units supported by the {@link Angle} class */
    public enum Units {
        RADIANS,
        DEGREES
    }

    private double baseAngle; // Normalized radians
    
    // region Constructors and factory methods for each angle unit
    /**
     * Constructor for the {@link Angle} class
     * @param radians the angle in radians
     */
    public Angle(double radians) { this.baseAngle = normalize(radians); }

    /** Default constructor for the {@link Angle} class, initializes angle to 0 radians */
    public Angle() { this(0.0); }

    /** Factory method to create an {@link Angle} from radians */
    public static Angle fromRad(double radians) { return new Angle(radians); }

    /** Factory method to create an {@link Angle} from degrees */
    public static Angle fromDeg(double degrees) { return new Angle(Math.toRadians(degrees)); }

    /** Factory method to create an {@link Angle} from a specified {@link Units} and value */
    public static Angle from(Units unit, double value) {
        switch (unit) {
            case RADIANS: return fromRad(value);
            case DEGREES: return fromDeg(value);
            default: throw new IllegalArgumentException("Unknown unit: " + unit);
        }
    }
    // endregion

    // region Getters for the angle in different units and normalization states
    /** @return the angle in radians without normalization */
    public double getRad() { return this.baseAngle; }

    /** @return the angle in degrees without normalization */
    public double getDeg() { return Math.toDegrees(this.baseAngle); }

    /** @return the angle in the specified {@link Units} */
    public double get(Units unit) {
        switch (unit) {
            case RADIANS: return getRad();
            case DEGREES: return getDeg();
            default: throw new IllegalArgumentException("Unknown unit: " + unit);
        }
    }
    // endregion

    // region Arithmetic operations
    /** @return a new {@link Angle} that is the sum of this angle and another angle */
    public Angle plus(Angle other) { return new Angle(this.baseAngle + other.baseAngle); }

    /** @return a new {@link Angle} that is the difference between this angle and another angle */
    public Angle subtract(Angle other) { return new Angle(this.baseAngle - other.baseAngle); }

    /** @return a new {@link Angle} that is this angle multiplied by a scalar */
    public Angle multiply(double scalar) { return new Angle(this.baseAngle * scalar); }

    /** @return a new {@link Angle} that is the product of this angle and another angle */
    public Angle multiply(Angle other) { return new Angle(this.baseAngle * other.baseAngle); }

    /** @return a new {@link Angle} that is this angle divided by a scalar */
    public Angle divide(double scalar) { return new Angle(this.baseAngle / scalar); }

    /** @return a new {@link Angle} that is the quotient of this angle and another angle */
    public Angle divide(Angle other) { return new Angle(this.baseAngle / other.baseAngle); }

    /** Mirror the angle across the y-axis in place */
    public void mirror() { this.baseAngle = Math.PI - this.baseAngle; }
    // endregion

    // region Utility methods
    /** @return a copy of this {@link Angle} */
    public Angle copy() { return new Angle(this.baseAngle); }

    /**
     * Normalizes the angle to the range [0, 2π)
     * @param radians the angle in radians to be normalized
     * @return the normalized angle in radians
     */
    public static double normalize(double radians) {
        return (radians % (2 * Math.PI) + (2 * Math.PI)) % (2 * Math.PI);
    }

    @Override
    @NonNull
    public String toString() { return String.format("Angle(%s radians)", this.baseAngle); }
    // endregion
}