package util;

import androidx.annotation.NonNull;

/**
 * Class to represent a distance consisting of formats for many common units.
 * Provides methods for conversion and operations between units.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 * @author Achintya - 30099 OmicronX
 */
public class Distance {
    /** Enum to represent the different distance units supported by the {@link Distance} class */
    public enum Units {
        INCHES,
        CENTIMETERS,
        METERS,
        MILLIMETERS,
        FEET,
        YARDS
    }

    private double baseDistance; // Inches

    // region Constants (unit to inch conversion factors)
    private static final double M_TO_INCH = 1000.0 / 25.4;
    private static final double CM_TO_INCH = 10.0 / 2.54;
    private static final double MM_TO_INCH = 1.0 / 25.4;
    private static final double FT_TO_INCH = 12.0;
    private static final double YD_TO_INCH = 36.0;
    // endregion

    // region Constructors and factory methods
    /**
     * Constructor for the {@link Distance} class
     * @param inches the distance in inches
     */
    public Distance(double inches) { this.baseDistance = inches; }

    /** Default constructor for the {@link Distance} class, initializes distance to 0 inches */
    public Distance() { this(0.0); }

    /** Factory method to create a {@link Distance} from inches */
    public static Distance fromIn(double inches) { return new Distance(inches); }

    /** Factory method to create a {@link Distance} from feet */
    public static Distance fromFt(double feet) { return new Distance(feet * FT_TO_INCH); }

    /** Factory method to create a {@link Distance} from yards */
    public static Distance fromYd(double yards) { return new Distance(yards * YD_TO_INCH); }

    /** Factory method to create a {@link Distance} from meters */
    public static Distance fromM(double meters) { return new Distance(meters * M_TO_INCH); }

    /** Factory method to create a {@link Distance} from centimeters */
    public static Distance fromCm(double centimeters) { return new Distance(centimeters * CM_TO_INCH); }

    /** Factory method to create a {@link Distance} from millimeters */
    public static Distance fromMm(double millimeters) { return new Distance(millimeters * MM_TO_INCH); }

    /** Factory method to create a {@link Distance} from a specified {@link Units} and value */
    public static Distance from(Units unit, double value) {
        switch(unit) {
            case INCHES: return fromIn(value);
            case FEET: return fromFt(value);
            case YARDS: return fromYd(value);
            case METERS: return fromM(value);
            case CENTIMETERS: return fromCm(value);
            case MILLIMETERS: return fromMm(value);
            default: throw new IllegalArgumentException("Unknown unit: " + unit);
        }
    }
    // endregion

    // region Getters for the distance in different units
    /** @return the distance in inches */
    public double getIn() { return this.baseDistance; }

    /** @return the distance in feet */
    public double getFt() { return this.baseDistance / FT_TO_INCH; }

    /** @return the distance in yards */
    public double getYd() { return this.baseDistance / YD_TO_INCH; }

    /** @return the distance in meters */
    public double getM() { return this.baseDistance / M_TO_INCH; }

    /** @return the distance in centimeters */
    public double getCm() { return this.baseDistance / CM_TO_INCH; }

    /** @return the distance in millimeters */
    public double getMm() { return this.baseDistance / MM_TO_INCH; }

    /** @return the distance in the specified {@link Units} */
    public double get(Units unit) {
        switch(unit) {
            case INCHES: return getIn();
            case FEET: return getFt();
            case YARDS: return getYd();
            case METERS: return getM();
            case CENTIMETERS: return getCm();
            case MILLIMETERS: return getMm();
            default: throw new IllegalArgumentException("Unknown unit: " + unit);
        }
    }
    // endregion

    // region Arithmetic operations
    /** @return a new {@link Distance} that is the sum of this distance and another distance */
    public Distance plus(Distance other) { return new Distance(this.baseDistance + other.baseDistance); }

    /** @return a new {@link Distance} that is the difference between this distance and another distance */
    public Distance subtract(Distance other) { return new Distance(this.baseDistance - other.baseDistance); }

    /** @return a new {@link Distance} that is this distance multiplied by a scalar value */
    public Distance multiply(double scalar) { return new Distance(this.baseDistance * scalar); }

    /** @return a new {@link Distance} that is the product of this distance and another distance */
    public Distance multiply(Distance other) { return new Distance(this.baseDistance * other.baseDistance); }

    /** @return a new {@link Distance} that is this distance divided by a scalar value */
    public Distance divide(double scalar) { return new Distance(this.baseDistance / scalar); }

    /** @return a new {@link Distance} that is the quotient of this distance and another distance */
    public Distance divide(Distance other) { return new Distance(this.baseDistance / other.baseDistance); }

    /** @return a new {@link Distance} that is the absolute value of this distance */
    public Distance abs() { return new Distance(Math.abs(this.baseDistance)); }

    /** Mirror the distance across the origin (negate the distance) in place */
    public void mirror() { this.baseDistance = -this.baseDistance; }
    // endregion

    // region Comparison operations
    /** @return true if this distance is equal to another distance, false otherwise */
    public boolean equals(Distance other) { return this.baseDistance == other.baseDistance; }

    /** @return true if this distance is less than another distance, false otherwise */
    public boolean lessThan(Distance other) { return this.baseDistance < other.baseDistance; }

    /** @return true if this distance is greater than another distance, false otherwise */
    public boolean greaterThan(Distance other) { return this.baseDistance > other.baseDistance; }
    // endregion

    // region Utility methods
    /** Return a copy of this {@link Distance} object */
    public Distance copy() { return new Distance(this.baseDistance); }

    @Override
    @NonNull
    public String toString() { return String.format("Distance(%s inches)", this.baseDistance); }
    // endregion
}