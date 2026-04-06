package followers.constants;

/**
 * Point to point follower constants class
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class P2PFollowerConstants {
    // Tunable constants
    public double translationalKp = 0.03;
    public double headingKp = 0.5;

    // Tolerances
    public double translationalTolerance = 1.0; // Inches
    public double headingTolerance = Math.toRadians(3); // Radians (user enters in degrees)

    // Power limits while following (note that these may be overridden by the drivetrain's power limits)
    public double maxPower = 1.0;
    public double minPower = 0.05;

    /**
     * Constructor for the P2PFollowerConstants class
     */
    public P2PFollowerConstants() {}

    /**
     * Sets the translational proportional gain.
     * @param translationalKp the translational Kp
     * @return this instance for chaining
     */
    public P2PFollowerConstants setTranslationalKp(double translationalKp) {
        this.translationalKp = translationalKp;
        return this;
    }

    /**
     * Sets the heading proportional gain.
     * @param headingKp the heading Kp
     * @return this instance for chaining
     */
    public P2PFollowerConstants setHeadingKp(double headingKp) {
        this.headingKp = headingKp;
        return this;
    }

    /**
     * Sets the translational tolerance.
     * @param translationalTolerance the tolerance in inches
     * @return this instance for chaining
     */
    public P2PFollowerConstants setTranslationalTolerance(double translationalTolerance) {
        this.translationalTolerance = translationalTolerance;
        return this;
    }

    /**
     * Sets the heading tolerance.
     * @param headingToleranceDegrees the tolerance in degrees
     * @return this instance for chaining
     */
    public P2PFollowerConstants setHeadingTolerance(double headingToleranceDegrees) {
        this.headingTolerance = Math.toRadians(headingToleranceDegrees);
        return this;
    }

    /**
     * Sets the maximum power.
     * @param maxPower the maximum power limit
     * @return this instance for chaining
     */
    public P2PFollowerConstants setMaxPower(double maxPower) {
        this.maxPower = maxPower;
        return this;
    }

    /**
     * Sets the minimum power.
     * @param minPower the minimum power limit
     * @return this instance for chaining
     */
    public P2PFollowerConstants setMinPower(double minPower) {
        this.minPower = minPower;
        return this;
    }
}
