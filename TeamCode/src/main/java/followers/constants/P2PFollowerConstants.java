package followers.constants;

import controllers.PDFLController;
import controllers.VectorControllers.PDLVectorController;

/**
 * Point to point follower constants class
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class P2PFollowerConstants {
    // Tunable constants
    public double translationalGain = 0.03;
    public double translationalD = 0.0;
    public double headingGain = 0.5;
    public double headingD = 0.0;

    // Tolerances

    // Power limits while following (note that these may be overridden by the drivetrain's power limits)
    public double maxPower = 1.0;
    public double minPower = 0.05;

    // Controllers
    public final PDLVectorController translationalController;
    public final PDFLController headingController;

    /**
     * Constructor for the P2PFollowerConstants class
     */
    public P2PFollowerConstants() {
        this.translationalController = new PDLVectorController(translationalGain, translationalD, minPower);
        this.headingController = new PDFLController(headingGain, headingD, 0.0, minPower);
        headingController.useAsAngularController();
    }

    // region Setters

    /**
     * Sets the translational proportional gain.
     * @param translationalGain the translational Kp
     * @return this instance for chaining
     */
    public P2PFollowerConstants setTranslationalGain(double translationalGain) {
        this.translationalGain = translationalGain;
        this.translationalController.setPDLCoefficients(translationalGain, translationalD, minPower);
        return this;
    }

    /**
     * Sets the translational derivative gain to mitigate overshoot
     * @param translationalD the derivative gain for translational movement
     * @return this instance for chaining
     */
    public P2PFollowerConstants setTranslationalD(double translationalD) {
        this.translationalD = translationalD;
        this.translationalController.setPDLCoefficients(translationalGain, translationalD, minPower);
        return this;
    }

    /**
     * Sets the heading proportional gain.
     * @param headingGain the heading Kp
     * @return this instance for chaining
     */
    public P2PFollowerConstants setHeadingGain(double headingGain) {
        this.headingGain = headingGain;
        this.headingController.setPDFLCoefficients(headingGain, headingD, 0.0, minPower);
        return this;
    }

    /**
     * Sets the heading derivative gain to mitigate heading overshoot
     * @param headingD the heading derivative gain
     * @return this instance for chaining
     */
    public P2PFollowerConstants setHeadingD(double headingD) {
        this.headingD = headingD;
        this.headingController.setPDFLCoefficients(headingGain, headingD, 0.0, minPower);
        return this;
    }

    /**
     * Sets the translational tolerance.
     * @param translationalTolerance the tolerance in inches
     * @return this instance for chaining
     */
    public P2PFollowerConstants setTranslationalTolerance(double translationalTolerance) {
        this.translationalController.setTolerance(translationalTolerance);
        return this;
    }

    /**
     * Sets the heading tolerance.
     * @param headingToleranceDegrees the tolerance in degrees
     * @return this instance for chaining
     */
    public P2PFollowerConstants setHeadingTolerance(double headingToleranceDegrees) {
        this.headingController.setTolerance(Math.toRadians(headingToleranceDegrees));
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
        this.headingController.setPDFLCoefficients(headingGain, headingD, 0.0, minPower);
        this.translationalController.setPDLCoefficients(translationalGain, translationalD, minPower);
        return this;
    }

    // endregion
}
