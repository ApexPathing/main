package followers.constants;

import controllers.PDFLController.PDFLCoefficients;
import controllers.PDFLController;
import drivetrains.Drivetrain;
import followers.P2PFollower;
import localizers.Localizer;
import util.Angle;
import util.Distance;

/**
 * Point to point follower constants class
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class P2PFollowerConstants extends FollowerConstants {
    // Tunable constants
    public PDFLCoefficients axialCoeffs = new PDFLCoefficients();
    public PDFLCoefficients strafeCoeffs = new PDFLCoefficients();
    public PDFLCoefficients headingCoeffs = new PDFLCoefficients();

    // Controllers
    public PDFLController axialController;
    public PDFLController strafeController;
    public PDFLController headingController;

    // Power limits
    public double maxTranslationalPower = 1.0;
    public double maxRotationalPower = 1.0;

    /**
     * Constructor for the P2PFollowerConstants class
     */
    public P2PFollowerConstants() {
        this.axialController = new PDFLController(axialCoeffs);
        this.strafeController = new PDFLController(strafeCoeffs);
        this.headingController = new PDFLController(headingCoeffs);
        this.headingController.useAsAngularController();
    }

    @Override
    public P2PFollower build(Drivetrain drivetrain, Localizer localizer) {
        this.axialController.setCoefficients(axialCoeffs);
        this.strafeController.setCoefficients(strafeCoeffs);
        this.headingController.setCoefficients(headingCoeffs);
        return new P2PFollower(this, drivetrain, localizer);
    }

    // region Setters
    /**
     * Sets the PDFL coefficients for the axial controller.
     * @param coeffs the new axial {@link PDFLCoefficients}
     * @return this instance for chaining
     */
    public P2PFollowerConstants setAxialCoeffs(PDFLCoefficients coeffs) {
        this.axialCoeffs = coeffs;
        return this;
    }

    /**
     * Sets the PDFL coefficients for the strafe controller.
     * @param coeffs the new strafe {@link PDFLCoefficients}
     * @return this instance for chaining
     */
    public P2PFollowerConstants setStrafeCoeffs(PDFLCoefficients coeffs) {
        this.strafeCoeffs = coeffs;
        return this;
    }

    /**
     * Sets the PDFL coefficients for the heading controller.
     * @param coeffs the new heading {@link PDFLCoefficients}
     * @return this instance for chaining
     */
    public P2PFollowerConstants setHeadingCoeffs(PDFLCoefficients coeffs) {
        this.headingCoeffs = coeffs;
        return this;
    }

    /**
     * Sets the translational error tolerance for the robot to be considered "at the target".
     * @param translationalTolerance the tolerance in inches
     * @return this instance for chaining
     */
    public P2PFollowerConstants setTranslationalTolerance(Distance translationalTolerance) {
        this.axialController.setTolerance(translationalTolerance);
        this.strafeController.setTolerance(translationalTolerance);
        return this;
    }

    /**
     * Sets the heading error tolerance for the robot to be considered "at the target".
     * @param headingTolerance the tolerance in degrees
     * @return this instance for chaining
     */
    public P2PFollowerConstants setHeadingTolerance(Angle headingTolerance) {
        this.headingController.setTolerance(headingTolerance);
        return this;
    }

    /**
     * Sets the maximum translational power that the follower can output.
     * Note that drivetrain power limits take precedence over this and this only affects following
     * @param maxTranslationalPower the maximum translational power (0 to 1)
     * @return this instance for chaining
     */
    public P2PFollowerConstants setMaxTranslationalPower(double maxTranslationalPower) {
        this.maxTranslationalPower = maxTranslationalPower;
        return this;
    }

    /**
     * Sets the maximum rotational power that the follower can output.
     * Note that drivetrain power limits take precedence over this and this only affects following
     * @param maxRotationalPower the maximum rotational power (0 to 1)
     * @return this instance for chaining
     */
    public P2PFollowerConstants setMaxRotationalPower(double maxRotationalPower) {
        this.maxRotationalPower = maxRotationalPower;
        return this;
    }
    // endregion
}
