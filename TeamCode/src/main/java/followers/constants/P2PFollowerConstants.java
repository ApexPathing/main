package followers.constants;

import controllers.PDLController;
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
    public PDLController.Coefficients axialCoeffs = new PDLController.Coefficients();
    public PDLController.Coefficients strafeCoeffs = new PDLController.Coefficients();
    public PDLController.Coefficients headingCoeffs = new PDLController.Coefficients();

    // Controllers
    public PDLController axialController;
    public PDLController strafeController;
    public PDLController headingController;

    // Power limits
    public double maxTranslationalPower = 1.0;
    public double maxRotationalPower = 1.0;

    /**
     * Constructor for the P2PFollowerConstants class
     */
    public P2PFollowerConstants() {
        this.axialController = new PDLController(axialCoeffs);
        this.strafeController = new PDLController(strafeCoeffs);
        this.headingController = new PDLController(headingCoeffs);
        this.headingController.useAsAngularController();
    }

    @Override
    public P2PFollower build(Drivetrain drivetrain, Localizer localizer) {
        this.axialController.setPDLCoefficients(axialCoeffs);
        this.strafeController.setPDLCoefficients(strafeCoeffs);
        this.headingController.setPDLCoefficients(headingCoeffs);
        return new P2PFollower(this, drivetrain, localizer);
    }

    // region Setters
    /**
     * Sets the PDL coefficients for the axial controller.
     * @param coeffs the new axial {@link PDLController.Coefficients}
     * @return this instance for chaining
     */
    public P2PFollowerConstants setAxialCoeffs(PDLController.Coefficients coeffs) {
        this.axialCoeffs = coeffs;
        return this;
    }

    /**
     * Sets the PDL coefficients for the strafe controller.
     * @param coeffs the new strafe {@link PDLController.Coefficients}
     * @return this instance for chaining
     */
    public P2PFollowerConstants setStrafeCoeffs(PDLController.Coefficients coeffs) {
        this.strafeCoeffs = coeffs;
        return this;
    }

    /**
     * Sets the PDL coefficients for the heading controller.
     * @param coeffs the new heading {@link PDLController.Coefficients}
     * @return this instance for chaining
     */
    public P2PFollowerConstants setHeadingCoeffs(PDLController.Coefficients coeffs) {
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
