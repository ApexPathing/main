package followers.constants;

import controllers.PDSController.PDSCoefficients;
import controllers.PDSController;
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
    public PDSCoefficients axialCoeffs = new PDSCoefficients();
    public PDSCoefficients strafeCoeffs = new PDSCoefficients();
    public PDSCoefficients headingCoeffs = new PDSCoefficients();

    // Controllers
    public PDSController axialController;
    public PDSController strafeController;
    public PDSController headingController;

    /**
     * Constructor for the P2PFollowerConstants class
     */
    public P2PFollowerConstants() {
        this.axialController = new PDSController(axialCoeffs);
        this.strafeController = new PDSController(strafeCoeffs);
        this.headingController = new PDSController(headingCoeffs);
        this.headingController.setAngularController();
    }

    @Override
    public P2PFollower build(Drivetrain drivetrain, Localizer localizer) {
        return new P2PFollower(this, drivetrain, localizer);
    }

    // region Setters
    /**
     * Sets the PDS coefficients for the axial controller.
     * @param coeffs the new axial {@link PDSCoefficients}
     * @return this instance for chaining
     */
    public P2PFollowerConstants setAxialCoeffs(PDSCoefficients coeffs) {
        this.axialCoeffs = coeffs;
        this.axialController.setCoefficients(coeffs);
        return this;
    }

    /**
     * Sets the PDS coefficients for the strafe controller.
     * @param coeffs the new strafe {@link PDSCoefficients}
     * @return this instance for chaining
     */
    public P2PFollowerConstants setStrafeCoeffs(PDSCoefficients coeffs) {
        this.strafeCoeffs = coeffs;
        this.strafeController.setCoefficients(coeffs);
        return this;
    }

    /**
     * Sets the PDS coefficients for the heading controller.
     * @param coeffs the new heading {@link PDSCoefficients}
     * @return this instance for chaining
     */
    public P2PFollowerConstants setHeadingCoeffs(PDSCoefficients coeffs) {
        this.headingCoeffs = coeffs;
        this.headingController.setCoefficients(coeffs);
        return this;
    }

    /**
     * Sets the axial error tolerance for the robot to be considered "at the target".
     * @param axialTolerance the tolerance in inches
     * @return this instance for chaining
     */
    public P2PFollowerConstants setAxialTolerance(Distance axialTolerance) {
        this.axialController.setTolerance(axialTolerance);
        return this;
    }

    /**
     * Sets the strafe error tolerance for the robot to be considered "at the target".
     * @param strafeTolerance the tolerance in inches
     * @return this instance for chaining
     */
    public P2PFollowerConstants setStrafeTolerance(Distance strafeTolerance) {
        this.strafeController.setTolerance(strafeTolerance);
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
     * Sets the maximum axial power that the follower can output.
     * Note that drivetrain power limits take precedence over this and this only affects following
     * @param maxAxialPower the maximum translational power (0 to 1)
     * @return this instance for chaining
     */
    public P2PFollowerConstants setMaxAxialPower(double maxAxialPower) {
        this.axialController.setMaxOutput(maxAxialPower);
        return this;
    }

    /**
     * Sets the maximum strafe power that the follower can output.
     * Note that drivetrain power limits take precedence over this and this only affects following
     * @param maxStrafePower the maximum strafe power (0 to 1)
     * @return this instance for chaining
     */
    public P2PFollowerConstants setMaxStrafePower(double maxStrafePower) {
        this.strafeController.setMaxOutput(maxStrafePower);
        return this;
    }

    /**
     * Sets the maximum rotational power that the follower can output.
     * Note that drivetrain power limits take precedence over this and this only affects following
     * @param maxTurnPower the maximum rotational power (0 to 1)
     * @return this instance for chaining
     */
    public P2PFollowerConstants setMaxTurnPower(double maxTurnPower) {
        this.headingController.setMaxOutput(maxTurnPower);
        return this;
    }

    /**
     * Sets the deadzone for the axial controller. The controller will output 0 if the error is
     * within the range of [-deadzone, deadzone].
     */
    public P2PFollowerConstants setAxialDeadzone(double axialDeadzone) {
        this.axialController.setDeadzone(axialDeadzone);
        return this;
    }

    /**
     * Sets the deadzone for the strafe controller. The controller will output 0 if the error is
     * within the range of [-deadzone, deadzone].
     */
    public P2PFollowerConstants setStrafeDeadzone(double strafeDeadzone) {
        this.strafeController.setDeadzone(strafeDeadzone);
        return this;
    }

    /**
     * Sets the deadzone for the heading controller. The controller will output 0 if the error is
     * within the range of [-deadzone, deadzone].
     */
    public P2PFollowerConstants setHeadingDeadzone(double headingDeadzone) {
        this.headingController.setDeadzone(headingDeadzone);
        return this;
    }
    // endregion
}
