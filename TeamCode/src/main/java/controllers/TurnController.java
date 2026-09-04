package controllers;

import feedforward.MotionParameters;

/**
 * Executes quick and displacement-profiled point turns.
 *
 * <p>Quick turns use the complete heading PDS controller. Profiled turns track a time-indexed
 * heading, angular velocity, and angular acceleration reference.
 *
 * @author DrPixelCat - 7842 alum
 */
public class TurnController {
    private static final double EPSILON = 1e-6;
    // Preserve breakaway authority through the low-speed end of profile deceleration. Waiting
    // until nearly zero lets static friction stop the robot for a loop before recovery restarts it.
    private static final double LOW_SPEED_ANGULAR_VELOCITY = 0.25;
    private static final double BREAKAWAY_RESERVE = 0.02;
    // Velocity feedback is a correction, not the primary command. Bounding its contribution keeps
    // an over-tuned/stale gain from turning profile tracking into full-power bang-bang control.
    private static final double MAX_ACCELERATING_FEEDBACK_POWER = 0.50;
    private static final double MAX_BRAKING_FEEDBACK_POWER = 0.25;

    private final PDSController headingPds;
    private double angularKV;
    private double angularKA;
    private double angularFeedforwardKS;
    private double angularVelocityFeedbackGain;

    private boolean quickEndpointCapture;

    public TurnController(PDSController.PDSCoefficients headingCoefficients,
                          double angularKV, double angularKA, double angularFeedforwardKS,
                          double angularVelocityFeedbackGain) {
        headingPds = new PDSController(headingCoefficients);
        headingPds.setAngularController();

        this.angularKV = angularKV;
        this.angularKA = angularKA;
        this.angularFeedforwardKS = angularFeedforwardKS;
        this.angularVelocityFeedbackGain = angularVelocityFeedbackGain;
    }

    public void setCoefficients(PDSController.PDSCoefficients coefficients) {
        headingPds.setCoefficients(coefficients);
        reset();
    }

    public void setMotionGains(double angularKV, double angularKA,
                               double angularVelocityFeedbackGain) {
        setMotionGains(angularKV, angularKA, angularFeedforwardKS,
                angularVelocityFeedbackGain);
    }

    /** Updates dynamic and moving-friction feedforward gains. */
    public void setMotionGains(double angularKV, double angularKA,
                               double angularFeedforwardKS,
                               double angularVelocityFeedbackGain) {
        this.angularKV = angularKV;
        this.angularKA = angularKA;
        this.angularFeedforwardKS = angularFeedforwardKS;
        this.angularVelocityFeedbackGain = angularVelocityFeedbackGain;
        reset();
    }

    /** Uses the complete heading PDS for an unprofiled turn. */
    public double calculateQuick(double headingError) { return headingPds.calculate(headingError); }

    /** Latches out static compensation after a quick turn reaches its endpoint band. */
    public double calculateQuick(double headingError, double measuredAngularVelocity,
                                 double endpointCaptureHeading) {
        if (Math.abs(headingError) <= endpointCaptureHeading) {
            quickEndpointCapture = true;
        }
        return headingPds.calculate(
                headingError, -measuredAngularVelocity, !quickEndpointCapture);
    }

    /**
     * Calculates a profiled turn command from a time-indexed position and velocity error.
     */
    public double calculateProfiled(double headingError, double intendedDirection,
                                    MotionParameters targets, double measuredAngularVelocity) {
        double targetVelocity = targets.getAngularVel();
        double targetAcceleration = targets.getAngularAccel();

        double motionSign = 0.0;
        if (Math.abs(targetVelocity) > EPSILON) {
            motionSign = Math.signum(targetVelocity);
        } else if (Math.abs(targetAcceleration) > EPSILON) {
            motionSign = Math.signum(targetAcceleration);
        }

        double feedforward = angularKV * targetVelocity + angularKA * targetAcceleration
                + angularFeedforwardKS * motionSign;
        double velocityFeedback = Math.abs(targetVelocity) > EPSILON
                ? clipVelocityFeedback(angularVelocityFeedbackGain
                * (targetVelocity - measuredAngularVelocity), intendedDirection)
                : 0.0;

        double errorRate = targetVelocity - measuredAngularVelocity;
        boolean stoppedReference = Math.abs(targetVelocity) <= EPSILON &&
                Math.abs(measuredAngularVelocity) < LOW_SPEED_ANGULAR_VELOCITY;
        double positionPower = headingPds.calculate(
                headingError, errorRate, stoppedReference);
        double requestedPower = feedforward + velocityFeedback + positionPower;
        return clip(ensureProfiledMotionBreakaway(
                requestedPower,
                targetVelocity,
                measuredAngularVelocity,
                headingPds.getCoefficients().kS
        ));
    }

    /**
     * Restores enough authority to restart a profiled turn if deceleration feedforward cancels
     * the velocity/static terms below breakaway while the profile still requests motion.
     */
    static double ensureProfiledMotionBreakaway(double requestedPower, double targetVelocity,
                                                 double measuredVelocity, double staticGain) {
        if (Math.abs(targetVelocity) <= EPSILON ||
                Math.abs(measuredVelocity) >= LOW_SPEED_ANGULAR_VELOCITY) {
            return requestedPower;
        }

        double minimumPower = Math.min(1.0, Math.abs(staticGain) + BREAKAWAY_RESERVE);
        if (Math.abs(requestedPower) >= minimumPower &&
                Math.signum(requestedPower) == Math.signum(targetVelocity)) {
            return requestedPower;
        }
        return Math.copySign(minimumPower, targetVelocity);
    }

    public void reset() {
        quickEndpointCapture = false;
        headingPds.reset();
    }

    private static double clip(double power) { return Math.max(-1.0, Math.min(1.0, power)); }

    private static double clipVelocityFeedback(double feedback, double intendedDirection) {
        if (Math.abs(intendedDirection) <= EPSILON) {
            return Math.max(-MAX_BRAKING_FEEDBACK_POWER,
                    Math.min(MAX_BRAKING_FEEDBACK_POWER, feedback));
        }
        double alongDirection = feedback * Math.signum(intendedDirection);
        double bounded = Math.max(-MAX_BRAKING_FEEDBACK_POWER,
                Math.min(MAX_ACCELERATING_FEEDBACK_POWER, alongDirection));
        return bounded * Math.signum(intendedDirection);
    }
}
