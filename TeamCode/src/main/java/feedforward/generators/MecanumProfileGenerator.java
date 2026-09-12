package feedforward.generators;

import core.FollowerConstants;
import drivetrains.Mecanum;
import drivetrains.Mecanum.DirectionalLut.DirectionalKinematics;
import geometry.Angle;
import geometry.PathPoint;
import geometry.Vector;
import paths.movements.Path;

/**
 * Generates holonomic profiles for mecanum drives, including direction-specific limits.
 *
 * <p>Unlike ideal swerve, mecanum does not have equal authority in every robot-relative direction.
 * The directional LUT scales velocity and acceleration costs so a diagonal/strafe-heavy segment
 * gets a lower limit than an efficient forward segment.
 *
 * @author DrPixelCat - 7842 alum
 */
public class MecanumProfileGenerator extends BaseProfileGenerator {
    /** Number of binary-search steps used for local velocity ceilings. */
    private static final int VELOCITY_SEARCH_ITERATIONS = 20;

    /** Direction-aware velocity/acceleration model for mecanum translation. */
    private final Mecanum.DirectionalLut limitCalculator;

    /** Creates a mecanum profile generator for a path. */
    public MecanumProfileGenerator(FollowerConstants constants, Path path) {
        super(constants, path);
        this.limitCalculator = new Mecanum.DirectionalLut(
                constants.forwardVelLimitIn, constants.forwardAccelLimitIn,
                constants.strafeVelLimitIn, constants.strafeAccelLimitIn
        );
    }

    @Override
    protected double calculateMaxTangentialVelocity(PathPoint point, Path path, double maxAngVel,
                                                    double maxAngAccel) {
        double s = point.getDistanceToEndIn();
        Vector tangent = point.getFirstDerivative();
        double kappa = point.getSignedCurvature();
        double dKappa = point.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        Angle headingAtPoint = path.getInterpolator().getHeadingTarg(s, tangent, finalTangent);
        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                finalTangent);

        DirectionalKinematics dirK = limitCalculator.getKinematics(tangent, headingAtPoint);
        double maxPhysicalVel = dirK.maxVel;

        double effectiveAngVelLimit = Math.min(constants.angularVelLimitRad, maxAngVel);
        double effectiveAngAccelLimit = Math.min(constants.angularAccelLimitRad,
                maxAngAccel);

        // Angular velocity limit: |f' * v| <= omega_max.
        if (Math.abs(fPrime) > EPSILON) {
            double maxVelFromOmega = effectiveAngVelLimit / Math.abs(fPrime);
            maxPhysicalVel = Math.min(maxPhysicalVel, maxVelFromOmega);
        }

        // Angular acceleration limit at zero tangential accel: |f'' * v^2| <= alpha_max.
        if (Math.abs(fDoublePrime) > EPSILON) {
            double maxVelFromAlpha = Math.sqrt(effectiveAngAccelLimit / Math.abs(fDoublePrime));
            maxPhysicalVel = Math.min(maxPhysicalVel, maxVelFromAlpha);
        }

        double min_v = 0.0;
        double max_v = maxPhysicalVel;

        // Directional multipliers make the closed-form limit messy; binary search is cheap here.
        for (int i = 0; i < VELOCITY_SEARCH_ITERATIONS; i++) {
            double mid_v = (min_v + max_v) / 2.0;

            if (evaluatePower(mid_v, kappa, fPrime, fDoublePrime, dirK) > 1.0) {
                max_v = mid_v;
            } else { min_v = mid_v; }
        }

        return Math.min(min_v, maxPhysicalVel);
    }

    /**
     * Estimates normalized mecanum power for a local state.
     *
     * <p>The tangential term uses the direction-dependent mecanum velocity multiplier. The
     * centripetal coefficient already represents the drivetrain's lateral acceleration authority.
     */
    private double evaluatePower(double v, double kappa, double fPrime,
                                 double fDoublePrime, DirectionalKinematics tangentKinematics) {
        // Apply the LUT as power cost multipliers instead of pretending strafe is as efficient.
        double boostedKV = constants.translationalKV * tangentKinematics.velMultiplier;
        double transPower = v * boostedKV
                + signedStatic(v, 0.0, constants.translationalFeedforwardKS);

        // kCentripetal is tuned directly from the drivetrain's full-power lateral acceleration.
        // Applying the directional acceleration multiplier here would charge the mecanum strafe
        // penalty twice and make every curved profile unnecessarily slow.
        double latPower = Math.abs(v * v * kappa * constants.kCentripetal);

        double omega = fPrime * v;
        double alpha = fDoublePrime * (v * v);
        double headingKs = signedStatic(omega, alpha, constants.angularFeedforwardKS);
        double rotPower =
                Math.abs(omega * constants.angularKV + alpha * constants.angularKA + headingKs);

        return transPower + latPower + rotPower;
    }

    @Override
    protected void evaluatePoint(Path path, PathPoint prev, PathPoint current, double v_prev,
                                 double v, double a_t, EvaluationResult outResult) {
        double s = current.getDistanceToEndIn();
        double kappa = current.getSignedCurvature();
        double dKappa = current.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);
        Angle robotHeading = path.getInterpolator().getHeadingTarg(
                s, current.getFirstDerivative(), finalTangent
        );

        DirectionalKinematics dirK = limitCalculator.getKinematics(current.getFirstDerivative(),
                robotHeading);
        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                finalTangent);

        double omega = fPrime * v;
        double alpha = fDoublePrime * (v * v) + fPrime * a_t;

        double pForward = v * constants.translationalKV * dirK.velMultiplier
                + a_t * constants.getTranslationalKA(v, a_t) * dirK.accelMultiplier
                + signedStatic(v, a_t, constants.translationalFeedforwardKS);

        // kCentripetal already maps lateral acceleration to motor power for this drivetrain.
        double pLateral = v * v * kappa * constants.kCentripetal;

        double headingKs = signedStatic(omega, alpha, constants.angularFeedforwardKS);
        double pHeading = omega * constants.angularKV + alpha * constants.angularKA + headingKs;

        outResult.pForward = Math.abs(pForward);
        outResult.pLateral = Math.abs(pLateral);
        outResult.pHeading = Math.abs(pHeading);

        outResult.totalPower = outResult.pForward + outResult.pLateral + outResult.pHeading;
        outResult.maxUtilization = outResult.totalPower;
    }

    @Override
    protected double getMaxTangentialAccel(double currentVel, PathPoint point, Path path,
                                           double maxAngAccel) {
        return calculateAngularLimitedTangentialAccel(currentVel, point, path, maxAngAccel, false);
    }

    @Override
    protected double calculateDynamicMaxAccel(double currentVel, PathPoint point, Path path,
                                              double maxAngAccel) {
        return calculateAngularLimitedTangentialAccel(currentVel, point, path, maxAngAccel, true);
    }

    /**
     * Limits tangential acceleration so heading acceleration stays inside angular constraints.
     *
     * <p>The same chain-rule relationship applies here: {@code alpha = f'' * v^2 + f' * a}.
     * Directional mecanum acceleration then clamps the result further with {@code dirK.maxAccel}.
     */
    private double calculateAngularLimitedTangentialAccel(double currentVel, PathPoint point,
                                                          Path path, double maxAngAccel,
                                                          boolean positiveAccel) {
        double s = point.getDistanceToEndIn();
        Vector tangent = point.getFirstDerivative();
        double kappa = point.getSignedCurvature();
        double dKappa = point.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        Angle robotHeading = path.getInterpolator().getHeadingTarg(s, tangent, finalTangent);
        DirectionalKinematics dirK = limitCalculator.getKinematics(tangent, robotHeading);

        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                finalTangent);

        double maxPhysicalAccel = dirK.maxAccel;
        double effectiveAngAccelLimit = Math.min(constants.angularAccelLimitRad,
                maxAngAccel);
        if (effectiveAngAccelLimit < EPSILON) { return 0.0; }

        double alphaBase = fDoublePrime * currentVel * currentVel;
        if (Math.abs(fPrime) < EPSILON) {
            // With no dtheta/ds term, tangential accel cannot reduce angular acceleration.
            return Math.abs(alphaBase) <= effectiveAngAccelLimit + EPSILON ? maxPhysicalAccel : 0.0;
        }

        double boundA = (-effectiveAngAccelLimit - alphaBase) / fPrime;
        double boundB = (effectiveAngAccelLimit - alphaBase) / fPrime;
        double minAccel = Math.min(boundA, boundB);
        double maxAccel = Math.max(boundA, boundB);

        double angularLimitedAccel = positiveAccel ? maxAccel : -minAccel;
        return Math.min(maxPhysicalAccel, Math.max(0.0, angularLimitedAccel));
    }

}
