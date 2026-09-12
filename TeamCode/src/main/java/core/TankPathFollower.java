package core;

import com.qualcomm.robotcore.util.Range;

import controllers.DriveController;
import drivetrains.BaseDrivetrain;
import feedforward.MotionParameters;
import geometry.Angle;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import localizers.BaseLocalizer;
import paths.movements.Path;

/** Executes profiled paths for non-holonomic drivetrains using Ramsete feedback. */
final class TankPathFollower {
    private static final double MIN_PROFILE_VELOCITY_IN_PER_SECOND = 6.0;
    private static final double INCHES_TO_METRES = 0.0254;
    // Conventional Ramsete tuning assumes SI: b has units rad^2/m^2 and zeta has units 1/rad.
    // Short FTC paths need faster lateral convergence than the conventional b=2 default.
    private static final double RAMSETE_B = 8.0;
    private static final double RAMSETE_ZETA = 0.7;

    private final BaseDrivetrain<?> drivetrain;
    private final DriveController driveController;
    private final FollowerConstants constants;
    private final double distanceTolerance;
    private final double headingTolerance;
    private final controllers.PDSController endpointHeadingController;

    TankPathFollower(BaseDrivetrain<?> drivetrain, DriveController driveController,
                     FollowerConstants constants, double distanceTolerance,
                     double headingTolerance) {
        this.drivetrain = drivetrain;
        this.driveController = driveController;
        this.constants = constants;
        this.distanceTolerance = distanceTolerance;
        this.headingTolerance = headingTolerance;
        endpointHeadingController = new controllers.PDSController(constants.angularCoeffs);
        endpointHeadingController.setAngularController();
    }

    boolean update(Path path, PathSegment segment, Pose current, BaseLocalizer<?> localizer,
                   MovementCallbackRunner callbacks, FollowerPathState state,
                   double translationalKV, double translationalKA,
                   double angularKV, double angularKA,
                   double velocityFeedbackGain, double angularVelocityFeedbackGain) {
        // region 1. Path Information
        // 1.1 Robot pose and path geometry
        Vector currentPosition = current.getVec();
        Angle currentHeading = current.getHeading();
        state.t = segment.getBestT(currentPosition);
        Vector closestPathPoint = segment.getPosition(state.t);
        double distanceRemaining = segment.getDistanceToEndIn(closestPathPoint, state.t);

        // 1.2 Progress callbacks
        double pathProgress = 1.0 - distanceRemaining / segment.getLengthIn();
        callbacks.process(path, Range.clip(pathProgress, 0.0, 1.0), currentHeading);

        // 1.3 Path targets and measured velocity
        Vector pathDerivative = segment.getFirstDerivative(state.t);
        Vector robotVelocity = localizer.getVel().getVec();
        Angle headingTarget = path.getInterpolator().getHeadingTarg(
                distanceRemaining, pathDerivative, segment.getFirstDerivative(1.0));
        double distanceTraveled = path.getParametricPath().getLengthIn() - distanceRemaining;
        MotionParameters targets = path.getFeedforwardLut().getTankFFParams(distanceTraveled);

        // endregion

        // region 2. Algorithm
        // 2.1 Signed motion targets
        double driveSign = Math.cos(headingTarget.getRad() - pathDerivative.getTheta().getRad()) < 0
                ? -1.0 : 1.0;
        double targetVelocity = driveSign * (state.t < 1.0
                ? Math.max(targets.getTangentialVel(), MIN_PROFILE_VELOCITY_IN_PER_SECOND) : 0.0);
        double targetAcceleration = driveSign * targets.getTangentialAccel();
        double targetAngularVelocity = targets.getAngularVel();
        double targetAngularAcceleration = targets.getAngularAccel();

        // 2.2 Tracking error in the robot frame
        Vector globalError = closestPathPoint.minus(currentPosition);
        Vector tangent = pathDerivative.normalize();
        state.crossTrackError = globalError.getY().getIn() * tangent.getX().getIn()
                - globalError.getX().getIn() * tangent.getY().getIn();
        Vector localError = globalError.rotate(Angle.fromRad(-currentHeading.getRad()));
        double forwardError = localError.getX().getIn();
        double lateralError = localError.getY().getIn();
        double headingError = currentHeading.getShortestAngleTo(headingTarget).getRad();

        // 2.3 Ramsete pose feedback
        double targetVelocityMetres = targetVelocity * INCHES_TO_METRES;
        double forwardErrorMetres = forwardError * INCHES_TO_METRES;
        double lateralErrorMetres = lateralError * INCHES_TO_METRES;
        double k = 2.0 * RAMSETE_ZETA * Math.sqrt(
                targetAngularVelocity * targetAngularVelocity
                        + RAMSETE_B * targetVelocityMetres * targetVelocityMetres);
        double sinc = Math.abs(headingError) < 1e-6
                ? 1.0 : Math.sin(headingError) / headingError;

        // Ramsete returns a linear correction in m/s and an angular correction in rad/s. Convert
        // only the linear correction back to Apex's inch-based feedforward-gain convention.
        double forwardCorrectionInches = k * forwardErrorMetres / INCHES_TO_METRES;
        double correctedVelocity = targetVelocity * Math.cos(headingError) + forwardCorrectionInches;
        double correctedAngularVelocity = targetAngularVelocity + k * headingError
                + RAMSETE_B * targetVelocityMetres * sinc * lateralErrorMetres;
        double currentAngularVelocity = localizer.getVel().getHeading().getRad();

        // 2.4 Measured progress and tracking state
        double actualForwardVelocity = robotVelocity
                .rotate(Angle.fromRad(-currentHeading.getRad())).getX().getIn();
        state.trackingVelocityTarget = correctedVelocity;
        state.trackingMeasuredVelocity = actualForwardVelocity;
        state.trackingProgressVelocity = actualForwardVelocity * driveSign;
        state.trackingAngularVelocityTarget = correctedAngularVelocity;
        state.trackingEndpointBlend = 0.0;
        state.velocitySampleAvailable = true;

        // 2.5 Velocity feedback and motion feedforward
        double forwardVelocityFeedback =
                (correctedVelocity - actualForwardVelocity) * velocityFeedbackGain;
        double turnVelocityFeedback = angularVelocityFeedbackGain
                * (correctedAngularVelocity - currentAngularVelocity);
        double forwardFeedforward = MotionFeedforward.calculate(
                correctedVelocity,
                MotionFeedforward.scaleBrakingAcceleration(
                        targetVelocity, targetAcceleration, actualForwardVelocity),
                translationalKV, translationalKA, constants.translationalFeedforwardKS);

        // 2.6 Blend into endpoint capture
        double endError = driveSign * Follower.pathEndpointTangentError(
                path.getEndPose().getVec(), currentPosition,
                segment.getFirstDerivative(1.0).normalize());
        double endpointPower = driveController.calculateEndDistance(
                endError, -actualForwardVelocity);
        endpointPower = Follower.ensureEndpointBreakawayPower(
                endpointPower, endError, actualForwardVelocity,
                constants.translationalCoeffs.kS,
                PathCompletionPolicy.distanceTolerance(distanceTolerance), distanceRemaining);
        double endpointBlend = Follower.endpointCaptureBlend(distanceRemaining);
        endpointHeadingController.setCoefficients(constants.angularCoeffs);
        double endpointTurn = endpointHeadingController.calculate(
                currentHeading.getShortestAngleTo(path.getEndPose().getHeading()).getRad(),
                -currentAngularVelocity);
        state.trackingEndpointBlend = endpointBlend;
        state.trackingFeedforward = forwardFeedforward;
        state.trackingVelocityFeedback = forwardVelocityFeedback;
        double profilePower = forwardFeedforward + forwardVelocityFeedback;
        forwardFeedforward = forwardFeedforward * (1.0 - endpointBlend)
                + endpointPower * endpointBlend;
        forwardVelocityFeedback *= 1.0 - endpointBlend;
        forwardFeedforward += forwardVelocityFeedback;
        // As with holonomic braking overrides, endpoint capture must not dilute an
        // already stronger braking command while the robot is above its speed target.
        if (endpointBlend > 0.0 && profilePower * actualForwardVelocity < 0.0
                && driveSign * actualForwardVelocity > driveSign * correctedVelocity) {
            forwardFeedforward = driveSign * Math.min(driveSign * forwardFeedforward,
                    driveSign * profilePower);
        }

        // 2.7 Angular feedforward for Ramsete's corrected chassis speed
        double turnFeedforward = correctedAngularVelocity * angularKV
                + targetAngularAcceleration * angularKA;

        // 2.8 Completion and stuck detection
        double endpointDistance = currentPosition.distanceTo(path.getEndPose().getVec()).getIn();
        double endpointHeadingError = currentHeading
                .getShortestAngleTo(path.getEndPose().getHeading()).getRad();
        boolean insideTolerance = PathCompletionPolicy.isInsideTolerance(
                endpointDistance, endpointHeadingError, distanceTolerance, headingTolerance);
        boolean velocitySettled = PathCompletionPolicy.isVelocitySettled(
                robotVelocity.getMagSq().getIn(), currentAngularVelocity);
        // A dwell inside pose tolerance is not a substitute for actually slowing down.
        if (velocitySettled
                && state.completion.isComplete(insideTolerance, true, System.nanoTime())) {
            return true;
        }
        // End the movement if the robot remains stuck.
        if (state.stuckWatchdog.update(
                Follower.stuckForSpeed(robotVelocity.getMag().getIn()), System.nanoTime())) {
            return true;
        }

        // endregion

        // region 3. Send powers
        double turnPower = (turnFeedforward + turnVelocityFeedback) * (1.0 - endpointBlend)
                + endpointTurn * endpointBlend;
        double powerLimit = drivetrain.getConstants().maxPower;
        // Preserve steering authority under saturation. Common wheel normalization would
        // reduce the requested turn whenever translation consumes the shared power budget.
        // For a tank, max(|forward-turn|, |forward+turn|) = |forward| + |turn|.
        turnPower = Range.clip(turnPower, -powerLimit, powerLimit);
        double forwardBudget = Math.max(0.0, powerLimit - Math.abs(turnPower));
        drivetrain.drive(Range.clip(forwardFeedforward, -forwardBudget, forwardBudget),
                0.0, turnPower);
        // endregion

        return false;
    }
}
