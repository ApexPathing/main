package core;

import com.qualcomm.robotcore.util.Range;

import controllers.DriveController;
import controllers.PDSController;
import drivetrains.BaseDrivetrain;
import feedforward.MotionParameters;
import geometry.Angle;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import localizers.BaseLocalizer;
import paths.movements.Path;

/** Executes quick and profiled paths for holonomic drivetrains. */
final class HolonomicPathFollower {
    private static final double MIN_PROFILE_VELOCITY_IN_PER_SECOND = 6.0;

    private final BaseDrivetrain<?> drivetrain;
    private final DriveController driveController;
    private final PDSController headingController;
    private final HolonomicCommandAllocator allocator;
    private final FollowerConstants constants;
    private final double distanceTolerance;
    private final double headingTolerance;

    HolonomicPathFollower(BaseDrivetrain<?> drivetrain, DriveController driveController,
                          PDSController headingController, HolonomicCommandAllocator allocator,
                          FollowerConstants constants, double distanceTolerance,
                          double headingTolerance) {
        this.drivetrain = drivetrain;
        this.driveController = driveController;
        this.headingController = headingController;
        this.allocator = allocator;
        this.constants = constants;
        this.distanceTolerance = distanceTolerance;
        this.headingTolerance = headingTolerance;
    }

    boolean update(Path path, PathSegment segment, Pose current, BaseLocalizer<?> localizer,
                   MovementCallbackRunner callbacks, FollowerPathState state,
                   boolean headingControllerEnabled, boolean driveControllerEnabled,
                   double translationalKV, double translationalKA,
                   double angularKV, double angularKA, double centripetalGain,
                   double velocityFeedbackGain, double angularVelocityFeedbackGain) {
        // region 1. Path Information
        // 1.1 Robot pose and path geometry
        Vector currentPosition = current.getVec();
        Angle currentHeading = current.getHeading();
        state.t = segment.getBestT(currentPosition);
        state.closestPathPoint = segment.getPosition(state.t);

        Vector firstDerivative = segment.getFirstDerivative(state.t);
        Vector secondDerivative = segment.getSecondDerivative(state.t);
        Vector unitTangent = firstDerivative.normalize();
        Vector lateralNormal = PathSegment.calculateLeftNormal(firstDerivative);
        Vector principalNormal = PathSegment.calculateArcNormal(firstDerivative, secondDerivative);
        state.crossTrackNormal = lateralNormal;
        state.pathNormal = principalNormal;
        Vector endTangent = segment.getFirstDerivative(1.0).normalize();
        double distanceRemaining = segment.getDistanceToEndIn(state.closestPathPoint, state.t);
        double signedEndpointError = Follower.pathEndpointTangentError(
                path.getEndPose().getVec(), currentPosition, endTangent);
        double distanceTraveled = path.getParametricPath().getLengthIn() - distanceRemaining;
        double curvature = segment.getSignedCurvature(state.t);

        // 1.2 Progress callbacks
        double pathProgress = 1.0 - distanceRemaining / segment.getLengthIn();
        callbacks.process(path, Range.clip(pathProgress, 0.0, 1.0), currentHeading);

        // 1.3 Motion targets
        boolean profiled = path.isProfiled();
        MotionParameters targets = profiled
                ? path.getFeedforwardLut().getFFParams(distanceTraveled) : null;
        Vector robotVelocity = localizer.getVel().getVec();
        double quickProgress = Range.clip(
                1.0 - distanceRemaining / path.getParametricPath().getLengthIn(), 0.0, 1.0);
        double commandedForwardVelocity = profiled
                ? targets.getTangentialVel()
                : path.getQuickVelocityLimit(quickProgress, constants.forwardVelLimitIn);
        boolean applyCorrectiveStatic = Math.abs(commandedForwardVelocity)
                < 0.10 * constants.forwardVelLimitIn;

        // 1.4 Measured progress and tracking state
        allocator.activeDriveModel();
        double measuredTangentialVelocity = robotVelocity.dot(unitTangent).getIn();
        double progressVelocity = state.progressEstimator.update(
                distanceTraveled, measuredTangentialVelocity, System.nanoTime());
        double robotTangentialVelocity = measuredTangentialVelocity;
        state.trackingMeasuredVelocity = measuredTangentialVelocity;
        state.trackingProgressVelocity = progressVelocity;
        state.trackingVelocityTarget = commandedForwardVelocity;
        state.trackingFeedforward = 0.0;
        state.trackingVelocityFeedback = 0.0;
        state.trackingEndpointBlend = 0.0;
        state.velocitySampleAvailable = true;

        Angle headingTarget = path.getInterpolator().getHeadingTarg(
                distanceRemaining, firstDerivative, endTangent);
        // endregion

        // region 2. Algorithm
        // 2.1 Heading feedforward and feedback
        double targetAngularVelocity = 0.0;
        double headingFeedforward = 0.0;
        if (profiled) {
            double headingTimeScale = MotionFeedforward.timeScale(
                    targets.getTangentialVel(), progressVelocity);
            targetAngularVelocity = targets.getAngularVel() * headingTimeScale;
            double targetAngularAcceleration = targets.getAngularAccel()
                    * headingTimeScale * headingTimeScale;
            headingFeedforward = targetAngularVelocity * angularKV
                    + targetAngularAcceleration * angularKA;
            if (Math.abs(targetAngularVelocity) > 1e-6) {
                headingFeedforward += Math.signum(targetAngularVelocity)
                        * constants.angularFeedforwardKS;
            }
        }

        double currentAngularVelocity = localizer.getVel().getHeading().getRad();
        state.trackingAngularVelocityTarget = targetAngularVelocity;
        double headingError = currentHeading.getShortestAngleTo(headingTarget).getRad();
        double headingTargetRate = profiled ? targetAngularVelocity
                : path.getInterpolator().getHeadingFirstDerivative(
                        distanceRemaining, curvature, endTangent) * progressVelocity;
        double headingFeedback = headingControllerEnabled
                ? Follower.calculatePathHeadingFeedback(
                        headingController, headingError, headingTargetRate,
                        currentAngularVelocity, applyCorrectiveStatic)
                : 0.0;
        double headingVelocityFeedback = headingControllerEnabled
                && Math.abs(targetAngularVelocity) > 1e-6
                ? angularVelocityFeedbackGain
                        * (targetAngularVelocity - currentAngularVelocity) : 0.0;
        double endpointHeadingError = currentHeading
                .getShortestAngleTo(path.getEndPose().getHeading()).getRad();
        if (distanceRemaining < Follower.PROFILED_ENDPOINT_CAPTURE_DISTANCE_INCHES) {
            headingFeedback = Follower.ensureAngularEndpointBreakawayPower(
                    headingFeedback, endpointHeadingError, currentAngularVelocity,
                    applyCorrectiveStatic ? constants.angularCoeffs.kS : 0.0,
                    PathCompletionPolicy.headingTolerance(headingTolerance));
        }

        // 2.2 Cross-track and centripetal corrections
        Vector positionalError = state.closestPathPoint.minus(currentPosition);
        state.crossTrackError = positionalError.dot(lateralNormal).getIn();
        state.centripetalError = positionalError.dot(principalNormal).getIn();
        double lateralFeedback = driveControllerEnabled
                ? driveController.calculateCrossTrack(
                        state.crossTrackError, applyCorrectiveStatic) : 0.0;
        if (driveControllerEnabled) {
            double measuredLateralVelocity = robotVelocity.dot(lateralNormal).getIn();
            lateralFeedback = Follower.ensureEndpointBreakawayPower(
                    lateralFeedback, state.crossTrackError, measuredLateralVelocity,
                    constants.translationalCoeffs.kS, PDSController.LINEAR_STATIC_DEADBAND,
                    distanceRemaining);
        }
        state.crossTrackCorrection = lateralNormal.times(lateralFeedback);
        state.centripetalCorrection = Follower.calculateCentripetalCorrection(
                principalNormal, robotTangentialVelocity, curvature, centripetalGain);

        // 2.3 Tangential feedforward and feedback
        double tangentFeedforward;
        double tangentFeedback = 0.0;
        double tangentVelocityFeedback = 0.0;
        if (state.t < 1.0) {
            if (profiled) {
                double velocityTarget = Math.max(
                        targets.getTangentialVel(), MIN_PROFILE_VELOCITY_IN_PER_SECOND);
                double accelerationTarget = MotionFeedforward.scaleBrakingAcceleration(
                        velocityTarget, targets.getTangentialAccel(), robotTangentialVelocity);
                double feedforward = MotionFeedforward.calculate(
                        velocityTarget, accelerationTarget, translationalKV, translationalKA,
                        constants.translationalFeedforwardKS);
                tangentVelocityFeedback =
                        (velocityTarget - robotTangentialVelocity) * velocityFeedbackGain;
                state.trackingVelocityTarget = velocityTarget;
                state.trackingFeedforward = feedforward;
                state.trackingVelocityFeedback = tangentVelocityFeedback;
                tangentFeedforward = feedforward;

                // Let endpoint braking override the boosted profile when needed.
                if (path.isAccelBoosted()) {
                    double decelPower = driveController.calculateEndDistance(
                            distanceRemaining, -measuredTangentialVelocity);
                    if (decelPower <= tangentFeedforward + tangentVelocityFeedback) {
                        tangentFeedback = decelPower;
                        tangentVelocityFeedback = 0.0;
                        tangentFeedforward = 0.0;
                    }
                }
            } else {
                // Quick paths choose between cruising power and endpoint braking.
                double endDistanceError = distanceRemaining
                        < Follower.PROFILED_ENDPOINT_CAPTURE_DISTANCE_INCHES
                        ? signedEndpointError : distanceRemaining;
                double decelPower = driveController.calculateEndDistance(
                        endDistanceError, -measuredTangentialVelocity);
                double velocityError = commandedForwardVelocity - robotTangentialVelocity;
                tangentVelocityFeedback = velocityError * velocityFeedbackGain;
                double propulsionStatic = Math.abs(robotTangentialVelocity)
                        < Follower.ENDPOINT_STALLED_VELOCITY_IN_PER_SECOND
                        ? Math.max(constants.translationalFeedforwardKS,
                                constants.translationalCoeffs.kS)
                        : constants.translationalFeedforwardKS;
                double feedforwardPower = commandedForwardVelocity * translationalKV
                        + Math.signum(commandedForwardVelocity) * propulsionStatic;
                double accelerationPower = feedforwardPower + tangentVelocityFeedback;
                if (decelPower <= accelerationPower) {
                    tangentFeedback = decelPower;
                    tangentVelocityFeedback = 0.0;
                    tangentFeedforward = 0.0;
                } else {
                    tangentFeedforward = feedforwardPower;
                }
            }
        } else {
            tangentFeedback = profiled ? 0.0 : driveController.calculateEndDistance(
                    signedEndpointError, -measuredTangentialVelocity);
            tangentFeedforward = 0.0;
        }

        // 2.4 Blend into endpoint capture
        if (profiled) {
            double endpointPower = driveController.calculateEndDistance(
                    signedEndpointError, -measuredTangentialVelocity);
            endpointPower = Follower.ensureEndpointBreakawayPower(
                    endpointPower, signedEndpointError, robotTangentialVelocity,
                    constants.translationalCoeffs.kS,
                    PathCompletionPolicy.distanceTolerance(distanceTolerance), distanceRemaining);
            double endpointBlend = Follower.endpointCaptureBlend(distanceRemaining);
            state.trackingEndpointBlend = endpointBlend;
            tangentFeedback = tangentFeedback * (1.0 - endpointBlend)
                    + endpointPower * endpointBlend;
            tangentVelocityFeedback *= 1.0 - endpointBlend;
            tangentFeedforward *= 1.0 - endpointBlend;
        }

        // 2.5 Reconcile propulsion and braking
        if (tangentFeedforward * tangentVelocityFeedback < 0.0) {
            double netTangentPower = tangentFeedforward + tangentVelocityFeedback;
            if (netTangentPower * tangentFeedforward >= 0.0) {
                tangentFeedforward = netTangentPower;
                tangentVelocityFeedback = 0.0;
            } else {
                tangentFeedforward = 0.0;
                tangentVelocityFeedback = netTangentPower;
            }
        }
        if (robotTangentialVelocity * tangentVelocityFeedback < 0.0) {
            tangentFeedback += tangentVelocityFeedback;
            tangentVelocityFeedback = 0.0;
        }

        // 2.6 Allocate available motor power
        HolonomicCommandAllocator.Result command = allocator.allocate(
                state.crossTrackCorrection, state.centripetalCorrection, unitTangent,
                currentHeading, tangentFeedback, headingFeedback, tangentFeedforward,
                tangentVelocityFeedback, headingVelocityFeedback, headingFeedforward);
        state.lastCommandDemand = command.demand;

        // 2.7 Completion and stuck detection
        double endpointDistance = currentPosition.distanceTo(path.getEndPose().getVec()).getIn();
        boolean insideTolerance = PathCompletionPolicy.isInsideTolerance(
                endpointDistance, endpointHeadingError, distanceTolerance, headingTolerance);
        boolean velocitySettled = PathCompletionPolicy.isVelocitySettled(
                robotVelocity.getMagSq().getIn(), currentAngularVelocity);
        if (state.completion.isComplete(insideTolerance, velocitySettled, System.nanoTime())) {
            return true;
        }
        // End the movement if the robot remains stuck.
        if (state.stuckWatchdog.update(
                Follower.stuckForSpeed(robotVelocity.getMag().getIn()), System.nanoTime())) {
            return true;
        }

        // endregion

        // region 3. Send powers
        drivetrain.drive(command.drive.getX().getIn(), command.drive.getY().getIn(), command.turn);
        // endregion

        return false;
    }
}
