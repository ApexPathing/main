package core;

import controllers.DriveController;
import controllers.DriveController.AllocatedCommand;
import controllers.TurnController;
import drivetrains.BaseDrivetrain;
import feedforward.MotionParameters;
import geometry.Angle;
import geometry.Pose;
import geometry.Vector;
import localizers.BaseLocalizer;
import paths.movements.Turn;

/** Executes point turns, including profiled turns and optional translation hold. */
final class TurnExecutor {
    private static final double SETTLED_ANGULAR_VELOCITY = 0.10;

    private final BaseDrivetrain<?> drivetrain;
    private final DriveController driveController;
    private final TurnController turnController;
    private final HolonomicCommandAllocator allocator;
    private final FollowerConstants constants;
    private final double headingTolerance;
    private final double distanceTolerance;

    private Angle targetHeading;
    private Vector targetPosition;
    private double direction;
    private double totalDisplacement;
    private double elapsedSeconds;
    private long lastUpdateNanos;

    TurnExecutor(BaseDrivetrain<?> drivetrain, DriveController driveController,
                 TurnController turnController, HolonomicCommandAllocator allocator,
                 FollowerConstants constants, double headingTolerance, double distanceTolerance) {
        this.drivetrain = drivetrain;
        this.driveController = driveController;
        this.turnController = turnController;
        this.allocator = allocator;
        this.constants = constants;
        this.headingTolerance = headingTolerance;
        this.distanceTolerance = distanceTolerance;
    }

    void start(Turn turn, long nowNanos) {
        targetHeading = turn.getEndPose().getHeading();
        targetPosition = turn.getStartPose().getVec();
        double signedTurn = turn.getStartPose().getHeading()
                .getShortestAngleTo(turn.getEndPose().getHeading()).getRad();
        direction = Math.signum(signedTurn);
        totalDisplacement = Math.abs(signedTurn);
        elapsedSeconds = 0.0;
        lastUpdateNanos = nowNanos;
    }

    boolean update(Turn turn, Pose current, BaseLocalizer<?> localizer,
                   MovementCallbackRunner callbacks, FollowerPathState state,
                   boolean driveControllerEnabled) {
        Angle currentHeading = current.getHeading();
        double headingError = currentHeading.getShortestAngleTo(targetHeading).getRad();
        long now = System.nanoTime();
        if (lastUpdateNanos != 0L) {
            elapsedSeconds += Math.max(0.0, (now - lastUpdateNanos) * 1e-9);
        }
        lastUpdateNanos = now;
        callbacks.process(turn, -1.0, currentHeading);

        double currentAngularVelocity = localizer.getVel().getHeading().getRad();
        double effectiveHeadingTolerance = PathCompletionPolicy.headingTolerance(headingTolerance);
        boolean profileComplete = turn.getFeedforwardLut() == null
                || elapsedSeconds >= turn.getFeedforwardLut().getDurationSeconds();
        if (profileComplete && Math.abs(headingError) < effectiveHeadingTolerance
                && Math.abs(currentAngularVelocity) < SETTLED_ANGULAR_VELOCITY) {
            return true;
        }

        double turnPower;
        if (turn.getFeedforwardLut() == null || totalDisplacement < 1e-9) {
            turnPower = turnController.calculateQuick(
                    headingError, currentAngularVelocity, effectiveHeadingTolerance);
        } else {
            MotionParameters targets = turn.getFeedforwardLut().getFFParamsByTime(elapsedSeconds);
            state.trackingAngularVelocityTarget = targets.getAngularVel();
            Angle profileHeading = turn.getStartPose().getHeading().plus(
                    Angle.fromRad(direction * targets.getDistAlongCurve()));
            double profileHeadingError = currentHeading.getShortestAngleTo(profileHeading).getRad();
            turnPower = turnController.calculateProfiled(
                    profileHeadingError, direction, targets, currentAngularVelocity);
        }

        turnPower = Follower.ensureAngularEndpointBreakawayPower(
                turnPower, headingError, currentAngularVelocity,
                constants.angularCoeffs.kS, effectiveHeadingTolerance);

        Vector positionError = targetPosition.minus(current.getVec());
        if (driveControllerEnabled && drivetrain.isHolonomic()
                && positionError.getMag().getIn() > distanceTolerance) {
            Vector fieldFeedback = driveController.calculatePointToPoint(
                    targetPosition, current.getVec());
            AllocatedCommand positionHold = allocator.allocatePositionHold(
                    fieldFeedback, currentHeading, 1.0 - Math.abs(turnPower));
            Vector robotCommand = positionHold.getRobotCommand();
            drivetrain.drive(robotCommand.getX().getIn(), robotCommand.getY().getIn(), turnPower);
        } else {
            drivetrain.drive(0.0, 0.0, turnPower);
        }
        return false;
    }

    void pause() {
        lastUpdateNanos = 0L;
    }

    void resume(long nowNanos) {
        lastUpdateNanos = nowNanos;
    }

    void markTransitionTick(long nowNanos) {
        lastUpdateNanos = nowNanos;
    }

    void reset() {
        targetHeading = null;
        targetPosition = null;
        direction = 0.0;
        totalDisplacement = 0.0;
        elapsedSeconds = 0.0;
        lastUpdateNanos = 0L;
    }
}
