package followers;

import com.qualcomm.robotcore.util.Range;

import controllers.PDFLController;
import controllers.PDSController;
import drivetrains.Drivetrain;
import followers.constants.BSplineFollowerConstants;
import localizers.Localizer;

import paths.movements.FollowerMovement;
import paths.movements.Path;
import paths.movements.Turn;
import geometry.PathSegment;
import paths.heading.HeadingInterpolator;

import geometry.Angle;
import geometry.Vector;
import geometry.Pose;

/**
 * MovementFollower class, capable of following FollowerMovements made with Builders
 * Important: Ensure your BSplineFollower constants are fully configured
 * before attempting to use this follower {@link BSplineFollowerConstants}
 * @author Sohum Arora 22985 Paraducks
 */
public class MovementFollower extends Follower {
    private static final double pi2 = 2 * Math.PI;
    private final BSplineFollowerConstants constants;

    // PDS Controllers for closed-loop feedback
    private final PDSController lateralController;
    private final PDFLController velocityController;
    private final PDSController driveController;
    private final PDSController headingController;

    // Architecture Change: Singular active movement. Throws exception if overridden prematurely.
    private FollowerMovement currentMovement = null;

    private long holdStartTimeNs = 0;
    private boolean holdTimerInitialized = false;
    private long pauseStartNs = 0;
    private boolean wasHoldingPosePrevFrame = false;

    /**
     * MovementFollower constructor
     * @param constants - Your BSplineFollowerConstants (ensure configured)
     */
    public MovementFollower(BSplineFollowerConstants constants, Drivetrain drivetrain, Localizer localizer) {
        super(drivetrain, localizer);
        this.constants = constants;

        // Initialize controllers with PDS coefficients from constants
        this.lateralController = new PDSController(constants.translationCoeffs);
        this.headingController = new PDSController(constants.headingCoeffs);

        // Mark heading controller as angular so it handles angle normalization
        this.headingController.setAngularController();
        this.velocityController = new PDFLController(new PDFLController.PDFLCoefficients());
        this.driveController = new PDSController(new PDSController.PDSCoefficients());
    }

    /**
     * Retrieves the active constants instance driving this follower.
     * Useful for live tuning via dashboards.
     */
    public BSplineFollowerConstants getConstants() { return this.constants; }

    /**
     * Sets the movement to be followed.
     * @param movement is the movement to be followed
     * @throws IllegalStateException if the follower is already busy executing a movement.
     */
    public void follow(FollowerMovement movement) {
        if (this.isBusy || this.currentMovement != null) {
            throw new IllegalStateException("Cannot follow a new movement while the follower is currently busy! Check !follower.isBusy() in your state machine before calling follow().");
        }

        // Standard call: Start executing immediately
        this.currentMovement = movement;
        this.isBusy = true;
        this.holdingPose = false;
        this.holdTimerInitialized = false;
        this.wasHoldingPosePrevFrame = false;

        // Reset controllers right before starting a new path to prevent derivative kick
        lateralController.reset();
        headingController.reset();
    }

    @Override
    public void update() {
        if (holdingPose && targetPose != null) {
            if (!wasHoldingPosePrevFrame) {
                pauseStartNs = System.nanoTime();
                wasHoldingPosePrevFrame = true;
            }
            holdPose();
            return;
        }

        if (wasHoldingPosePrevFrame) {
            long pauseDurationNs = System.nanoTime() - pauseStartNs;
            if (holdTimerInitialized && holdStartTimeNs > 0) {
                holdStartTimeNs += pauseDurationNs;
            }
            wasHoldingPosePrevFrame = false;
        }

        if (!isBusy || currentMovement == null) {
            drivetrain.stop();
            return;
        }

        Pose current = getPose();

        // Turn logic
        if (currentMovement instanceof Turn) {
            Turn turn = (Turn) currentMovement;

            double targetHeading = turn.getEndPose().getHeading().getRad();
            double currentHeading = current.getHeading().getRad();
            double headingError = getShortestAngularDistance(currentHeading, targetHeading);

            if (Math.abs(headingError) < constants.headingTolerance) {
                this.isBusy = false;
                this.currentMovement = null;
                this.breakFollowing();
                return;
            }

            Vector targetPoseVec = turn.getStartPose().getPos();
            Vector error = targetPoseVec.minus(current.getPos());

            double errorMag = error.getMag().getIn();
            double translationPower = lateralController.calculateFromError(errorMag);
            Vector feedback = errorMag > 0 ? error.normalize().times(translationPower) : Vector.zero();

            double turnPower = headingController.calculateFromError(headingError);

            // Pass the calculated feedback instead of 0, 0
            drive(feedback.getX().getIn(), feedback.getY().getIn(), turnPower, currentHeading);

            // region Curve movement

        } else if (currentMovement instanceof Path) {
            Path pathSegmentMove = (Path) currentMovement;
            PathSegment segment = pathSegmentMove.getParametricPath();
            HeadingInterpolator interpolator = pathSegmentMove.getInterpolator();

            if (segment == null || interpolator == null) {
                stop();
                return;
            }

            // 1. READS: Extract all state, geometry, and error values
            double t = segment.getBestT(current.getPos());
            double currentHeading = current.getHeading().getRad();

            Vector robotVelocity = localizer.getVelocity().getPos();
            Vector targetPoseVec = segment.getPosition(t);

            // Raw derivatives needed ONLY for calculating the radius of curvature
            Vector velVec = segment.getFirstDerivative(t);
            Vector accelVec = segment.getSecondDerivative(t);

            Vector normal = PathSegment.calculateArcNormal(velVec, accelVec);

            // The normalized tangent dictates pure geometric direction, independent of speed
            Vector unitTangent = velVec.normalize();

            double distanceRemaining = segment.getDistanceToEnd_in(targetPoseVec, t);
            double distanceTravelled = segment.getLength_in() - distanceRemaining;
            double pathProgression = distanceTravelled / segment.getLength_in();

            Angle targetAngle = interpolator.getHeading(pathProgression, velVec);
            double targetHeading = targetAngle.getRad();

            Vector fieldError = targetPoseVec.minus(current.getPos());
            double headingError = getShortestAngularDistance(currentHeading, targetHeading);

            // 2. CALCULATIONS: Apply physics, scaling, and PID math
            double availableMotorPower = 1.0;

            // --- 1. Turn Power Allocation (Highest Priority) ---
            double turnPower = headingController.calculateFromError(headingError);
            availableMotorPower -= Math.abs(turnPower);

            // --- 2. Combined Lateral Power (Centripetal + Feedback) ---
            double radius = PathSegment.calculateRadiusOfCurvature(velVec, accelVec);
            double robotVelMag = robotVelocity.getMag().getIn();

            // Required inward acceleration based on ACTUAL current speed
            double requiredLateralAccel = (robotVelMag * robotVelMag) / radius;
            double centripetalMag = requiredLateralAccel / constants.getMaxLateralAccel();

            // Project field error onto the normal vector to isolate lateral drift
            double crossTrackError = fieldError.dot(normal).getIn();
            double lateralFeedbackMag = lateralController.calculateFromError(crossTrackError);

            // Mathematically combine centripetal feedforward and corrective feedback
            double netLateralMag = centripetalMag + lateralFeedbackMag;
            netLateralMag = Range.clip(netLateralMag, -availableMotorPower, availableMotorPower);

            Vector lateralDriveVec = normal.times(netLateralMag);
            availableMotorPower -= Math.abs(netLateralMag);

            // --- 3. Tangent/Along-Track Power Allocation ---

            // TODO: Implement properly with velocity limit in constants!
            double desiredVelocity = 10.0;

            // Cap the desired speed if the upcoming curve is too sharp
            if (radius != Double.POSITIVE_INFINITY) {
                double maxSafeVelocity = Math.sqrt(constants.getMaxLateralAccel() * radius);
                if (desiredVelocity > maxSafeVelocity) {
                    desiredVelocity = maxSafeVelocity;
                }
            }

            // Project field error onto the tangent vector to isolate forward/backward error
            double alongTrackError = fieldError.dot(unitTangent).getIn();

            // Calculate deceleration if current speed exceeds the safe requested speed
            double neededAccel = 0.0;
            if (desiredVelocity < robotVelMag) {
                neededAccel = desiredVelocity - robotVelMag; // Yields a negative value for braking
            }

            // Feedforward: kV scales the target speed, kA (with negative accel) pulls power back to brake
            double feedforward = (constants.kV * desiredVelocity) + (constants.kA * neededAccel);

            double tangentFeedbackMag;
            if (t < 1.0) {
                tangentFeedbackMag = driveController.calculateFromError(alongTrackError);
            } else {
                // Infinite line fallback for end of path
                Vector endPos = segment.getPosition(1.0);
                Vector endTangent = segment.getFirstDerivative(1.0).normalize();
                Vector endToRobot = current.getPos().minus(endPos);

                double distancePastEnd = endToRobot.dot(endTangent).getIn();
                tangentFeedbackMag = driveController.calculateFromError(-distancePastEnd);
            }

            // Calculate the velocity feedback and safely append the Feedforward and kS
            double velocityFeedback = velocityController.calculateFromError(desiredVelocity - robotVelMag)
                    + feedforward
                    + driveController.getCoefficients().kS;

            double positionFeedback = tangentFeedbackMag;

            // Cap the spatial position request with the velocity ceiling
            double totalTangentPower = Math.min(velocityFeedback, positionFeedback);
            totalTangentPower = Range.clip(totalTangentPower, -availableMotorPower, availableMotorPower);

            // Apply the power scalar to the purely directional unit tangent
            Vector tangentDriveVec = unitTangent.times(totalTangentPower);

            // --- Final Blending ---
            Vector driveOutput = tangentDriveVec.plus(lateralDriveVec);

            double driveX = driveOutput.getX().getIn();
            double driveY = driveOutput.getY().getIn();

            // 3. WRITES: Actuate motors or advance state machine
            if (t >= constants.tTolerance && distanceRemaining < constants.distanceTolerance) {
                Vector finalPosition = currentMovement.getEndPose().getPos();
                this.setTargetPose(new Pose(finalPosition, Angle.fromRad(targetHeading)));
                this.holdingPose = true;
                this.isBusy = false;
                this.currentMovement = null;
                this.breakFollowing();
                return;
            }

            drive(driveX, driveY, turnPower, currentHeading);
        }
    }

    private void holdPose() {
        Pose currentPose = getPose();

        Vector error = targetPose.getPos().minus(currentPose.getPos());
        double errorMag = error.getMag().getIn();
        double headingError = getShortestAngularDistance(currentPose.getHeading().getRad(), targetPose.getHeading().getRad());

        if (errorMag < constants.distanceTolerance && Math.abs(headingError) < constants.headingTolerance) {
            drivetrain.stop();
            return;
        }
        double translationPower = lateralController.calculateFromError(errorMag);
        Vector feedback = errorMag > 0 ? error.normalize().times(translationPower) : Vector.zero();

        double turnPower = headingController.calculateFromError(headingError);

        drive(feedback.getX().getIn(), feedback.getY().getIn(), turnPower, currentPose.getHeading().getRad());
    }

    private double getShortestAngularDistance(double currentRad, double targetRad) {
        double diff = (targetRad - currentRad) % (pi2);
        if (diff > Math.PI) diff -= pi2;
        else if (diff < -Math.PI) diff += pi2;
        return diff;
    }

    private double calculateNewAvailablePower(double availablePower, double newRequest) {
        return Range.clip(availablePower - Math.abs(newRequest), 0.0, 1.0);
    }

    @Override
    public void stop() {
        super.stop();
        this.holdTimerInitialized = false;
        this.wasHoldingPosePrevFrame = false;
        this.currentMovement = null;
    }
}