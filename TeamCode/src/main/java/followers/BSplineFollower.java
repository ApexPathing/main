package followers;


import drivetrains.Drivetrain;
import followers.constants.BSplineFollowerConstants;
import localizers.Localizer;
import paths.Path;
import paths.PathSegment;
import paths.heading.HeadingInterpolator;
import util.Angle;
import util.Pose;
import util.Vector;

/**
 * BSplineFollower class, capable of following paths made with PathBuilder
 * Important: Ensure your BSplineFollower constants are fully configured
 * before attempting to use this follower {@link BSplineFollowerConstants}
 * @author Sohum Arora 22985 Paraducks
 */
public class BSplineFollower extends Follower {
    private static final double pi2 = 2 * Math.PI;
    private final BSplineFollowerConstants constants;
    private Path path;
    private long holdStartTimeNs = 0;
    private boolean holdTimerInitialized = false;
    private long pauseStartNs = 0;
    private boolean wasHoldingPosePrevFrame = false;
    /**
     * BSplineFollower constructor
     * @param constants - Your BSplineFollowerConstants (ensure configured)
     */
    public BSplineFollower(BSplineFollowerConstants constants, Drivetrain drivetrain, Localizer localizer) {
        super(drivetrain, localizer);
        this.constants = constants;
    }

    /**
     * Sets the path to be followed
     * @param path is the path to be followed
     */
    public void followPath(Path path) {
        this.path = path;
        this.path.reset();
        this.isBusy = true;
        this.holdingPose = false;
        this.holdTimerInitialized = false;
        this.wasHoldingPosePrevFrame = false;
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

        if (!isBusy || path == null) {
            drivetrain.stop();
            return;
        }

        Pose current = getPose();
        Path.PathNode currentNode = path.getCurrentNode();

        //Turn logic
        if (currentNode.type == Path.NodeType.TURN) {
            double targetHeading = currentNode.targetHeading.getRad();
            double currentHeading = current.getHeading();
            double headingError = getShortestAngularDistance(currentHeading, targetHeading);

            if (Math.abs(headingError) < constants.headingTolerance) {
                if (path.isLastSegment()) {
                    this.isBusy = false;
                    this.breakFollowing(drivetrain);
                } else {
                    path.advance();
                }
                return;
            }

            double turnPower = headingError * constants.headingP;
            drive(0, 0, turnPower, currentHeading);
        } else if (currentNode.type == Path.NodeType.HOLD) {
            if (!holdTimerInitialized) {
                holdStartTimeNs = System.nanoTime();
                holdTimerInitialized = true;
            }

            long elapsedNs = System.nanoTime() - holdStartTimeNs;
            long totalDurationNs = (long) (currentNode.holdDurationSeconds * 1e9);

            if (elapsedNs >= totalDurationNs) {
                holdTimerInitialized = false;
                if (path.isLastSegment()) {
                    this.isBusy = false;
                    this.breakFollowing(drivetrain);
                } else {
                    path.advance();
                }
                return;
            }

            Pose lockPose = currentNode.holdPose;
            Vector error = lockPose.toVec().subtract(current.toVec());
            Vector feedback = error.multiply(constants.translationP);

            double headingError = getShortestAngularDistance(current.getHeading(), lockPose.getHeading());
            double turnPower = headingError * constants.headingP;

            drive(feedback.getX(), feedback.getY(), turnPower, current.getHeading());
        } else if (currentNode.type == Path.NodeType.DRIVE) {
            PathSegment segment = currentNode.segment;
            HeadingInterpolator interpolator = currentNode.interpolator;

            if (segment == null || interpolator == null) { //null check
                stop();
                return;
            }

            double t = segment.getBestT(current.toVec());

            Vector targetPoseVec = segment.getPosition(t);
            Vector targetVel = segment.getFirstDerivative(t);

            Vector error = targetPoseVec.subtract(current.toVec());
            Vector feedback = error.multiply(constants.translationP);

            Vector feedforward = targetVel.multiply(constants.velocityFF);
            Vector drivePower = feedback.add(feedforward);

            double driveX = drivePower.getX();
            double driveY = drivePower.getY();

            Angle targetAngle = interpolator.getHeading(t, targetVel);
            double targetHeading = targetAngle.getRad();
            double currentHeading = current.getHeading();

            double headingError = getShortestAngularDistance(currentHeading, targetHeading);
            double turnPower = headingError * constants.headingP;

            double distance = segment.getDistanceToEnd_in(targetPoseVec, t);
            if (t >= constants.tTolerance && distance < constants.distanceTolerance) {
                if (path.isLastSegment()) {
                    Vector finalPosition = segment.getPosition(1.0);
                    this.setTargetPose(new Pose(finalPosition.getX(), finalPosition.getY(), targetHeading));
                    this.holdingPose = true;
                    this.isBusy = false;
                    this.breakFollowing(drivetrain);
                } else {
                    path.advance();
                }
                return;
            }

            drive(driveX, driveY, turnPower, currentHeading);
        }
    }

    /**
     * Logic to hold a specific Pose on the field
     */
    private void holdPose() {
        Pose currentPose = getPose();

        Vector error = targetPose.toVec().subtract(currentPose.toVec());
        double errorMag = error.getMagnitude();
        double headingError = getShortestAngularDistance(currentPose.getHeading(), targetPose.getHeading());

        if (errorMag < constants.distanceTolerance && Math.abs(headingError) < constants.headingTolerance) {
            drivetrain.stop();
            return;
        }

        double maxPower = 1.0;
        Vector feedback = errorMag > 0 ? error.normalize().multiply(Math.min(errorMag * constants.translationP, maxPower)) : new Vector(0, 0);

        double turnPower = Math.max(-maxPower, Math.min(maxPower, headingError * constants.headingP));

        drive(feedback.getX(), feedback.getY(), turnPower, currentPose.getHeading());
    }

    private double getShortestAngularDistance(double currentRad, double targetRad) {
        double diff = (targetRad - currentRad) % (pi2);
        if (diff > Math.PI) diff -= pi2;
        else if (diff < -Math.PI) diff += pi2;
        return diff;
    }

    @Override
    public void stop() {
        super.stop();
        this.holdTimerInitialized = false;
        this.wasHoldingPosePrevFrame = false;
    }
}
