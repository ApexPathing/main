package core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import controllers.DriveController;
import controllers.DriveController.AllocatedCommand;
import controllers.TurnController;
import controllers.PDSController.PDSCoefficients;
import controllers.PDSController;
import drivetrains.BaseDrivetrain;
import drivetrains.BaseDrivetrainConstants;
import drivetrains.DualActuated;
import drivetrains.Mecanum;
import feedforward.MotionParameters;
import geometry.Angle;
import geometry.AngleUnit;
import geometry.Dist;
import geometry.DistUnit;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import localizers.BaseLocalizer;
import paths.Callback;
import paths.movements.FollowerMovement;
import paths.movements.Path;
import paths.movements.Turn;

/**
 * Apex Pathing's main Follower class. Handles the execution of generated paths and turns using
 * kinematic feedforward and feedback controllers.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author DrPixelCat - 7842 alum
 * @author Dylan B. - 18597 RoboClovers - Delta
 * @author Xander Haemel - 31616 404 Not Found
 */
public class Follower {
    private static final FollowerDiagnostics NO_DIAGNOSTICS = new FollowerDiagnostics() {
        @Override
        public void recordPose(Pose pose) { }

        @Override
        public void recordCurrentPath(Path path) { }

        @Override
        public void clearCurrentPath() { }
    };
    private static volatile FollowerDiagnostics diagnostics = NO_DIAGNOSTICS;

    private static final double PROFILED_ENDPOINT_CAPTURE_DISTANCE_INCHES = 4.0;
    private static final double ENDPOINT_STALLED_VELOCITY_IN_PER_SECOND = 0.25;
    private static final double PROFILED_HEADING_CAPTURE_RADIANS = Math.toRadians(10.0);
    private static final double SETTLED_LINEAR_VELOCITY_SQ = 64.0;
    private static final double SETTLED_ANGULAR_VELOCITY = 0.25;
    private static final double COMPLETION_DWELL_SECONDS = 0.10;
    private static final double MAX_CENTRIPETAL_POWER = 0.65;
    private static final double PROGRESS_VELOCITY_FILTER_SECONDS = 0.08;
    private static final double MAX_PROGRESS_SAMPLE_SECONDS = 0.10;
    private static final double MIN_COMPLETION_DISTANCE_INCHES = 1.0;
    private static final double MIN_COMPLETION_HEADING_RADIANS = Math.toRadians(2.0);
    private final FollowerConstants constants;
    private final BaseDrivetrain<?> drivetrain;
    private final BaseLocalizer<?> localizer;

    private enum HolonomicDriveModel { ANISOTROPIC, ISOTROPIC }

    private final double headingTol; // Radians
    private final double distanceTol; // Inches

    private Angle lastHeading; // Tracks heading between ticks for angular callback sweeps
    private Pose lastPose;

    private final PDSController headingController;
    private final TurnController turnController;
    private final DriveController driveController;
    private double translationalKV;
    private double translationalKA;
    private double angularKV;
    private double angularKA;
    private double centripetalGain;
    private double velocityFeedbackGain;
    private double angularVelocityFeedbackGain;

    private FollowerMovement currentMovement ;
    private boolean paused = false;

    private boolean headingControllerEnabled = true;
    private boolean driveControllerEnabled = true;

    private PathSegment segment;
    private Angle targetHeading;
    private Vector targetTurnPoseVec;
    private double turnDirection;
    private double turnTotalDisplacement;
    private double crossTrackError;
    private double centripetalError;
    private double t;
    private Vector closestPathPoint = Vector.zero();
    private Vector crossTrackNormal = Vector.zero();
    private Vector pathNormal = Vector.zero();
    private Vector crossTrackCorrection = Vector.zero();
    private Vector centripetalCorrection = Vector.zero();
    private CommandDemand lastCommandDemand = CommandDemand.ZERO;
    private long completionToleranceEnteredNanos;
    private double previousPathDistanceIn = Double.NaN;
    private double pathProgressVelocityIn;
    private long previousPathProgressNanos;

    /** Raw controller demand from the most recent holonomic update, before normalization. */
    public static final class CommandDemand {
        private static final CommandDemand ZERO =
                new CommandDemand(false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        public final boolean available;
        public final double crossTrack;
        public final double tangentCorrection;
        public final double headingCorrection;
        public final double centripetal;
        public final double forwardVelocity;
        public final double headingVelocity;
        public final double driveFeedforward;
        public final double headingFeedforward;
        public final double correctiveTotal;
        public final double velocityTotal;
        public final double feedforwardTotal;
        public final double total;

        private CommandDemand(boolean available, double crossTrack, double tangentCorrection,
                              double headingCorrection, double centripetal,
                              double forwardVelocity, double headingVelocity,
                              double driveFeedforward, double headingFeedforward,
                              double correctiveTotal, double velocityTotal,
                              double feedforwardTotal, double total) {
            this.available = available;
            this.crossTrack = crossTrack;
            this.tangentCorrection = tangentCorrection;
            this.headingCorrection = headingCorrection;
            this.centripetal = centripetal;
            this.forwardVelocity = forwardVelocity;
            this.headingVelocity = headingVelocity;
            this.driveFeedforward = driveFeedforward;
            this.headingFeedforward = headingFeedforward;
            this.correctiveTotal = correctiveTotal;
            this.velocityTotal = velocityTotal;
            this.feedforwardTotal = feedforwardTotal;
            this.total = total;
        }
    }

    /** Constructs the drivetrain, localizer, and follower from the given {@link ApexConstants}. */
    public Follower(ApexConstants constants, HardwareMap hardwareMap) {
        this(constants, hardwareMap, false);
    }

    public Follower(ApexConstants constants, HardwareMap hardwareMap, boolean tuningMode) {
        BaseDrivetrainConstants<?> drivetrainConstants = constants.drivetrainConstants();

        this.drivetrain = drivetrainConstants.build(hardwareMap);
        this.localizer = constants.localizerConstants().build(hardwareMap);
        this.constants = FollowerConstants.getInstance();

        this.headingTol = drivetrainConstants.headingTolerance.getRad();
        this.distanceTol = drivetrainConstants.distanceTolerance.getIn();

        this.translationalKV = this.constants.translationalKV;
        this.translationalKA = this.constants.translationalKA;
        this.angularKV = this.constants.angularKV;
        this.angularKA = this.constants.angularKA;
        this.centripetalGain = this.constants.kCentripetal;
        this.velocityFeedbackGain = this.constants.velocityFeedbackGain;
        this.angularVelocityFeedbackGain = this.constants.angularVelocityFeedbackGain;

        this.headingController = new PDSController(this.constants.angularCoeffs);
        this.headingController.setAngularController();

        this.turnController = new TurnController(
                this.constants.angularCoeffs, angularKV, angularKA,
                this.constants.angularFeedforwardKS, angularVelocityFeedbackGain
        );
        this.driveController = new DriveController(
                Dist.fromIn(this.constants.forwardVelLimitIn),
                Dist.fromIn(this.constants.strafeVelLimitIn),
                this.constants.translationalCoeffs,
                !tuningMode && (drivetrain instanceof Mecanum || drivetrain instanceof DualActuated)
        );
    }

    // region Private methods

    /**
     * Evaluates all callbacks attached to the current movement and executes them if their
     * conditions are met.
     *
     * @param s The current geometric progression percentage [0.0, 1.0]. Pass -1.0 for turns.
     * @param currentHeading The robot's current field orientation.
     */
    private void processCallbacks(double s, Angle currentHeading) {
        Callback[] callbacks = null;
        if (currentMovement instanceof Path) {
            callbacks = ((Path) currentMovement).getCallbacks();
        } else if (currentMovement instanceof Turn) {
            callbacks = ((Turn) currentMovement).getCallbacks();
        }

        if (callbacks != null) {
            for (Callback cb : callbacks) {
                if (cb.isTriggered()) { continue; }

                boolean shouldTrigger = false;

                if (cb.getType() == Callback.CallbackType.DISTANCE) {
                    if (s >= cb.getS() && s >= 0.0) {
                        shouldTrigger = true;
                    }
                } else if (cb.getType() == Callback.CallbackType.ANGLE) {
                    double error =
                            Math.abs(currentHeading.getShortestAngleTo(cb.getTheta()).getRad());

                    // Trigger if resting within 1 degree of target
                    if (error < Math.toRadians(1.0)) {
                        shouldTrigger = true;
                    }
                    // Trigger if the target angle was swept past between the last tick and this
                    // tick
                    else if (lastHeading != null) {
                        double tickSweep = lastHeading.getShortestAngleTo(currentHeading).getRad();
                        double targetSweep = lastHeading.getShortestAngleTo(cb.getTheta()).getRad();

                        // If sweeps are in the same direction AND the tick sweep is larger, it
                        // was crossed
                        if (Math.signum(tickSweep) == Math.signum(targetSweep) &&
                                Math.abs(targetSweep) <= Math.abs(tickSweep)) {
                            shouldTrigger = true;
                        }
                    }
                }

                if (shouldTrigger) {
                    cb.getAction().run();
                    cb.setTriggered(true);
                }
            }
        }
        lastHeading = currentHeading;
    }

    private HolonomicDriveModel getActiveHolonomicDriveModel() {
        if (drivetrain instanceof Mecanum) { return HolonomicDriveModel.ANISOTROPIC; }
        if (drivetrain instanceof DualActuated) {
            if (!drivetrain.isHolonomic()) {
                throw new IllegalStateException(
                        "Dual-actuated drivetrain is not in its holonomic state."
                );
            }
            return HolonomicDriveModel.ANISOTROPIC;
        }
        if (!drivetrain.isHolonomic()) {
            throw new IllegalStateException(
                    "A holonomic allocation was requested while the drivetrain was non-holonomic."
            );
        }
        return HolonomicDriveModel.ISOTROPIC;
    }

    private AllocatedCommand allocateHolonomicStage(Vector fieldCommand, Angle currentHeading,
                                                    double availablePower,
                                                    HolonomicDriveModel driveModel) {
        if (driveModel == HolonomicDriveModel.ANISOTROPIC) {
            return driveController.allocateMecanum(
                    fieldCommand, currentHeading, availablePower);
        }
        return driveController.allocateIsotropic(fieldCommand, currentHeading, availablePower);
    }

    /** Returns the divisor that normalizes translation and turn as one command. */
    static double commandNormalizationScale(double x, double y, double turn,
                                            boolean anisotropic) {
        double translation = anisotropic ? Math.abs(x) + Math.abs(y) : Math.hypot(x, y);
        return Math.max(1.0, translation + Math.abs(turn));
    }

    /** Returns one shared scale for every component in a lower-priority stage. */
    private static double remainingScale(double demand, double available) {
        return demand > available && demand > 1e-12 ? available / demand : 1.0;
    }

    /** Accepts a settled endpoint immediately or a continuously accurate pose after a short dwell. */
    private boolean completionReady(boolean insideTolerance, boolean velocitySettled) {
        if (!insideTolerance) {
            completionToleranceEnteredNanos = 0L;
            return false;
        }
        if (velocitySettled) { return true; }
        long now = System.nanoTime();
        if (completionToleranceEnteredNanos == 0L) {
            completionToleranceEnteredNanos = now;
            return false;
        }
        return (now - completionToleranceEnteredNanos) * 1e-9 >= COMPLETION_DWELL_SECONDS;
    }

    /** Estimates useful along-path speed while rejecting discontinuous timing samples. */
    private double updatePathProgressVelocity(double distanceTraveled,
                                              double measuredTangentialVelocity) {
        long now = System.nanoTime();
        if (!Double.isFinite(previousPathDistanceIn) || previousPathProgressNanos == 0L) {
            previousPathDistanceIn = distanceTraveled;
            previousPathProgressNanos = now;
            pathProgressVelocityIn = measuredTangentialVelocity;
            return pathProgressVelocityIn;
        }
        double dt = (now - previousPathProgressNanos) * 1e-9;
        double distanceDelta = distanceTraveled - previousPathDistanceIn;
        previousPathDistanceIn = distanceTraveled;
        previousPathProgressNanos = now;
        if (dt <= 1e-4 || dt > MAX_PROGRESS_SAMPLE_SECONDS || distanceDelta < 0.0) {
            return pathProgressVelocityIn;
        }
        double sample = distanceDelta / dt;
        double alpha = dt / (PROGRESS_VELOCITY_FILTER_SECONDS + dt);
        pathProgressVelocityIn += alpha * (sample - pathProgressVelocityIn);
        return pathProgressVelocityIn;
    }

    // endregion
    // Public methods

    /**
     * The main execution loop of the follower.
     * Must be called continuously during the active OpMode loop to drive the robot along the path.
     */
    public void update() {
        update(false);
    }

    /**
     * The main execution loop of the follower.
     * Must be called continuously during the active OpMode loop to drive the robot along the path.
     *
     * @param holdPose whether to hold the most recently commanded end pose when idle
     */
    public void update(boolean holdPose) {
        localizer.update();
        diagnostics.recordPose(localizer.getPose());

        // Exit early if nothing is running or if paused.
        if (currentMovement == null || paused) {
            if (holdPose && lastPose != null) {
                double angularResponse = headingController.calculate(
                        lastPose.getHeading(AngleUnit.RAD) - getPose().getHeading(AngleUnit.RAD)
                );
                Vector translationalResponse;
                if (drivetrain.isHolonomic()) {
                    translationalResponse = driveController.calculatePointToPoint(
                            lastPose.getVec(), getPose().getVec()
                    );
                } else {
                    Vector globalError = lastPose.getVec().minus(getPose().getVec());
                    Vector localError =
                            globalError.rotate(lastPose.getHeading().times(-1.0));
                    double forwardError = localError.getX(DistUnit.IN);
                    translationalResponse = new Vector(
                            Dist.fromIn(driveController.calculateEndDistance(forwardError)),
                            Dist.zero()
                    );
                }
                drivetrain.drive(
                        translationalResponse.getX().getIn(),
                        translationalResponse.getY().getIn(),
                        angularResponse
                );
            }
            return;
        }

        Pose current = getPose();
        Vector currentPos = current.getVec();
        Angle currentHeading = current.getHeading();
        lastCommandDemand = CommandDemand.ZERO;

        // region Turn Execution
        if (currentMovement instanceof Turn) {
            Turn turn = (Turn) currentMovement;
            double headingError = currentHeading.getShortestAngleTo(targetHeading).getRad();

            // Process angular callbacks (-1 s value is used to indicate a turn)
            processCallbacks(-1.0, currentHeading);

            // Require both positional accuracy and low angular velocity to prevent momentum
            // overshoot
            double currentAngularVel = localizer.getVel().getHeading().getRad();
            boolean turnInsideTolerance = Math.abs(headingError) < Math.max(
                    headingTol, MIN_COMPLETION_HEADING_RADIANS);
            if (completionReady(turnInsideTolerance,
                    Math.abs(currentAngularVel) < SETTLED_ANGULAR_VELOCITY)) {
                this.stop();
                return;
            }

            double totalTurnPower;
            if (turn.getFeedforwardLut() == null || turnTotalDisplacement < 1e-9) {
                totalTurnPower = turnController.calculateQuick(
                        headingError, currentAngularVel,
                        Math.max(headingTol, MIN_COMPLETION_HEADING_RADIANS));
            } else {
                double signedTravel = turn.getStartPose().getHeading()
                        .getShortestAngleTo(currentHeading).getRad() * turnDirection;
                double angularDisplacement = Range.clip(
                        signedTravel, 0.0, turnTotalDisplacement);
                MotionParameters turnTargets = turn.getFeedforwardLut()
                        .getFFParams(angularDisplacement);
                totalTurnPower = turnController.calculateProfiled(
                        headingError,
                        turnDirection,
                        turnTargets,
                        currentAngularVel
                );
            }
            // Quick and profiled turns can both settle just outside tolerance after their
            // command falls below drivetrain breakaway power.
            totalTurnPower = ensureAngularEndpointBreakawayPower(
                    totalTurnPower,
                    headingError,
                    currentAngularVel,
                    constants.angularCoeffs.kS,
                    Math.max(headingTol, MIN_COMPLETION_HEADING_RADIANS)
            );

            Vector error = targetTurnPoseVec.minus(currentPos);
            double errorMag = error.getMag().getIn();

            // Hold xy only when translational control is explicitly enabled. Tuning phases can
            // disable it to guarantee that a Turn produces no x/y drivetrain command.
            if (driveControllerEnabled && drivetrain.isHolonomic() && errorMag > distanceTol) {
                Vector fieldFeedback = driveController.calculatePointToPoint(
                        targetTurnPoseVec, currentPos);
                AllocatedCommand positionHold = allocateHolonomicStage(
                        fieldFeedback,
                        currentHeading,
                        1.0 - Math.abs(totalTurnPower),
                        getActiveHolonomicDriveModel()
                );
                Vector robotCommand = positionHold.getRobotCommand();
                drivetrain.drive(robotCommand.getX().getIn(),
                        robotCommand.getY().getIn(), totalTurnPower);
            } else {
                drivetrain.drive(0, 0, totalTurnPower);
            }

        } else if (segment == null) {
            this.stop();
            // region Holonomic Following
        } else if (drivetrain.isHolonomic()) {
            // Retrieve path geometry at closest point
            t = segment.getBestT(currentPos);

            Vector targetPoseVec = segment.getPosition(t);
            closestPathPoint = targetPoseVec;
            double s = segment.getDistanceToEndIn(targetPoseVec, t);
            Vector velVec = segment.getFirstDerivative(t);
            Vector accelVec = segment.getSecondDerivative(t);
            Vector unitTangent = velVec.normalize();
            Vector lateralNormal = PathSegment.calculateLeftNormal(velVec);
            Vector normal = PathSegment.calculateArcNormal(velVec, accelVec);
            crossTrackNormal = lateralNormal;
            pathNormal = normal;
            Path path = (Path) currentMovement;
            Vector endTangent = segment.getFirstDerivative(1.0).normalize();
            double signedEndpointError = pathEndpointTangentError(
                    path.getEndPose().getVec(), currentPos, endTangent);

// Process scheduled distance and angular callbacks
            double pathProgress = 1.0 - s / segment.getLengthIn();
            processCallbacks(Range.clip(pathProgress, 0.0, 1.0), currentHeading);

            Vector robotVel = localizer.getVel().getVec();
            double distanceRemaining = segment.getDistanceToEndIn(targetPoseVec, t);
            double kappa = segment.getSignedCurvature(t);
            double dKappa = segment.getCurvatureDerivative(t);

            boolean isProfiled = path.isProfiled();
            double distanceTraveled = path.getParametricPath().getLengthIn() - s;
            MotionParameters targets = isProfiled ?
                    path.getFeedforwardLut().getFFParams(distanceTraveled) : null;
            double quickProgress = Range.clip(
                    1.0 - s / path.getParametricPath().getLengthIn(), 0.0, 1.0);
            double commandedForwardVelocity = isProfiled
                    ? targets.getTangentialVel()
                    : path.getQuickVelocityLimit(quickProgress, constants.forwardVelLimitIn);
            boolean applyCorrectiveStatic = Math.abs(commandedForwardVelocity) <
                    0.10 * constants.forwardVelLimitIn;

            HolonomicDriveModel driveModel = getActiveHolonomicDriveModel();

// Localizers report field-axis velocity. Project it directly onto the path tangent;
            double measuredTangentialVelocity = robotVel.dot(unitTangent).getIn();
            double robotTangentialVel = updatePathProgressVelocity(
                    distanceTraveled, measuredTangentialVelocity);

// Calculate heading power allocation
            Angle headingTarg = path.getInterpolator().getHeadingTarg(s, velVec, endTangent);
            double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, endTangent);
            double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                    endTangent);

            double omegaTarget = 0.0;
            double headingFF = 0.0;
            if (isProfiled) {
                omegaTarget = fPrime * targets.getTangentialVel();
                double alphaTarget = fDoublePrime *
                        (targets.getTangentialVel() * targets.getTangentialVel()) +
                        fPrime * targets.getTangentialAccel();

                headingFF = omegaTarget * angularKV + alphaTarget * angularKA;
                if (Math.abs(omegaTarget) > 1e-6) {
                    headingFF += Math.signum(omegaTarget) * constants.angularFeedforwardKS;
                }
            }

            double currentAngularVelocity = localizer.getVel().getHeading().getRad();
            double headingError = currentHeading.getShortestAngleTo(headingTarg).getRad();
            double headingFeedback = headingControllerEnabled
                    ? headingController.calculate(
                    headingError, -currentAngularVelocity, applyCorrectiveStatic) : 0.0;
            double headingVelocityFeedback = headingControllerEnabled
                    && Math.abs(omegaTarget) > 1e-6
                    ? angularVelocityFeedbackGain * (omegaTarget - currentAngularVelocity) : 0.0;
            double endpointHeadingError = currentHeading.getShortestAngleTo(
                    path.getEndPose().getHeading()).getRad();
            if (distanceRemaining < PROFILED_ENDPOINT_CAPTURE_DISTANCE_INCHES) {
                headingFeedback = ensureAngularEndpointBreakawayPower(
                        headingFeedback,
                        endpointHeadingError,
                        currentAngularVelocity,
                        applyCorrectiveStatic ? constants.angularCoeffs.kS : 0.0,
                        Math.max(headingTol, MIN_COMPLETION_HEADING_RADIANS)
                );
            }

// Calculate lateral cross track power allocation
            Vector positionalError = targetPoseVec.minus(currentPos);
            crossTrackError = positionalError.dot(lateralNormal).getIn();
            centripetalError = positionalError.dot(normal).getIn();
            double lateralFeedbackMag = driveControllerEnabled
                    ? driveController.calculateCrossTrack(
                    crossTrackError, applyCorrectiveStatic) : 0.0;
            crossTrackCorrection = lateralNormal.times(lateralFeedbackMag);

            centripetalCorrection = calculateCentripetalCorrection(
                    normal, robotTangentialVel, kappa, centripetalGain);

            double totalTangentPower;
            double tangentFeedback = 0.0;
            double tangentVelocityFeedback = 0.0;
            if (t < 1.0) {
                if (isProfiled) {
                    double motionSign = feedforwardMotionSign(
                            targets.getTangentialVel(), targets.getTangentialAccel());
                    double feedforward = translationalKV * targets.getTangentialVel() +
                            translationalKA * targets.getTangentialAccel() +
                            motionSign * constants.translationalFeedforwardKS;

                    // TODO: Verify p only feedback performance, compare to SquID
                    tangentVelocityFeedback = (targets.getTangentialVel() - robotTangentialVel) *
                            velocityFeedbackGain;
                    totalTangentPower = feedforward;

                    if (path.isAccelBoosted()) {
                        double decelPower =
                                driveController.calculateEndDistance(distanceRemaining);
                        if (decelPower <= totalTangentPower + tangentVelocityFeedback) {
                            tangentFeedback = decelPower;
                            tangentVelocityFeedback = 0.0;
                            totalTangentPower = 0.0;
                        }
                    }
                } else {
                    // Closest-point progress can remain just below 1.0 after the chassis passes
                    // a straight endpoint. Near the end, use signed endpoint error so a quick
                    // path brakes and reverses instead of continuing forever.
                    double endDistanceError = distanceRemaining <
                            PROFILED_ENDPOINT_CAPTURE_DISTANCE_INCHES
                            ? signedEndpointError : distanceRemaining;
                    double decelPower = driveController.calculateEndDistance(endDistanceError);
                    double maxVel = commandedForwardVelocity;
                    double velError = maxVel - robotTangentialVel;
                    tangentVelocityFeedback = velError * velocityFeedbackGain;
                    double feedforwardPower = maxVel * translationalKV
                            + Math.signum(maxVel) * constants.translationalFeedforwardKS;
                    double accelPower = feedforwardPower + tangentVelocityFeedback;

                    if (decelPower <= accelPower) {
                        // Deceleration is corrective, so it owns the tangent command. Adding
                        // propulsion afterward would defeat the stopping constraint.
                        tangentFeedback = decelPower;
                        tangentVelocityFeedback = 0.0;
                        totalTangentPower = 0.0;
                    } else {
                        // The robot is still accelerating: velocity correction precedes the
                        // propulsion feedforward in the shared command budget.
                        tangentFeedback = 0.0;
                        totalTangentPower = feedforwardPower;
                    }
                }
            } else {
                // Apply reverse feedback if robot drifts past the final point
                tangentFeedback = isProfiled ? 0.0 :
                        driveController.calculateEndDistance(signedEndpointError);
                totalTangentPower = 0.0;
            }

            if (isProfiled) {
                double endpointPower = driveController.calculateEndDistance(signedEndpointError);
                endpointPower = ensureEndpointBreakawayPower(
                        endpointPower, signedEndpointError, robotTangentialVel,
                        constants.translationalCoeffs.kS,
                        Math.max(distanceTol, MIN_COMPLETION_DISTANCE_INCHES),
                        distanceRemaining);
                double endpointBlend = Range.clip(
                        (PROFILED_ENDPOINT_CAPTURE_DISTANCE_INCHES - distanceRemaining) /
                                PROFILED_ENDPOINT_CAPTURE_DISTANCE_INCHES, 0.0, 1.0);
                tangentFeedback = tangentFeedback * (1.0 - endpointBlend) +
                        endpointPower * endpointBlend;
                tangentVelocityFeedback *= 1.0 - endpointBlend;
                totalTangentPower *= 1.0 - endpointBlend;
            }

// Stage 1: position, heading, and centripetal corrections share equal priority.
            Vector feedbackRobot = crossTrackCorrection.plus(centripetalCorrection)
                    .plus(unitTangent.times(tangentFeedback))
                    .rotate(Angle.fromRad(-currentHeading.getRad()));
            boolean anisotropic = driveModel == HolonomicDriveModel.ANISOTROPIC;
            double feedbackScale = commandNormalizationScale(
                    feedbackRobot.getX().getIn(), feedbackRobot.getY().getIn(),
                    headingFeedback, anisotropic);
            double rawFeedbackDemand = anisotropic
                    ? Math.abs(feedbackRobot.getX().getIn()) +
                    Math.abs(feedbackRobot.getY().getIn()) + Math.abs(headingFeedback)
                    : feedbackRobot.getMag().getIn() + Math.abs(headingFeedback);
            feedbackRobot = feedbackRobot.times(1.0 / feedbackScale);
            headingFeedback /= feedbackScale;
            double feedbackDemand = anisotropic
                    ? Math.abs(feedbackRobot.getX().getIn()) +
                    Math.abs(feedbackRobot.getY().getIn()) + Math.abs(headingFeedback)
                    : feedbackRobot.getMag().getIn() + Math.abs(headingFeedback);

            // Stage 2: translational propulsion receives the remaining power first.
            Vector feedforwardRobot = unitTangent.times(totalTangentPower)
                    .rotate(Angle.fromRad(-currentHeading.getRad()));
            double rawHeadingFeedforwardDemand = Math.abs(headingFF);
            double driveFeedforwardDemand = anisotropic
                    ? Math.abs(feedforwardRobot.getX().getIn()) +
                    Math.abs(feedforwardRobot.getY().getIn())
                    : feedforwardRobot.getMag().getIn();
            double feedforwardDemand = driveFeedforwardDemand + Math.abs(headingFF);
            double remaining = Math.max(0.0, 1.0 - feedbackDemand);
            double driveFeedforwardScale = remainingScale(driveFeedforwardDemand, remaining);
            feedforwardRobot = feedforwardRobot.times(driveFeedforwardScale);
            remaining = Math.max(0.0,
                    remaining - driveFeedforwardDemand * driveFeedforwardScale);

            // Stage 3: forward and heading velocity feedback share the capacity left by
            // corrective control and translational propulsion.
            Vector velocityFeedbackRobot = unitTangent.times(tangentVelocityFeedback)
                    .rotate(Angle.fromRad(-currentHeading.getRad()));
            double velocityFeedbackDemand = anisotropic
                    ? Math.abs(velocityFeedbackRobot.getX().getIn()) +
                    Math.abs(velocityFeedbackRobot.getY().getIn()) +
                    Math.abs(headingVelocityFeedback)
                    : velocityFeedbackRobot.getMag().getIn() +
                    Math.abs(headingVelocityFeedback);
            double rawHeadingVelocityDemand = Math.abs(headingVelocityFeedback);
            double velocityFeedbackScale = remainingScale(velocityFeedbackDemand, remaining);
            velocityFeedbackRobot = velocityFeedbackRobot.times(velocityFeedbackScale);
            headingVelocityFeedback *= velocityFeedbackScale;

            // Predictive angular power uses only capacity left after correction, propulsion,
            // and velocity tracking.
            remaining = Math.max(0.0,
                    remaining - velocityFeedbackDemand * velocityFeedbackScale);
            headingFF *= remainingScale(Math.abs(headingFF), remaining);

            lastCommandDemand = new CommandDemand(true,
                    crossTrackCorrection.getMag().getIn(), Math.abs(tangentFeedback),
                    Math.abs(headingFeedback * feedbackScale),
                    centripetalCorrection.getMag().getIn(),
                    Math.abs(tangentVelocityFeedback), rawHeadingVelocityDemand,
                    Math.abs(totalTangentPower), rawHeadingFeedforwardDemand,
                    rawFeedbackDemand, velocityFeedbackDemand, feedforwardDemand,
                    rawFeedbackDemand + velocityFeedbackDemand + feedforwardDemand);

            Vector finalDriveOutput = feedbackRobot.plus(feedforwardRobot)
                    .plus(velocityFeedbackRobot);
            double turnPow = headingFeedback + headingFF + headingVelocityFeedback;

            double endpointDistance = currentPos.distanceTo(path.getEndPose().getVec()).getIn();
            boolean pathInsideTolerance =
                    endpointDistance < Math.max(distanceTol, MIN_COMPLETION_DISTANCE_INCHES) &&
                    Math.abs(endpointHeadingError) < Math.max(
                            headingTol, MIN_COMPLETION_HEADING_RADIANS);
            boolean pathVelocitySettled =
                    robotVel.getMagSq().getIn() < SETTLED_LINEAR_VELOCITY_SQ &&
                    Math.abs(currentAngularVelocity) < SETTLED_ANGULAR_VELOCITY;
            if (completionReady(pathInsideTolerance, pathVelocitySettled)) {
                stop();
                return;
            }

            drivetrain.drive(
                    finalDriveOutput.getX().getIn(), finalDriveOutput.getY().getIn(), turnPow
            );
            // region Tank Following
        } else {
            // Process tank driving via Ramsete controller
            t = segment.getBestT(currentPos);
            Vector targetPoseVec = segment.getPosition(t);
            double s = segment.getDistanceToEndIn(targetPoseVec, t);

            // Process scheduled distance and angular callbacks
            double pathProgress = 1.0 - s / segment.getLengthIn();
            processCallbacks(Range.clip(pathProgress, 0.0, 1.0), currentHeading);

            Vector velVec = segment.getFirstDerivative(t);
            Vector robotVel = localizer.getVel().getVec();

            Path path = (Path) currentMovement;
            Angle headingTarg = path.getInterpolator().getHeadingTarg(s, velVec,
                    segment.getFirstDerivative(1.0));
            double distanceTraveled = path.getParametricPath().getLengthIn() - s;
            MotionParameters targets =
                    path.getFeedforwardLut().getFFParams(distanceTraveled);

            double v_d = targets.getTangentialVel();
            double a_d = targets.getTangentialAccel();
            double omega_d = targets.getAngularVel();
            double alpha_d = targets.getAngularAccel();

            // Transform global error to robot local frame
            Vector globalError = targetPoseVec.minus(currentPos);
            Vector localError = globalError.rotate(Angle.fromRad(-currentHeading.getRad()));

            double e_x = localError.getX().getIn();
            double e_y = localError.getY().getIn();
            double e_theta = currentHeading.getShortestAngleTo(headingTarg).getRad();

            // Calculate non linear Ramsete gains
            double b = 2.0;
            double zeta = 0.7;
            double k = 2.0 * zeta * Math.sqrt(Math.pow(omega_d, 2) + b * Math.pow(v_d, 2));
            double sinc = (Math.abs(e_theta) < 1e-6) ? 1.0 : Math.sin(e_theta) / e_theta;

            double forwardFeedback = k * e_x * translationalKV;
            double currentAngularVelocity = localizer.getVel().getHeading().getRad();
            double turnFeedback = (k * e_theta + b * v_d * sinc * e_y) * angularKV;
            double feedbackScale = Math.max(1.0,
                    Math.abs(forwardFeedback) + Math.abs(turnFeedback));
            forwardFeedback /= feedbackScale;
            turnFeedback /= feedbackScale;
            double remaining = Math.max(0.0,
                    1.0 - Math.abs(forwardFeedback) - Math.abs(turnFeedback));

            double actualForwardVelocity = robotVel.rotate(
                    Angle.fromRad(-currentHeading.getRad())).getX().getIn();
            double forwardVelocityFeedback =
                    (v_d - actualForwardVelocity) * velocityFeedbackGain;
            double turnVelocityFeedback =
                    Math.abs(omega_d) > 1e-6
                            ? angularVelocityFeedbackGain *
                            (omega_d - currentAngularVelocity) : 0.0;
            double velocityFeedbackDemand = Math.abs(forwardVelocityFeedback) +
                    Math.abs(turnVelocityFeedback);
            double forwardFeedforward = v_d * Math.cos(e_theta) * translationalKV +
                    a_d * translationalKA + Math.signum(v_d) *
                    constants.translationalFeedforwardKS;
            double turnFeedforward = omega_d * angularKV + alpha_d * angularKA +
                    Math.signum(omega_d) * constants.angularFeedforwardKS;
            double feedforwardDemand = Math.abs(forwardFeedforward) +
                    Math.abs(turnFeedforward);
            double feedforwardScale = remainingScale(feedforwardDemand, remaining);
            forwardFeedforward *= feedforwardScale;
            turnFeedforward *= feedforwardScale;
            remaining = Math.max(0.0,
                    remaining - feedforwardDemand * feedforwardScale);
            double velocityFeedbackScale = remainingScale(velocityFeedbackDemand, remaining);
            forwardVelocityFeedback *= velocityFeedbackScale;
            turnVelocityFeedback *= velocityFeedbackScale;
            double totalTangentPower = forwardFeedback + forwardFeedforward +
                    forwardVelocityFeedback;
            double turnPow = turnFeedback + turnFeedforward + turnVelocityFeedback;

            if (s < distanceTol && robotVel.getMagSq().getIn() < 16) {
                stop();
                return;
            }

            drivetrain.drive(totalTangentPower, 0.0, turnPow);
        }
    }

    /**
     * Starts following the given movement.
     *
     * @param movement The movement object to be executed.
     * @throws IllegalStateException if the follower is already busy executing a movement.
     */
    public void follow(FollowerMovement movement) {
        if (isBusy()) {
            throw new IllegalStateException(
                    "Cannot execute a new movement while another movement is still in progress. " +
                            "Tip: use follower.isBusy() to check if the follower is currently " +
                            "executing a movement before starting a new one."
            );
        }

        this.currentMovement = movement;
        this.completionToleranceEnteredNanos = 0L;
        this.previousPathDistanceIn = Double.NaN;
        this.previousPathProgressNanos = 0L;
        this.pathProgressVelocityIn = 0.0;
        this.currentMovement.setStarted(true);
        this.currentMovement.setEnded(false);
        this.targetHeading = movement.getEndPose().getHeading();
        this.lastPose = movement.getEndPose();

        if (movement instanceof Turn) {
            diagnostics.clearCurrentPath();
            Turn turn = (Turn) currentMovement;
            this.targetTurnPoseVec = turn.getStartPose().getVec();
            double signedTurn = turn.getStartPose().getHeading().getShortestAngleTo(
                    turn.getEndPose().getHeading()
            ).getRad();
            this.turnDirection = Math.signum(signedTurn);
            this.turnTotalDisplacement = Math.abs(signedTurn);
        } else if (movement instanceof Path) {
            Path pathSegmentMove = (Path) currentMovement;
            this.segment = pathSegmentMove.getParametricPath();
            diagnostics.recordCurrentPath(pathSegmentMove);
            if (drivetrain instanceof DualActuated) {
                if (pathSegmentMove.getPathType() == Path.PathType.HOLONOMIC) {
                    ((DualActuated) drivetrain).activateHolonomicState();
                } else {
                    ((DualActuated) drivetrain).activateTractionState();
                }
            }
        }

        headingController.reset();
        turnController.reset();
        driveController.reset();
        paused = false;

        // Reset tracker for angular callbacks so it doesn't instantly trigger on path start
        lastHeading = null;
    }

    /** Instantly stops the drivetrain and ends any ongoing movement. */
    public void stop() {
        if (this.currentMovement != null) {
            this.currentMovement.setEnded(true);
        }

        this.currentMovement = null;
        this.segment = null;
        this.targetHeading = null;
        this.targetTurnPoseVec = null;
        this.turnDirection = 0.0;
        this.turnTotalDisplacement = 0.0;
        this.completionToleranceEnteredNanos = 0L;
        this.previousPathDistanceIn = Double.NaN;
        this.previousPathProgressNanos = 0L;
        this.pathProgressVelocityIn = 0.0;

        diagnostics.clearCurrentPath();

        this.drivetrain.stop();
    }

    /**
     * Installs an optional diagnostics observer. Intended for simulator and logging integrations.
     * Passing {@code null} restores the dependency-free no-op production default.
     */
    public static void setDiagnostics(FollowerDiagnostics observer) {
        diagnostics = observer == null ? NO_DIAGNOSTICS : observer;
    }

    /** Halts the current movement temporarily without clearing the target state */
    public void pause() {
        this.paused = true;
        this.drivetrain.stop();
    }

    /** Resumes a paused movement from the robots current location. */
    public void resume() {
        if (this.paused) {
            this.paused = false;
        }
    }

    static double pathEndpointTangentError(Vector endpoint, Vector current, Vector endTangent) {
        return endpoint.minus(current).dot(endTangent).getIn();
    }

    static double blendProfiledEndpointPower(double profilePower, double endpointPower,
                                               double pathDistanceRemaining) {
        double blend = Range.clip(
                (PROFILED_ENDPOINT_CAPTURE_DISTANCE_INCHES - pathDistanceRemaining) /
                        PROFILED_ENDPOINT_CAPTURE_DISTANCE_INCHES,
                0.0,
                1.0
        );
        return profilePower * (1.0 - blend) + endpointPower * blend;
    }

    static double ensureEndpointBreakawayPower(double requestedPower, double endpointError,
                                                double tangentialVelocity, double staticGain,
                                                double positionTolerance,
                                                double pathDistanceRemaining) {
        boolean inEndpointCapture = pathDistanceRemaining <
                PROFILED_ENDPOINT_CAPTURE_DISTANCE_INCHES;
        boolean outsideTolerance = Math.abs(endpointError) > positionTolerance;
        boolean stalled = Math.abs(tangentialVelocity) <
                ENDPOINT_STALLED_VELOCITY_IN_PER_SECOND;
        if (!inEndpointCapture || !outsideTolerance || !stalled) { return requestedPower; }

        double minimumPower = Math.min(1.0, Math.abs(staticGain));
        if (Math.abs(requestedPower) >= minimumPower) { return requestedPower; }
        return Math.copySign(minimumPower, endpointError);
    }

    static double ensureAngularEndpointBreakawayPower(double requestedPower, double headingError,
                                                       double angularVelocity, double staticGain,
                                                       double headingTolerance) {
        boolean inEndpointCapture = Math.abs(headingError) <
                PROFILED_HEADING_CAPTURE_RADIANS;
        boolean outsideTolerance = Math.abs(headingError) > headingTolerance;
        boolean stalled = Math.abs(angularVelocity) < 0.05;
        if (!inEndpointCapture || !outsideTolerance || !stalled) { return requestedPower; }

        double minimumPower = Math.min(1.0, Math.abs(staticGain));
        if (Math.abs(requestedPower) >= minimumPower) { return requestedPower; }
        return Math.copySign(minimumPower, headingError);
    }

    /**
     * Drives the robot using the provided inputs. The joystick inputs are adjusted for
     * field-centric or robot-centric control based on the constants. Any active follower movement
     * will be stopped as manual control takes priority over following a path. If you want to use
     * a standard control scheme, you can pass your gamepad to the other manual method.
     *
     * @param x forward/backward input where positive is forward
     * @param y left/right input where positive is left
     * @param turn rotation input where positive is counter-clockwise
     */
    public void manual(double x, double y, double turn) {
        if (isBusy()) { stop(); }
        drivetrain.drive(x, y, turn, this.getPose().getHeading(AngleUnit.RAD));
    }

    /**
     * Drives the robot using standard gamepad inputs. The left stick controls forward/backward and
     * left/right movement, while the right stick controls rotation. Any active follower movement
     * will be stopped as manual control takes priority over following a path. If you want to
     * use a different control scheme, use the other manual method with custom inputs.
     *
     * @param gamepad the gamepad to read inputs from
     */
    public void manual(Gamepad gamepad) {
        // Left stick Y is negated because forward is negative on the gamepad
        // Left stick X is negated because left is positive in the coordinate system
        // Right stick X is negated because CC is positive in the coordinate system.
        manual(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x);
    }

    /**
     * Retrieves the robots current pose estimate from the localizer.
     *
     * @return The current global position and heading.
     */
    public Pose getPose() { return localizer.getPose(); }

    /**
     * Checks if the follower is currently executing a movement.
     *
     * @return true if a movement is in progress false otherwise.
     */
    public boolean isBusy() { return currentMovement != null; }

    /**
     * Forcibly overrides the localizers current pose estimate.
     *
     * @param pose The new global position and heading.
     */
    public void setPose(Pose pose) { localizer.setPose(pose); }

    /**
     * Retrieves the robots current velocity estimate from the localizer.
     *
     * @return The current velocity expressed in robot local frame.
     */
    public Pose getVelocity() { return localizer.getVel(); }

    /** Returns the unfiltered localizer velocity for diagnostics and response logging. */
    public Pose getRawVelocity() { return localizer.getRawVel(); }

    /**
     * Retrieves the robots current acceleration estimate from the localizer.
     *
     * @return The current acceleration expressed in robot local frame.
     */
    public Pose getAcceleration() { return localizer.getAccel(); }

    public double getBestT() { return t; }

    public double getCrossTrackErrorIn() { return crossTrackError; }

    /** Signed path error toward the local center of curvature; positive means outside the turn. */
    public double getCentripetalErrorIn() { return centripetalError; }

    /** Returns the latest pre-normalization command breakdown for saturation diagnosis. */
    public CommandDemand getLastCommandDemand() { return lastCommandDemand; }

    /** The most recent closest point used by the holonomic follower. Intended for diagnostics. */
    public Vector getClosestPathPoint() { return closestPathPoint; }

    /** Continuous left-hand path normal used to define signed cross-track feedback. */
    public Vector getCrossTrackNormal() { return crossTrackNormal; }

    /** The principal path normal, which always points toward the local center of curvature. */
    public Vector getPathNormal() { return pathNormal; }

    /** Field-space feedback vector that corrects cross-track error. */
    public Vector getCrossTrackCorrection() { return crossTrackCorrection; }

    /** Field-space feedforward vector that supplies centripetal acceleration. */
    public Vector getCentripetalCorrection() { return centripetalCorrection; }

    public void disableHeadingController() { this.headingControllerEnabled = false; }

    public void disableDriveController() { this.driveControllerEnabled = false; }

    public void disableControllers() { disableHeadingController(); disableDriveController(); }

    public void enableHeadingController() { this.headingControllerEnabled = true; }

    public void enableDriveController() { this.driveControllerEnabled = true; }

    public void enableControllers() { enableHeadingController(); enableDriveController(); }

    public void setHeadingCoefficients(PDSCoefficients coefficients) {
        headingController.setCoefficients(coefficients);
        turnController.setCoefficients(coefficients);
    }

    public void setDriveCoefficients(PDSCoefficients coefficients) {
        driveController.setCoefficients(coefficients);
    }

    public void setCentripetal(double centripetalGain) { this.centripetalGain = centripetalGain; }

    /**
     * Builds centripetal power from a principal normal. Because the normal already contains the
     * bend direction, curvature contributes magnitude only; applying its sign again reverses the
     * force on clockwise/right-hand curves.
     */
    static Vector calculateCentripetalCorrection(Vector principalNormal,
                                                  double tangentialVelocity,
                                                  double signedCurvature,
                                                  double gain) {
        double magnitude = tangentialVelocity * tangentialVelocity *
                Math.abs(signedCurvature) * gain;
        return principalNormal.times(Math.min(magnitude, MAX_CENTRIPETAL_POWER));
    }

    /** Uses acceleration to select static-friction direction while a profile starts from rest. */
    static double feedforwardMotionSign(double targetVelocity, double targetAcceleration) {
        if (Math.abs(targetVelocity) > 1e-6) { return Math.signum(targetVelocity); }
        if (Math.abs(targetAcceleration) > 1e-6) { return Math.signum(targetAcceleration); }
        return 0.0;
    }

    public void setVelocityFeedback(double velocityFeedbackGain,
                                    double angularVelocityFeedbackGain) {
        this.velocityFeedbackGain = velocityFeedbackGain;
        this.angularVelocityFeedbackGain = angularVelocityFeedbackGain;
        turnController.setMotionGains(angularKV, angularKA, angularVelocityFeedbackGain);
    }

    /** Applies a newly refined drivetrain model without reconstructing the follower. */
    public void setFeedforwardGains(double translationalKV, double translationalKA,
                                    double angularKV, double angularKA) {
        this.translationalKV = translationalKV;
        this.translationalKA = translationalKA;
        this.angularKV = angularKV;
        this.angularKA = angularKA;
        turnController.setMotionGains(angularKV, angularKA, angularVelocityFeedbackGain);
    }

    /** Applies refined moving-friction and dynamic feedforward gains. */
    public void setFeedforwardGains(double translationalKS, double translationalKV,
                                    double translationalKA, double angularKS,
                                    double angularKV, double angularKA) {
        constants.translationalFeedforwardKS = translationalKS;
        constants.angularFeedforwardKS = angularKS;
        this.translationalKV = translationalKV;
        this.translationalKA = translationalKA;
        this.angularKV = angularKV;
        this.angularKA = angularKA;
        turnController.setMotionGains(
                angularKV, angularKA, angularKS, angularVelocityFeedbackGain);
    }

    /** This method is intended for internal use only. */
    public BaseLocalizer<?> getLocalizer() { return localizer; }

    /** This method is intended for internal use only. */
    public BaseDrivetrain<?> getDrivetrain() { return drivetrain; }

    /** This method is intended for internal use only. */
    public FollowerConstants getConstants() { return constants; }

    // endregion
}
