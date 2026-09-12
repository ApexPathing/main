package core;

import controllers.DriveController;
import controllers.DriveController.AllocatedCommand;
import drivetrains.BaseDrivetrain;
import drivetrains.DualActuated;
import drivetrains.Mecanum;
import geometry.Angle;
import geometry.Vector;

/** Allocates holonomic correction, propulsion, and velocity commands into one motor-power budget. */
final class HolonomicCommandAllocator {
    enum DriveModel { ANISOTROPIC, ISOTROPIC }

    static final class Result {
        final Vector drive;
        final double turn;
        final Follower.CommandDemand demand;

        Result(Vector drive, double turn, Follower.CommandDemand demand) {
            this.drive = drive;
            this.turn = turn;
            this.demand = demand;
        }
    }

    private final BaseDrivetrain<?> drivetrain;
    private final DriveController driveController;

    HolonomicCommandAllocator(BaseDrivetrain<?> drivetrain, DriveController driveController) {
        this.drivetrain = drivetrain;
        this.driveController = driveController;
    }

    DriveModel activeDriveModel() {
        if (drivetrain instanceof Mecanum) {
            return DriveModel.ANISOTROPIC;
        }
        if (drivetrain instanceof DualActuated) {
            if (!drivetrain.isHolonomic()) {
                throw new IllegalStateException(
                        "Dual-actuated drivetrain is not in its holonomic state.");
            }
            return DriveModel.ANISOTROPIC;
        }
        if (!drivetrain.isHolonomic()) {
            throw new IllegalStateException(
                    "A holonomic allocation was requested while the drivetrain was non-holonomic.");
        }
        return DriveModel.ISOTROPIC;
    }

    AllocatedCommand allocatePositionHold(Vector fieldCommand, Angle currentHeading,
                                          double availablePower) {
        if (activeDriveModel() == DriveModel.ANISOTROPIC) {
            return driveController.allocateMecanum(fieldCommand, currentHeading, availablePower);
        }
        return driveController.allocateIsotropic(fieldCommand, currentHeading, availablePower);
    }

    Result allocate(Vector crossTrackCorrection, Vector centripetalCorrection, Vector unitTangent,
                    Angle currentHeading, double tangentFeedback, double headingFeedback,
                    double tangentFeedforward, double tangentVelocityFeedback,
                    double headingVelocityFeedback, double headingFeedforward) {
        boolean anisotropic = activeDriveModel() == DriveModel.ANISOTROPIC;

        Vector feedbackRobot = crossTrackCorrection.plus(centripetalCorrection)
                .plus(unitTangent.times(tangentFeedback))
                .rotate(Angle.fromRad(-currentHeading.getRad()));
        double feedbackScale = normalizationScale(
                feedbackRobot.getX().getIn(), feedbackRobot.getY().getIn(),
                headingFeedback, anisotropic);
        double rawFeedbackDemand = demand(feedbackRobot, headingFeedback, anisotropic);
        feedbackRobot = feedbackRobot.times(1.0 / feedbackScale);
        headingFeedback /= feedbackScale;
        double feedbackDemand = demand(feedbackRobot, headingFeedback, anisotropic);

        Vector feedforwardRobot = unitTangent.times(tangentFeedforward)
                .rotate(Angle.fromRad(-currentHeading.getRad()));
        double rawHeadingFeedforwardDemand = Math.abs(headingFeedforward);
        double driveFeedforwardDemand = translationDemand(feedforwardRobot, anisotropic);
        double feedforwardDemand = driveFeedforwardDemand + Math.abs(headingFeedforward);
        double remaining = Math.max(0.0, 1.0 - feedbackDemand);
        double driveFeedforwardScale = remainingScale(driveFeedforwardDemand, remaining);
        feedforwardRobot = feedforwardRobot.times(driveFeedforwardScale);
        remaining = Math.max(0.0,
                remaining - driveFeedforwardDemand * driveFeedforwardScale);

        Vector velocityFeedbackRobot = unitTangent.times(tangentVelocityFeedback)
                .rotate(Angle.fromRad(-currentHeading.getRad()));
        double velocityFeedbackDemand = demand(
                velocityFeedbackRobot, headingVelocityFeedback, anisotropic);
        double rawHeadingVelocityDemand = Math.abs(headingVelocityFeedback);
        double velocityFeedbackScale = remainingScale(velocityFeedbackDemand, remaining);
        velocityFeedbackRobot = velocityFeedbackRobot.times(velocityFeedbackScale);
        headingVelocityFeedback *= velocityFeedbackScale;

        remaining = Math.max(0.0,
                remaining - velocityFeedbackDemand * velocityFeedbackScale);
        headingFeedforward *= remainingScale(Math.abs(headingFeedforward), remaining);

        Follower.CommandDemand commandDemand = new Follower.CommandDemand(true,
                crossTrackCorrection.getMag().getIn(), Math.abs(tangentFeedback),
                Math.abs(headingFeedback * feedbackScale),
                centripetalCorrection.getMag().getIn(),
                Math.abs(tangentVelocityFeedback), rawHeadingVelocityDemand,
                Math.abs(tangentFeedforward), rawHeadingFeedforwardDemand,
                rawFeedbackDemand, velocityFeedbackDemand, feedforwardDemand,
                rawFeedbackDemand + velocityFeedbackDemand + feedforwardDemand);

        return new Result(
                feedbackRobot.plus(feedforwardRobot).plus(velocityFeedbackRobot),
                headingFeedback + headingFeedforward + headingVelocityFeedback,
                commandDemand);
    }

    static double normalizationScale(double x, double y, double turn, boolean anisotropic) {
        double translation = anisotropic ? Math.abs(x) + Math.abs(y) : Math.hypot(x, y);
        return Math.max(1.0, translation + Math.abs(turn));
    }

    static double remainingScale(double demand, double available) {
        return demand > available && demand > 1e-12 ? available / demand : 1.0;
    }

    private static double demand(Vector command, double turn, boolean anisotropic) {
        return translationDemand(command, anisotropic) + Math.abs(turn);
    }

    private static double translationDemand(Vector command, boolean anisotropic) {
        return anisotropic
                ? Math.abs(command.getX().getIn()) + Math.abs(command.getY().getIn())
                : command.getMag().getIn();
    }
}
