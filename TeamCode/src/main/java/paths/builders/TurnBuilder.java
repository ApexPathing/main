package paths.builders;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import paths.callbacks.Callback;
import paths.movements.Turn;
import geometry.Angle;
import geometry.Pose;

/**
 * A builder class designed to construct a {@link Turn} fluently.
 * <p>
 * This handles stationary point-turns, allowing users to inject mechanical
 * callbacks at specific angles during the rotation.
 */
public class TurnBuilder implements MovementBuilder<Turn> {
    private final Pose startPose;
    private Angle targetHeading = null;

    // Instead of a Runnable, we use a Consumer so we can pass the finalized Turn object into the lambda later
    private final List<Consumer<Turn>> buildTasks = new ArrayList<>();

    /**
     * Initializes the TurnBuilder with the robot's starting state.
     * @param startPose The Pose of the robot before the turn begins.
     */
    public TurnBuilder(Pose startPose) { this.startPose = startPose; }

    /**
     * Defines the target angle for the point turn.
     * @param targetHeading The Angle to rotate to.
     * @return The current TurnBuilder instance.
     */
    public TurnBuilder turnTo(Angle targetHeading) {
        this.targetHeading = targetHeading;
        return this;
    }

    /**
     * Attaches an executable callback to trigger when the robot passes a specific angle during the turn.
     *
     * @param angle The angle at which the callback should trigger.
     * @param action The code to execute.
     * @return The current TurnBuilder instance.
     */
    public TurnBuilder addAngularCallback(Angle angle, Runnable action) {
        // We define the validation math now, but wait to execute it until build() is called
        buildTasks.add((finalTurn) -> {

            double startRad = finalTurn.getStartPose().getHeading().getRad();
            double endRad = finalTurn.getEndPose().getHeading().getRad();
            double targetRad = angle.getRad();

            double totalDiff = getShortestAngularDifference(startRad, endRad);
            double targetDiff = getShortestAngularDifference(startRad, targetRad);

            if (Math.abs(totalDiff) < 1e-6) {
                if (Math.abs(targetDiff) > 1e-6) {
                    throw new IllegalArgumentException("Callback out of bounds: The turn has no rotational distance.");
                }
            } else if ((totalDiff * targetDiff < 0) || (Math.abs(targetDiff) > Math.abs(totalDiff))) {
                throw new IllegalArgumentException("Angular callback is outside the sweep range of this turn.");
            }

            finalTurn.addCallback(new Callback(angle, action));
        });

        return this;
    }

    /**
     * Compiles the turn, verifies callback bounds, and returns the executable Turn movement.
     * @return The fully constructed {@link Turn}.
     */
    @Override
    public Turn build() {
        if (targetHeading == null) {
            throw new IllegalStateException("Cannot build Turn: No target heading was specified! Use .turnTo().");
        }

        // Create the final, accurate Turn object
        Turn finalTurn = new Turn(startPose, targetHeading);

        // Execute all the deferred math and callback attachments, handing them the finalTurn object
        for (Consumer<Turn> task : buildTasks) {
            task.accept(finalTurn);
        }

        return finalTurn;
    }

    /**
     * Helper method to calculate the shortest signed angular difference between two radians.
     * Result is always in the range [-PI, PI].
     */
    private double getShortestAngularDifference(double from, double to) {
        double diff = (to - from) % (2 * Math.PI);
        if (diff > Math.PI) diff -= 2 * Math.PI;
        else if (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }
}