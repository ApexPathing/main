package paths.movements;

import java.util.ArrayList;
import java.util.List;

import geometry.Pose;
import paths.Callback;

/**
 * Base class for {@link Path} and {@link Turn} movements.
 *
 * @author Sohum Arora - 22985 Paraducks
 */
public abstract class FollowerMovement {
    protected Pose endPose;
    private boolean started = false;
    private boolean ended = false;
    private final List<Callback> callbacks = new ArrayList<Callback>();

    /**
     * Gets the expected final pose of the robot after this movement is completed. Generally, this
     * is used to get the start pose of the next movement in a sequence.
     *
     * @return the final Pose.
     */
    public Pose getEndPose() { return endPose; }

    public boolean hasStarted() { return started; }

    public boolean hasEnded() { return ended; }

    public void setStarted(boolean started) { this.started = started; }

    public void setEnded(boolean ended) { this.ended = ended; }

    /** @return Fresh callback objects attached specifically to this traversal. */
    public Callback[] getCallbacks() { return callbacks.toArray(new Callback[0]); }

    /** Adds a callback without exposing the mutable callback collection. */
    protected void addCallback(Callback callback) { callbacks.add(callback); }

    /** Returns an independent movement that traverses this movement in reverse. */
    public abstract FollowerMovement reversed();

    public Path toPath() { return (Path) this; }

    public Turn toTurn() { return (Turn) this; }
}
