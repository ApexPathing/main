package core;

import geometry.Vector;

/** Mutable path-execution state shared by the path executors and Follower diagnostics. */
final class FollowerPathState {
    final PathCompletionPolicy completion = new PathCompletionPolicy();
    final PathProgressEstimator progressEstimator = new PathProgressEstimator();
    final FollowerStuckWatchdog stuckWatchdog = new FollowerStuckWatchdog();

    double t;
    double crossTrackError;
    double centripetalError;
    Vector closestPathPoint = Vector.zero();
    Vector crossTrackNormal = Vector.zero();
    Vector pathNormal = Vector.zero();
    Vector crossTrackCorrection = Vector.zero();
    Vector centripetalCorrection = Vector.zero();
    Follower.CommandDemand lastCommandDemand = Follower.CommandDemand.ZERO;
    double trackingVelocityTarget;
    double trackingMeasuredVelocity;
    double trackingProgressVelocity;
    double trackingFeedforward;
    double trackingVelocityFeedback;
    double trackingAngularVelocityTarget;
    double trackingEndpointBlend;
    boolean velocitySampleAvailable;

    void reset() {
        completion.reset();
        progressEstimator.reset();
        stuckWatchdog.reset();
        velocitySampleAvailable = false;
    }

    void pause() {
        velocitySampleAvailable = false;
        stuckWatchdog.reset();
    }
}
