package paths;

import paths.builders.MovementBuilder; // Updated to use the unified builder
import paths.builders.PathBuilder;
import paths.builders.TurnBuilder;
import paths.movements.FollowerMovement;
import paths.movements.Path;
import paths.movements.Turn;

import geometry.Angle;
import geometry.Pose;
import util.AngleUnit;
import util.DistUnit;
import util.PoseFactory;

public class ExamplePathAPIV3 {
    private DistUnit distUnit = DistUnit.IN;
    private AngleUnit angleUnit = AngleUnit.DEG;
    public PoseFactory pose = new PoseFactory(distUnit, angleUnit);
    private Pose startPose;

    // Storing our routine components cleanly
    public Path testPath;
    public Turn testTurn;

    public ExamplePathAPIV3(PoseFactory.Mirror mirror) {
        pose.setMirror(mirror);
        startPose = Pose.Common.CENTER.get(); // (0, 0, 0)
        buildRoutine();
    }

    public void exampleCallback() {
        // This will run when the follower reaches a specific progression or angle
    }

    /**
     * A comprehensive showcase of the Unified Movement Builder API.
     */
    private void buildRoutine() {

        // 1. THE CORE B-SPLINE
        // Explicitly casting the FollowerMovement return type to a concrete Path
        testPath = (Path) new PathBuilder(startPose)
                // A B-Spline can be created with 2 points in Apex because of ghost points that are added during construction
                .addControlPoints(
                        pose.of(15, 0), // Standard waypoint
                        pose.of(25, 0, 90), // INTENTIONAL WARNING: Apex will ignore this intermediate heading and warn the user!
                        pose.arcPoseOf(25, 25, 10), // ArcEnforcement: Forces large, relaxed curves into a sharper turn with a 10in radius while maintaining C2 continuity
                        pose.of(45, 25, 45) // The final waypoint dictates the target heading for the end of this curve
                )

                // 2. DISTANCE CALLBACK: Triggers our custom function exactly halfway (s=0.5) down the curve
                .addDistanceCallback(0.5, this::exampleCallback)

                // 3. ANGULAR CALLBACK: Triggers precisely when the robot rotates past the 180-degree mark
                .addAngularCallback(Angle.fromRad(Math.PI), this::exampleCallback)

                // 4. ADVANCED LAMBDA INTERPOLATOR
                // Overrides the previous curve's heading logic with custom math.
                // Here, we command the robot to do a full 360-degree tornado spin over the course of the curve.
                .interpolateWith(s -> Angle.fromDeg(180 + (s * 360.0)))

                // 5. COMPILE: Locks the path, calculates all Look-Up Tables, and finalizes geometry.
                .build();

        // ---------------------------------------------------------

        // 6. THE TURN BUILDER
        // Seamlessly starts EXACTLY where the last path ended using .getEndPose()
        // Explicitly casting the FollowerMovement return type to a concrete Turn
        testTurn = (Turn) new TurnBuilder(testPath.getEndPose())
                // Defines the final heading the robot should rotate to
                .turnTo(Angle.fromRad(Math.PI / 2))

                // Safety validated callback during the spin!
                .addAngularCallback(Angle.fromRad(Math.PI / 3), this::exampleCallback)

                // Locks the turn and finalizes the callback math
                .build();
    }

    /**
     * Optional helper to retrieve the full routine for a state machine.
     */
    public FollowerMovement[] getAutoRoutine() {
        return new FollowerMovement[] { testPath, testTurn };
    }
}