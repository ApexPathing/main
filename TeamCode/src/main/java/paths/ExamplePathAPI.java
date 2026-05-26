package paths;

import paths.heading.HeadingInterpolator;
import paths.heading.InterpolationStyle;
import util.Angle;
import util.Distance;
import util.Pose;

/**
 * NOTE TO GITHUB SNOOPERS: This is subject to change, please let us know if you have suggestions! We'd
 * love to hear from you!
 */
public class ExamplePathAPI {

    public Path testPath() {
        return new PathBuilder(new Pose(0, 0, 0, Distance.Units.INCHES, Angle.Units.RADIANS))

                // 2. GLOBAL OVERRIDE: Set the default behavior for all following segments.
                // Options: TANGENT_OPTIMAL, TANGENT_FORWARD, SMOOTH_START_TO_END (Default)
                .setInterpolationStyle(InterpolationStyle.TANGENT_OPTIMAL)

                // 3. THE B-SPLINE: A fluid curve through multiple control points.
                .bSplineTo(
                        // Apex handles unit conversions automatically!
                        new Pose(Distance.Units.MILLIMETERS, 600, 0),

                        // Using the (x, y) only constructor.
                        // Intermediate headings are ignored (Apex will warn you if you put one here!)
                        new Pose(15, 10),

                        // The heading of the FINAL pose dictates the robot's target angle at the end of the curve.
                        new Pose(25, 20, Math.toRadians(90))
                )

                // 4. POINT TURN: Stationary rotation.
                // The robot stops translating, rotates to the target angle, and updates the tracker state.
                .turnTo(Angle.fromDeg(135))

                // 5. STRAIGHT LINE: Simple point-to-point translation.
                .lineTo(
                        new Pose(0, 0, Math.toRadians(180))
                )

                // 6. IN-LINE OVERRIDE: Overrides the heading strategy of the segment generated directly above it.
                // This replaces TANGENT_OPTIMAL for that specific lineTo() segment with TANGENT_FORWARD.
                .interpolateSegment((InterpolationStyle.TANGENT_FORWARD))

                // 7. ADVANCED LAMBDA OVERRIDE: Want the robot to spin exactly 3 times over the course of a curve?
                // You can pass a custom function where 's' is the distance percentage (0.0 to 1.0).
                .bSplineTo(
                        new Pose(10, 10),
                        new Pose(20, 0, 0)
                )
                .interpolateSegment((s -> new Angle(s * (6 * Math.PI))))

                // 8. FAILSAFE DEMONSTRATION: Missing headings!
                // We provide no end heading here. Apex won't crash; it will fall back to TANGENT_FORWARD
                // and push a warning string to the path object.
                .lineTo(
                        new Pose(Distance.Units.INCHES, 30, 0)
                )

                // 9. COMPILE: Locks the path and calculates all the Look-Up Tables.
                .build();
    }
}