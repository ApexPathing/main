package paths;

import paths.heading.InterpolationStyle;
import util.Angle;
import util.Distance;
import util.Pose;
import util.PoseBuilder;

public class ExamplePathAPIV3 {
    private Distance.Units distUnit = Distance.Units.INCHES;
    private Angle.Units angleUnit = Angle.Units.DEGREES;
    public PoseBuilder pose = new PoseBuilder(distUnit, angleUnit);
    private Pose startPose;

    public ExamplePathAPIV3(boolean mirror) {
        pose.setMirror(mirror);
        // Uses the builder's build method to apply units and mirroring
        startPose = pose.build(0, 0, 0);
    }

    public void exampleCallback() {
        // This will run when the follower reaches 50% of the path
    }

    public Path testPath() {
        return new PathBuilder(startPose)
                .addControlPoints(
                        pose.at(10, 0),
                        /* TODO: Implement tighten method in Path */
                        Path.tighten(pose.at(15, 15)),
                        pose.at(25, 25, 90)
                )
                .addCallback(0.5, () -> {
                    exampleCallback();
                })
                .interpolateWith(InterpolationStyle.TANGENT_OPTIMAL)
                .build();
    }
}