package org.firstinspires.ftc.teamcode.apexpathing;

import core.Follower;
import geometry.Angle;
import geometry.Pose;
import paths.heading.InterpolationStyle;
import paths.builders.TurnBuilder;
import paths.movements.Path;
import paths.movements.Turn;
import geometry.AngleUnit;
import geometry.DistUnit;
import geometry.GeometryFactory;

public class ExampleAutoPath {
    private static final Pose startPose = Pose.zero();

    public GeometryFactory factory;
    public Path testPath;
    public Turn testTurn;
    public Path returnPath;
    public Path strafeOutPath;
    public Path strafeBackPath;
    public String callbackMessage = "Callback not triggered yet";
    public boolean outboundCallbackTriggered;
    public boolean turnCallbackTriggered;
    public boolean returnCallbackTriggered;

    public ExampleAutoPath(Follower follower, GeometryFactory.PoseMirror mirror) {
        factory = new GeometryFactory(follower)
                .setDistUnit(DistUnit.IN)
                .setAngleUnit(AngleUnit.DEG)
                .setPoseMirror(mirror);
        build();
    }

    public void exampleDistanceCallback() {
        outboundCallbackTriggered = true;
        callbackMessage = "Outbound distance callback triggered!";
    }

    public void exampleAngularCallback() {
        turnCallbackTriggered = true;
        callbackMessage = "Angular callback triggered!";
    }

    public void exampleReturnCallback() {
        returnCallbackTriggered = true;
        callbackMessage = "Return distance callback triggered!";
    }

    private void build() {
        testPath = factory.holonomicPath(startPose, // Forward and left curve
                        factory.arcPose(30, 0, 7),
                        factory.arcPose(30, -30, 7),
                        factory.arcPose(-30, -30, 7),
                        factory.arcPose(-30, 30, 7),
                        factory.pose(30, 30, -90)
                )
                .interpolateWith(InterpolationStyle.TANGENT_OPTIMAL)
                .addDistanceCallback(0.5, this::exampleDistanceCallback)
                .profiledBuild();
        Angle turnStart = testPath.getEndPose().getHeading();
        Angle turnEnd = factory.angle(0);
        Angle turnSweep = turnStart.getShortestAngleTo(turnEnd);
        TurnBuilder turnBuilder = factory.turn(testPath.getEndPose()).turnTo(turnEnd);
        // Only attach an angular callback when there is a real sweep.
        if (Math.abs(turnSweep.getRad()) >= 1e-6) {
            turnBuilder.addAngularCallback(turnStart.plus(turnSweep.times(.5)),
                    this::exampleAngularCallback);
        }
        testTurn = turnBuilder.quickBuild();
        returnPath = factory.holonomicPath(testTurn.getEndPose(),
                        factory.pose(0, 30),
                        startPose
                )
                .interpolateWith(InterpolationStyle.TANGENT_BACKWARD)
                .setDistanceToStartFinalTurn(factory.dist(30))
                .addDistanceCallback(0.5, this::exampleReturnCallback)
                .profiledBuild();
        Pose strafeEnd = factory.pose(0, 24, 0);
        strafeOutPath = factory.holonomicPath(startPose, strafeEnd)
                .interpolateWith(InterpolationStyle.CONSTANT_START_HEADING)
                .profiledBuild();
        strafeBackPath = factory.holonomicPath(strafeEnd, startPose)
                .interpolateWith(InterpolationStyle.CONSTANT_START_HEADING)
                .profiledBuild();
    }
}
