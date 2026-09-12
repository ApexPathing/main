package paths.heading;

import geometry.Angle;
import geometry.Vector;

/** Replays another heading interpolator over the same path in reverse traversal order. */
public final class ReversedHeadingInterpolator implements HeadingInterpolator {
    private final HeadingInterpolator source;
    private final Vector sourceFinalTangent;
    private double pathLength;

    public ReversedHeadingInterpolator(HeadingInterpolator source, double pathLength,
                                       Vector sourceFinalTangent) {
        this.source = source;
        this.pathLength = pathLength;
        this.sourceFinalTangent = sourceFinalTangent;
    }

    @Override
    public Angle getHeadingTarg(double s, Vector pathTangent, Vector finalTangent) {
        return source.getHeadingTarg(pathLength - s, pathTangent.times(-1.0),
                sourceFinalTangent);
    }

    @Override
    public double getHeadingFirstDerivative(double s, double kappa, Vector finalTangent) {
        return -source.getHeadingFirstDerivative(pathLength - s, -kappa,
                sourceFinalTangent);
    }

    @Override
    public double getHeadingSecondDerivative(double s, double dKappa, Vector finalTangent) {
        return source.getHeadingSecondDerivative(pathLength - s, dKappa,
                sourceFinalTangent);
    }

    @Override
    public void setPathLength(double lengthInches) { this.pathLength = lengthInches; }
}
