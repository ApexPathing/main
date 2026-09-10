package tuning.localizer;

/** Start/end observations and independently measured signed rotation for one spin. */
public final class SpinTrial {
    public final CalibrationSnapshot start;
    public final CalibrationSnapshot end;
    public final double angleRad;
    public final double reportedAngleRad;

    public SpinTrial(CalibrationSnapshot start, CalibrationSnapshot end, double angleRad) {
        this(start, end, angleRad, AngleDelta.between(start.pose, end.pose));
    }

    public SpinTrial(CalibrationSnapshot start, CalibrationSnapshot end, double angleRad,
                     double reportedAngleRad) {
        this.start = start;
        this.end = end;
        this.angleRad = angleRad;
        this.reportedAngleRad = reportedAngleRad;
    }

    private static final class AngleDelta {
        static double between(geometry.Pose start, geometry.Pose end) {
            return geometry.Angle.wrap(end.getHeading().getRad() - start.getHeading().getRad());
        }
    }
}
