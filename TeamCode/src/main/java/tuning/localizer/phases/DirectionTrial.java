package tuning.localizer.phases;

import tuning.localizer.LocalizationTunerContext;
import tuning.localizer.LocalizerAdapter;
import tuning.localizer.TuningPhase;

/** Raw localizer observations captured around one commanded translation pulse. */
public final class DirectionTrial {
    public final CalibrationAxis axis;
    public final CalibrationSnapshot start;
    public final CalibrationSnapshot end;

    public DirectionTrial(CalibrationAxis axis, CalibrationSnapshot start,
                          CalibrationSnapshot end) {
        this.axis = axis;
        this.start = start;
        this.end = end;
    }
}
