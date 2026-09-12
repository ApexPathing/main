package tuning.localizer.phases;

import tuning.localizer.LocalizationTunerContext;
import tuning.localizer.LocalizerAdapter;
import tuning.localizer.TuningPhase;

import geometry.Pose;

/** Immutable pose and raw-channel observation captured at a trial boundary. */
public final class CalibrationSnapshot {
    public final Pose pose;
    public final int[] rawTicks;

    public CalibrationSnapshot(Pose pose, int[] rawTicks) {
        this.pose = pose;
        this.rawTicks = rawTicks == null ? new int[0] : rawTicks.clone();
    }
}
