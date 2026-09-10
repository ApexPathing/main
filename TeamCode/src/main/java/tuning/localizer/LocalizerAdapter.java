package tuning.localizer;

import java.util.List;

import localizers.BaseLocalizer;
import localizers.BaseLocalizerConstants;

/**
 * Supplies the small amount of localizer-specific observation and fitting needed by the tuner.
 * Unknown custom localizers use the monitor-only adapter returned by {@link LocalizerAdapters}.
 */
public interface LocalizerAdapter {
    /** Display name of the configured localizer. */
    String getName();

    /** Returns whether a measured-distance trial can identify this axis. */
    boolean supportsDistance(CalibrationAxis axis);

    /** Returns whether rotation can identify offsets or heading scale. */
    boolean supportsSpinCalibration();

    /** Returns a trial-boundary snapshot after the context has updated the localizer. */
    CalibrationSnapshot snapshot(BaseLocalizer<?> localizer);

    /** Fits geometry from measured translation without mutating the active configuration. */
    CalibrationCandidate fitDistance(BaseLocalizerConstants<?> config, CalibrationAxis axis,
                                     CalibrationSnapshot start, CalibrationSnapshot end,
                                     double measuredInches);

    /** Fits geometry from paired clockwise/counterclockwise rotation trials. */
    CalibrationCandidate fitSpin(BaseLocalizerConstants<?> config, List<SpinTrial> trials);
}
