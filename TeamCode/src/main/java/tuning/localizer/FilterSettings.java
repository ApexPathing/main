package tuning.localizer;

import localizers.BaseLocalizer;
import localizers.util.AdaptiveKalmanFilter;

/** Reversible snapshot of every public velocity-filter setting. */
final class FilterSettings {
    final BaseLocalizer.VelocityFilterMode mode;
    final int window;
    final AdaptiveKalmanFilter.KalmanTuning x;
    final AdaptiveKalmanFilter.KalmanTuning y;
    final AdaptiveKalmanFilter.KalmanTuning heading;

    private FilterSettings(BaseLocalizer<?> localizer) {
        mode = localizer.getVelocityFilterMode();
        window = localizer.getFilterWindowSize();
        x = localizer.getXKalmanTuning();
        y = localizer.getYKalmanTuning();
        heading = localizer.getHeadingKalmanTuning();
    }

    static FilterSettings capture(BaseLocalizer<?> localizer) { return new FilterSettings(localizer); }

    void apply(BaseLocalizer<?> localizer) {
        localizer.setVelocityFilterMode(mode);
        if (mode == BaseLocalizer.VelocityFilterMode.MOVING_AVERAGE) {
            localizer.setFilterWindow(window);
        } else {
            localizer.setXKalmanTuning(x);
            localizer.setYKalmanTuning(y);
            localizer.setHeadingKalmanTuning(heading);
        }
        localizer.setKalmanAutoTuning(false, false, false);
    }
}
