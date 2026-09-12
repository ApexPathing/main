package tuning.localizer.phases;

import tuning.localizer.LocalizationTunerContext;
import tuning.localizer.LocalizerAdapter;
import tuning.localizer.TuningPhase;

import geometry.Pose;

/** Compact evidence gathered during filter tuning. */
final class FilterMetrics {
    private double rawSquared;
    private double filteredSquared;
    private double stopSquared;
    private int stationarySamples;
    private int stopSamples;
    private int invalidSamples;

    void sample(Pose raw, Pose filtered, boolean stationary, boolean stopping, boolean valid) {
        if (!valid) {
            invalidSamples++;
            return;
        }
        double rawMagnitude = magnitude(raw);
        double filteredMagnitude = magnitude(filtered);
        if (stationary) {
            rawSquared += rawMagnitude * rawMagnitude;
            filteredSquared += filteredMagnitude * filteredMagnitude;
            stationarySamples++;
        }
        if (stopping) { stopSquared += filteredMagnitude * filteredMagnitude; stopSamples++; }
    }

    double rawStationaryRms() { return rms(rawSquared, stationarySamples); }
    double filteredStationaryRms() { return rms(filteredSquared, stationarySamples); }
    double stopResidualRms() { return rms(stopSquared, stopSamples); }
    int getStationarySamples() { return stationarySamples; }
    int getStopSamples() { return stopSamples; }
    int getInvalidSamples() { return invalidSamples; }

    private static double magnitude(Pose value) {
        double x = value.getX().getIn(), y = value.getY().getIn();
        double heading = value.getHeading().getRad();
        return Math.sqrt(x * x + y * y + heading * heading);
    }
    private static double rms(double sum, int count) { return count == 0 ? 0.0 : Math.sqrt(sum / count); }
}
