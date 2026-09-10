package tuning.localizer;

import geometry.Pose;

/** Compact evidence gathered during filter tuning or validation. */
final class FilterMetrics {
    private double rawSquared;
    private double filteredSquared;
    private double stopSquared;
    private int stationarySamples;
    private int stopSamples;
    private int validSamples;
    private int invalidSamples;
    private double peakRaw;
    private double peakFiltered;

    void sample(Pose raw, Pose filtered, boolean stationary, boolean stopping, boolean valid) {
        if (valid) { validSamples++; } else { invalidSamples++; }
        double rawMagnitude = magnitude(raw);
        double filteredMagnitude = magnitude(filtered);
        peakRaw = Math.max(peakRaw, rawMagnitude);
        peakFiltered = Math.max(peakFiltered, filteredMagnitude);
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
    int getValidSamples() { return validSamples; }
    int getInvalidSamples() { return invalidSamples; }
    double getPeakRaw() { return peakRaw; }
    double getPeakFiltered() { return peakFiltered; }

    private static double magnitude(Pose value) {
        double x = value.getX().getIn(), y = value.getY().getIn();
        double heading = value.getHeading().getRad();
        return Math.sqrt(x * x + y * y + heading * heading);
    }
    private static double rms(double sum, int count) { return count == 0 ? 0.0 : Math.sqrt(sum / count); }
}
