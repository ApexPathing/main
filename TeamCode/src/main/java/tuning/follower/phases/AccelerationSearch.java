package tuning.follower.phases;

/** Bounded kA search: signed tracking bias chooses the half; RMS retains the best trial. */
final class AccelerationSearch {
    private final int rounds;
    private double lower;
    private double upper;
    private int completed;
    private double current;
    private double bestCandidate = Double.NaN;
    private double bestRms = Double.POSITIVE_INFINITY;

    AccelerationSearch(double upper, int rounds) {
        if (!Double.isFinite(upper) || upper <= 0.0 || rounds <= 0) {
            throw new IllegalArgumentException("kA search bounds and rounds must be positive");
        }
        this.upper = upper;
        this.rounds = rounds;
        current = upper * 0.5;
    }

    double current() { return current; }
    int completed() { return completed; }
    int rounds() { return rounds; }
    boolean isComplete() { return completed >= rounds; }
    double bestCandidate() { return bestCandidate; }
    double bestRms() { return bestRms; }

    void record(double rms, double signedMeanError) {
        if (isComplete() || !Double.isFinite(rms) || rms < 0.0 ||
                !Double.isFinite(signedMeanError)) {
            throw new IllegalArgumentException("Invalid kA search result");
        }
        if (rms < bestRms) {
            bestRms = rms;
            bestCandidate = current;
        }
        if (signedMeanError > 0.0) { lower = current; }
        else { upper = current; }
        completed++;
        current = (lower + upper) * 0.5;
    }
}
