package tuning.localizer;

import org.json.JSONObject;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** Proposed geometry values and concise review metrics produced by one calibration procedure. */
public final class CalibrationCandidate {
    private final JSONObject values;
    private final List<String> metrics;

    public CalibrationCandidate(JSONObject values, List<String> metrics) {
        this.values = values;
        this.metrics = Collections.unmodifiableList(new ArrayList<>(metrics));
    }

    /** Returns a defensive copy of the proposed geometry values. */
    public JSONObject getValues() {
        try { return new JSONObject(values.toString()); }
        catch (org.json.JSONException e) {
            throw new IllegalStateException("Could not copy calibration values", e);
        }
    }

    /** Returns human-readable evidence for the result screen. */
    public List<String> getMetrics() { return metrics; }
}
