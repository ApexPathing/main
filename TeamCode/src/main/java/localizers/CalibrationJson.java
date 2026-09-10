package localizers;

import org.json.JSONObject;

import geometry.Angle;
import geometry.DistUnit;
import geometry.Pose;
import geometry.Vector;

/** Package-private JSON helpers shared by built-in localizer configurations. */
final class CalibrationJson {
    private CalibrationJson() { }

    static JSONObject vector(Vector value) {
        try {
            return new JSONObject().put("xIn", value.getX().getIn())
                    .put("yIn", value.getY().getIn());
        } catch (Exception e) { throw new IllegalStateException(e); }
    }

    static Vector vector(JSONObject json, Vector fallback) {
        if (json == null) { return fallback; }
        double x = finite(json, "xIn", fallback.getX().getIn());
        double y = finite(json, "yIn", fallback.getY().getIn());
        return Vector.of(x, y, DistUnit.IN);
    }

    static JSONObject pose(Pose value) {
        try {
            return vector(value.getVec()).put("headingRad", value.getHeading().getRad());
        } catch (Exception e) { throw new IllegalStateException(e); }
    }

    static Pose pose(JSONObject json, Pose fallback) {
        if (json == null) { return fallback; }
        return new Pose(vector(json, fallback.getVec()), Angle.fromRad(
                finite(json, "headingRad", fallback.getHeading().getRad())));
    }

    static double finite(JSONObject json, String key, double fallback) {
        double value = json.optDouble(key, fallback);
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(key + " must be finite");
        }
        return value;
    }

    static double positive(JSONObject json, String key, double fallback) {
        double value = finite(json, key, fallback);
        if (value <= 0.0) { throw new IllegalArgumentException(key + " must be positive"); }
        return value;
    }
}
