package core;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

import localizers.BaseLocalizer;
import localizers.BaseLocalizerConstants;
import localizers.util.AdaptiveKalmanFilter;

/** Persisted geometry and velocity-filter calibration for one robot configuration. */
public final class LocalizationConstants {
    public static final int SCHEMA_VERSION = 1;

    /** Validation state recorded independently for geometry and filters. */
    public enum Status { UNCALIBRATED, ACCEPTED, NEEDS_VALIDATION }

    private final JSONObject setups;
    private boolean followerRevalidationRequired;

    private LocalizationConstants(JSONObject setups, boolean followerRevalidationRequired) {
        this.setups = setups;
        this.followerRevalidationRequired = followerRevalidationRequired;
    }

    /** Loads persisted calibration. Invalid or unsupported files produce an empty calibration. */
    public static LocalizationConstants load() {
        File file = ApexStorage.getReadableLocalizationFile();
        if (!file.exists()) { return empty(); }
        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            StringBuilder text = new StringBuilder();
            String line;
            while ((line = reader.readLine()) != null) { text.append(line); }
            JSONObject root = new JSONObject(text.toString());
            if (root.optInt("schemaVersion", -1) != SCHEMA_VERSION) { return empty(); }
            JSONObject setups = root.optJSONObject("setups");
            return new LocalizationConstants(setups == null ? new JSONObject() : setups,
                    root.optBoolean("followerRevalidationRequired", false));
        } catch (Exception ignored) {
            return empty();
        }
    }

    /** Creates an empty in-memory calibration. */
    public static LocalizationConstants empty() {
        return new LocalizationConstants(new JSONObject(), false);
    }

    /** Applies compatible saved geometry before hardware construction. */
    public boolean applyGeometry(String setupName, BaseLocalizerConstants<?> config) {
        JSONObject setup = compatibleSetup(setupName, config);
        if (setup == null) { return false; }
        JSONObject geometry = setup.optJSONObject("geometry");
        if (geometry == null) { return false; }
        config.applyCalibrationValues(geometry);
        return true;
    }

    /** Applies compatible filter settings after hardware construction. */
    public boolean applyFilter(String setupName, BaseLocalizerConstants<?> config,
                               BaseLocalizer<?> localizer) {
        JSONObject setup = compatibleSetup(setupName, config);
        JSONObject filter = setup == null ? null : setup.optJSONObject("filter");
        if (filter == null) { return false; }
        try {
            BaseLocalizer.VelocityFilterMode mode = BaseLocalizer.VelocityFilterMode.valueOf(
                    filter.getString("mode"));
            localizer.setVelocityFilterMode(mode);
            if (mode == BaseLocalizer.VelocityFilterMode.MOVING_AVERAGE) {
                localizer.setFilterWindow(Math.max(1, filter.getInt("window")));
            } else {
                localizer.setXKalmanTuning(readTuning(filter.getJSONObject("x")));
                localizer.setYKalmanTuning(readTuning(filter.getJSONObject("y")));
                localizer.setHeadingKalmanTuning(readTuning(filter.getJSONObject("heading")));
            }
            return true;
        } catch (Exception ignored) {
            return false;
        }
    }

    /** Captures current geometry and filter values after the operator accepts a result. */
    public void capture(String setupName, BaseLocalizerConstants<?> config,
                        BaseLocalizer<?> localizer, Status geometryStatus, Status filterStatus) {
        JSONObject setup = setup(setupName);
        try {
            setup.put("localizerType", config.getCalibrationType());
            setup.put("geometry", config.getCalibrationValues());
            setup.put("geometryStatus", geometryStatus.name());
            setup.put("filterStatus", filterStatus.name());
            JSONObject filter = new JSONObject();
            filter.put("mode", localizer.getVelocityFilterMode().name());
            filter.put("window", localizer.getFilterWindowSize());
            if (localizer.getVelocityFilterMode() == BaseLocalizer.VelocityFilterMode.ADAPTIVE_KALMAN) {
                filter.put("x", tuning(localizer.getXKalmanTuning()));
                filter.put("y", tuning(localizer.getYKalmanTuning()));
                filter.put("heading", tuning(localizer.getHeadingKalmanTuning()));
            }
            setup.put("filter", filter);
            followerRevalidationRequired = true;
        } catch (Exception e) {
            throw new IllegalStateException("Could not capture localization calibration", e);
        }
    }

    /** Marks filter validation stale after accepted geometry changes. */
    public void invalidateFilter(String setupName) {
        try { setup(setupName).put("filterStatus", Status.NEEDS_VALIDATION.name()); }
        catch (Exception e) { throw new IllegalStateException(e); }
    }

    /** Returns a saved status, defaulting to uncalibrated. */
    public Status getStatus(String setupName, boolean filter) {
        try {
            return Status.valueOf(setup(setupName).optString(
                    filter ? "filterStatus" : "geometryStatus", Status.UNCALIBRATED.name()));
        } catch (Exception ignored) { return Status.UNCALIBRATED; }
    }

    /** Marks one independent geometry procedure accepted for menu progress reporting. */
    public void markGeometryStepAccepted(String setupName, String step) {
        try {
            JSONObject setup = setup(setupName);
            JSONObject steps = setup.optJSONObject("geometrySteps");
            if (steps == null) { steps = new JSONObject(); setup.put("geometrySteps", steps); }
            steps.put(step, Status.ACCEPTED.name());
        } catch (Exception e) { throw new IllegalStateException(e); }
    }

    /** Returns the acceptance state of an individual geometry procedure. */
    public Status getGeometryStepStatus(String setupName, String step) {
        try {
            JSONObject steps = setup(setupName).optJSONObject("geometrySteps");
            return steps == null ? Status.UNCALIBRATED
                    : Status.valueOf(steps.optString(step, Status.UNCALIBRATED.name()));
        } catch (Exception ignored) { return Status.UNCALIBRATED; }
    }

    /** Returns whether follower tuning should be checked after localization changed. */
    public boolean isFollowerRevalidationRequired() { return followerRevalidationRequired; }

    /** Serializes the complete calibration file. */
    public JSONObject toJson() {
        try {
            JSONObject root = new JSONObject().put("schemaVersion", SCHEMA_VERSION)
                    .put("followerRevalidationRequired", followerRevalidationRequired)
                    .put("setups", setups);
            return new JSONObject(root.toString());
        } catch (Exception e) { throw new IllegalStateException(e); }
    }

    /** Restores an earlier snapshot after a failed transactional save. */
    public void restore(JSONObject snapshot) {
        try {
            JSONObject restored = snapshot.getJSONObject("setups");
            org.json.JSONArray oldNames = setups.names();
            if (oldNames != null) {
                for (int i = 0; i < oldNames.length(); i++) { setups.remove(oldNames.getString(i)); }
            }
            org.json.JSONArray names = restored.names();
            if (names != null) {
                for (int i = 0; i < names.length(); i++) {
                    String name = names.getString(i);
                    setups.put(name, restored.get(name));
                }
            }
            followerRevalidationRequired = snapshot.optBoolean(
                    "followerRevalidationRequired", false);
        } catch (Exception e) { throw new IllegalArgumentException("Invalid calibration snapshot", e); }
    }

    /** Saves through ApexStorage's backup-backed replacement. */
    public void save() throws java.io.IOException {
        try {
            ApexStorage.saveLocalization(toJson().toString(4));
        } catch (org.json.JSONException e) {
            throw new java.io.IOException("Could not serialize localization calibration", e);
        }
    }

    private JSONObject compatibleSetup(String name, BaseLocalizerConstants<?> config) {
        JSONObject setup = setups.optJSONObject(name);
        return setup != null && config.getCalibrationType().equals(
                setup.optString("localizerType")) ? setup : null;
    }

    private JSONObject setup(String name) {
        JSONObject setup = setups.optJSONObject(name);
        if (setup != null) { return setup; }
        setup = new JSONObject();
        try { setups.put(name, setup); }
        catch (Exception e) { throw new IllegalStateException(e); }
        return setup;
    }

    private static JSONObject tuning(AdaptiveKalmanFilter.KalmanTuning value) throws Exception {
        return new JSONObject().put("measurementVariance", value.measurementVariance)
                .put("processVariance", value.processVariance);
    }

    private static AdaptiveKalmanFilter.KalmanTuning readTuning(JSONObject value) throws Exception {
        double measurement = value.getDouble("measurementVariance");
        double process = value.getDouble("processVariance");
        if (!Double.isFinite(measurement) || measurement <= 0
                || !Double.isFinite(process) || process <= 0) {
            throw new IllegalArgumentException("Kalman variances must be finite and positive");
        }
        return new AdaptiveKalmanFilter.KalmanTuning(measurement, process);
    }
}
