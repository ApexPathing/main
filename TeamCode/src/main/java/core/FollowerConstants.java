package core;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

import drivetrains.BaseDrivetrain;
import controllers.PDSController.PDSCoefficients;

/**
 * Class to hold constants for the Follower class. These constants are loaded from a JSON file
 * created by the tuners. If the file does not exist or cannot be read, default values will be used.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 * @author DrPixelCat - 7842 alum
 */
public class FollowerConstants {
    /**
     * Note to developers:
     * If you want to add new constants, create the variable here and add it to the loadValues() and
     * valuesToJson() methods. This will ensure that the new constants are loaded from the JSON file and
     * saved back to it.
     */
    private static FollowerConstants instance;
    public BaseDrivetrain.DrivetrainType drivetrainType =
            BaseDrivetrain.DrivetrainType.MECANUM;
    public PDSCoefficients angularCoeffs = new PDSCoefficients();
    public PDSCoefficients translationalCoeffs = new PDSCoefficients();

    public double velocityFeedbackGain = 0.0;
    public double angularVelocityFeedbackGain = 0.0;
    public double translationalKV = 0.0, translationalKA = 0.0;
    public double angularKV = 0.0, angularKA = 0.0;
    /** Moving-friction feedforward terms; PDS kS remains the larger breakaway value. */
    public double translationalFeedforwardKS = 0.0, angularFeedforwardKS = 0.0;
    public double kCentripetal = 0.0;

    public double forwardVelLimitIn = 0.0;
    public double forwardAccelLimitIn = 0.0;
    public double strafeVelLimitIn = 0.0;
    public double strafeAccelLimitIn = 0.0;
    public double angularVelLimitRad = 0.0;
    public double angularAccelLimitRad = 0.0;

    public enum Profile { DEFAULT, TANK, HOLONOMIC }
    private final java.util.EnumMap<Profile, JSONObject> profiles =
            new java.util.EnumMap<>(Profile.class);
    private Profile activeProfile = Profile.DEFAULT;
    private JSONObject unassignedLegacy;
    private boolean allowUntuned;

    private FollowerConstants() { reload(); }
    private FollowerConstants(boolean empty) { }

    public Profile getActiveProfile() { return activeProfile; }
    public boolean hasUnassignedLegacy() { return unassignedLegacy != null; }
    public boolean requiresStrafeLimits() {
        return drivetrainType != BaseDrivetrain.DrivetrainType.TANK
                && activeProfile != Profile.TANK;
    }

    /** Select hardware without guessing which mode produced a legacy dual calibration. */
    public void configure(BaseDrivetrain.DrivetrainType type, Profile profile, boolean tuning) {
        allowUntuned = tuning;
        if (type != drivetrainType) {
            profiles.clear();
            unassignedLegacy = null;
            activeProfile = null;
            loadValues(new JSONObject());
        }
        drivetrainType = type;
        selectProfile(type == BaseDrivetrain.DrivetrainType.DUAL_ACTUATED ? profile : Profile.DEFAULT);
    }

    private void validateProfile(Profile profile) {
        if (profile == null || (drivetrainType == BaseDrivetrain.DrivetrainType.DUAL_ACTUATED
                && profile == Profile.DEFAULT)) {
            throw new IllegalArgumentException("Dual-actuated tuning requires TANK or HOLONOMIC");
        }
    }

    public void selectProfile(Profile profile) {
        validateProfile(profile);
        if (profile == activeProfile) { return; }
        if (!profiles.containsKey(profile) && !allowUntuned
                && drivetrainType == BaseDrivetrain.DrivetrainType.DUAL_ACTUATED) {
            throw new IllegalStateException("No saved follower profile for " + profile
                    + ". Run Follower Tuner for this mode first.");
        }
        if (activeProfile != null) { profiles.put(activeProfile, valuesToJson()); }
        loadValues(profiles.containsKey(profile) ? profiles.get(profile) : new JSONObject());
        activeProfile = profile;
    }

    /** Detached values for path generation; does not change the active drivetrain mode. */
    public FollowerConstants forProfile(Profile profile) {
        if (drivetrainType != BaseDrivetrain.DrivetrainType.DUAL_ACTUATED) { return this; }
        validateProfile(profile);
        if (profile != activeProfile && !profiles.containsKey(profile) && !allowUntuned) {
            throw new IllegalStateException("No saved follower profile for " + profile);
        }
        FollowerConstants result = new FollowerConstants(true);
        result.drivetrainType = drivetrainType;
        result.activeProfile = profile;
        result.loadValues(profile == activeProfile ? valuesToJson()
                : profiles.containsKey(profile) ? profiles.get(profile) : new JSONObject());
        return result;
    }

    /** Explicit operator assignment; legacy data is never automatically copied to both modes. */
    public void assignLegacyToActiveProfile() {
        if (unassignedLegacy == null || activeProfile == null) { return; }
        loadValues(unassignedLegacy);
        profiles.put(activeProfile, valuesToJson());
        unassignedLegacy = null;
    }

    public static FollowerConstants fromJson(JSONObject json) {
        FollowerConstants result = new FollowerConstants(true);
        result.readJson(json);
        return result;
    }

    private void readJson(JSONObject json) {
        int version = json.optInt("schemaVersion", 1);
        if (version != 1 && version != 2) {
            throw new IllegalArgumentException("Unsupported follower constants schema: " + version);
        }
        BaseDrivetrain.DrivetrainType type;
        try { type = BaseDrivetrain.DrivetrainType.valueOf(json.getString("drivetrainType")); }
        catch (Exception e) { throw new IllegalArgumentException("Invalid drivetrainType in follower constants", e); }
        profiles.clear();
        drivetrainType = type;
        unassignedLegacy = json.optJSONObject("unassignedLegacy");
        activeProfile = null;
        loadValues(new JSONObject());
        if (version == 1) {
            if (type == BaseDrivetrain.DrivetrainType.DUAL_ACTUATED) { unassignedLegacy = json; }
            else { activeProfile = Profile.DEFAULT; loadValues(json); }
        } else {
            JSONObject saved = json.optJSONObject("profiles");
            if (saved != null) {
                for (Profile profile : Profile.values()) {
                    JSONObject values = saved.optJSONObject(profile.name());
                    if (values != null) { profiles.put(profile, values); }
                }
            }
            if (type != BaseDrivetrain.DrivetrainType.DUAL_ACTUATED) {
                activeProfile = Profile.DEFAULT;
                loadValues(profiles.containsKey(Profile.DEFAULT)
                        ? profiles.get(Profile.DEFAULT) : new JSONObject());
            }
        }
    }

    public JSONObject toJson() {
        if (activeProfile != null) { profiles.put(activeProfile, valuesToJson()); }
        JSONObject json = new JSONObject();
        JSONObject saved = new JSONObject();
        try {
            json.put("schemaVersion", 2);
            json.put("drivetrainType", drivetrainType.name());
            for (Profile profile : profiles.keySet()) { saved.put(profile.name(), profiles.get(profile)); }
            json.put("profiles", saved);
            if (unassignedLegacy != null) { json.put("unassignedLegacy", unassignedLegacy); }
        } catch (org.json.JSONException e) { throw new IllegalStateException(e); }
        return json;
    }

    public static FollowerConstants getInstance() {
        if (instance == null) {
            instance = new FollowerConstants();
        }
        return instance;
    }

    private double loadDouble(JSONObject json, String key) {
        try {
            double value = json.getDouble(key);
            if (!Double.isFinite(value)) { throw new IllegalArgumentException("Non-finite " + key); }
            return value;
        } catch (Exception e) {
            return 0.0;
        }
    }

    public void reload() {
        File file = ApexStorage.getReadableConstantsFile();
        if (!file.exists()) { return; }

        JSONObject json;
        try {
            BufferedReader reader = new BufferedReader(new FileReader(file));
            StringBuilder sb = new StringBuilder();
            String line = reader.readLine();
            while (line != null) {
                sb.append(line);
                line = reader.readLine();
            }
            reader.close();
            json = new JSONObject(sb.toString());
        } catch (Exception e) {
            return;
        }

        readJson(json);
    }

    private void loadValues(JSONObject json) {
        angularCoeffs.setkP(loadDouble(json, "headingP"));
        angularCoeffs.setkD(loadDouble(json, "headingD"));
        angularCoeffs.setkS(loadDouble(json, "headingS"));

        translationalCoeffs.setkP(loadDouble(json, "translationalP"));
        translationalCoeffs.setkD(loadDouble(json, "translationalD"));
        translationalCoeffs.setkS(loadDouble(json, "translationalS"));

        translationalKV = loadDouble(json, "translationKV");
        translationalKA = loadDouble(json, "translationKA");
        angularKV = loadDouble(json, "angularKV");
        angularKA = loadDouble(json, "angularKA");
        angularFeedforwardKS = json.has("angularFeedforwardS")
                ? loadDouble(json, "angularFeedforwardS") : angularCoeffs.kS;
        translationalFeedforwardKS = json.has("translationalFeedforwardS")
                ? loadDouble(json, "translationalFeedforwardS") : translationalCoeffs.kS;
        velocityFeedbackGain = loadDouble(json, "velocityFeedbackGain");
        angularVelocityFeedbackGain = loadDouble(json, "angularVelocityFeedbackGain");
        kCentripetal = loadDouble(json, "kCentripetal");

        forwardVelLimitIn = loadDouble(json, "forwardVelLimitIn");
        forwardAccelLimitIn = loadDouble(json, "forwardAccelLimitIn");
        strafeVelLimitIn = loadDouble(json, "strafeVelLimitIn");
        strafeAccelLimitIn = loadDouble(json, "strafeAccelLimitIn");
        angularVelLimitRad = loadDouble(json, "angularVelLimitRad");
        angularAccelLimitRad = loadDouble(json, "angularAccelLimitRad");
    }

    private JSONObject valuesToJson() {
        JSONObject json = new JSONObject();
        try {
            json.put("headingP", angularCoeffs.kP);
            json.put("headingD", angularCoeffs.kD);
            json.put("headingS", angularCoeffs.kS);
            json.put("translationalP", translationalCoeffs.kP);
            json.put("translationalD", translationalCoeffs.kD);
            json.put("translationalS", translationalCoeffs.kS);
            json.put("translationKV", translationalKV);
            json.put("translationKA", translationalKA);
            json.put("angularKV", angularKV);
            json.put("angularKA", angularKA);
            json.put("angularFeedforwardS", angularFeedforwardKS);
            json.put("translationalFeedforwardS", translationalFeedforwardKS);
            json.put("velocityFeedbackGain", velocityFeedbackGain);
            json.put("angularVelocityFeedbackGain", angularVelocityFeedbackGain);
            json.put("kCentripetal", kCentripetal);
            json.put("forwardVelLimitIn", forwardVelLimitIn);
            json.put("forwardAccelLimitIn", forwardAccelLimitIn);
            json.put("strafeVelLimitIn", strafeVelLimitIn);
            json.put("strafeAccelLimitIn", strafeAccelLimitIn);
            json.put("angularVelLimitRad", angularVelLimitRad);
            json.put("angularAccelLimitRad", angularAccelLimitRad);
        } catch (Exception e) {
            throw new IllegalStateException("Cannot save non-finite follower constants", e);
        }
        return json;
    }

    /** The inertia coefficient applies to signed acceleration, including planned braking. */
    public double getTranslationalKA(double velocity, double acceleration) {
        return translationalKA;
    }
}
