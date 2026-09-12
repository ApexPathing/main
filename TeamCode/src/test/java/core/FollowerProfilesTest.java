package core;

import drivetrains.BaseDrivetrain.DrivetrainType;
import org.json.JSONObject;
import org.junit.Test;
import static org.junit.Assert.*;
import static core.FollowerConstants.Profile.*;

public class FollowerProfilesTest {
    private FollowerConstants legacy(String type) throws Exception {
        return FollowerConstants.fromJson(new JSONObject().put("drivetrainType", type)
                .put("translationKV", 0.12).put("headingP", 0.8));
    }

    @Test public void legacyDualRequiresExplicitAssignmentAndPreservesOtherMode() throws Exception {
        FollowerConstants c = legacy("DUAL_ACTUATED");
        assertTrue(c.hasUnassignedLegacy());
        assertThrows(IllegalStateException.class, () -> c.configure(DrivetrainType.DUAL_ACTUATED, TANK, false));
        c.configure(DrivetrainType.DUAL_ACTUATED, TANK, true);
        assertEquals(0, c.translationalKV, 0);
        c.assignLegacyToActiveProfile();
        assertEquals(0.12, c.translationalKV, 0);
        c.selectProfile(HOLONOMIC);
        assertEquals(0, c.translationalKV, 0);
        c.translationalKV = 0.3;
        c.angularCoeffs.kP = 1.7;
        c.strafeVelLimitIn = 35;
        FollowerConstants restored = FollowerConstants.fromJson(c.toJson());
        restored.configure(DrivetrainType.DUAL_ACTUATED, TANK, false);
        assertEquals(0.12, restored.translationalKV, 0);
        assertEquals(0.8, restored.angularCoeffs.kP, 0);
        assertFalse(restored.requiresStrafeLimits());
        restored.selectProfile(HOLONOMIC);
        assertEquals(0.3, restored.translationalKV, 0);
        assertEquals(1.7, restored.angularCoeffs.kP, 0);
        assertEquals(35, restored.strafeVelLimitIn, 0);
        assertTrue(restored.requiresStrafeLimits());
    }

    @Test public void detachedGenerationDoesNotChangeActiveValues() throws Exception {
        FollowerConstants c = legacy("DUAL_ACTUATED");
        c.configure(DrivetrainType.DUAL_ACTUATED, TANK, true);
        c.forwardVelLimitIn = 20;
        c.selectProfile(HOLONOMIC);
        c.forwardVelLimitIn = 60;
        FollowerConstants tank = c.forProfile(TANK);
        assertEquals(20, tank.forwardVelLimitIn, 0);
        tank.angularCoeffs.kP = 99;
        assertEquals(HOLONOMIC, c.getActiveProfile());
        assertEquals(60, c.forwardVelLimitIn, 0);
        assertEquals(0, c.angularCoeffs.kP, 0);
        c.selectProfile(TANK);
        assertEquals(0, c.angularCoeffs.kP, 0);
    }

    @Test public void singleModeLegacyMigratesAndUnassignedLegacySurvivesSaving() throws Exception {
        FollowerConstants single = legacy("MECANUM");
        FollowerConstants restored = FollowerConstants.fromJson(single.toJson());
        assertEquals(DEFAULT, restored.getActiveProfile());
        assertEquals(0.12, restored.translationalKV, 0);
        FollowerConstants dual = FollowerConstants.fromJson(legacy("DUAL_ACTUATED").toJson());
        assertTrue(dual.hasUnassignedLegacy());
        assertThrows(IllegalStateException.class, () -> dual.forProfile(TANK));
    }

    @Test public void rejectsUnknownSchemaAndInvalidProfileAndNonfiniteSave() throws Exception {
        assertThrows(IllegalArgumentException.class, () -> FollowerConstants.fromJson(
                new JSONObject().put("schemaVersion", 99).put("drivetrainType", "DUAL_ACTUATED")));
        FollowerConstants c = legacy("DUAL_ACTUATED");
        assertThrows(IllegalArgumentException.class, () -> c.selectProfile(DEFAULT));
        c.configure(DrivetrainType.DUAL_ACTUATED, TANK, true);
        c.angularKV = Double.NaN;
        assertThrows(IllegalStateException.class, c::toJson);
    }
}
