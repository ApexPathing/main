package feedforward.generators;

import core.FollowerConstants;
import org.json.JSONObject;
import org.junit.Test;
import static org.junit.Assert.assertEquals;

public class TankTractionLimitTest {
    @Test public void bothCurveDirectionsHaveTheSameTractionLimit() {
        assertEquals(10, TankProfileGenerator.tractionLimitedVelocity(60, .2, 20), 1e-9);
        assertEquals(10, TankProfileGenerator.tractionLimitedVelocity(60, -.2, 20), 1e-9);
    }

    @Test public void unsetLimitAndStraightPathsRemainUsable() {
        assertEquals(60, TankProfileGenerator.tractionLimitedVelocity(60, .2, 0), 0);
        assertEquals(60, TankProfileGenerator.tractionLimitedVelocity(60, 0, 20), 0);
        assertEquals(5, TankProfileGenerator.tractionLimitedVelocity(5, .2, 20), 0);
    }

    @Test public void tractionLimitSurvivesDualProfileSwitchAndSerialization() throws Exception {
        FollowerConstants constants = FollowerConstants.fromJson(new JSONObject()
                .put("schemaVersion", 2).put("drivetrainType", "DUAL_ACTUATED")
                .put("profiles", new JSONObject()
                        .put("TANK", new JSONObject().put("maxCentripetalAccelIn", 20))
                        .put("HOLONOMIC", new JSONObject())));
        constants.selectProfile(FollowerConstants.Profile.TANK);
        assertEquals(20, constants.maxCentripetalAccelIn, 0);
        constants.selectProfile(FollowerConstants.Profile.HOLONOMIC);
        assertEquals(0, constants.maxCentripetalAccelIn, 0);
        FollowerConstants reloaded = FollowerConstants.fromJson(constants.toJson());
        assertEquals(20, reloaded.forProfile(FollowerConstants.Profile.TANK)
                .maxCentripetalAccelIn, 0);
    }
}
