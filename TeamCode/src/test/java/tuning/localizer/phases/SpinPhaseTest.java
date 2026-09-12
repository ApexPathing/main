package tuning.localizer.phases;

import static org.junit.Assert.assertEquals;
import org.junit.Test;

public class SpinPhaseTest {
    @Test
    public void discoveryRampOnlyRaisesAndClampsPower() {
        assertEquals(0.2 + SpinPhase.POWER_RAMP_PER_SECOND * 0.02,
                SpinPhase.rampTurnPower(0.2, 0.02), 1e-12);
        assertEquals(0.2, SpinPhase.rampTurnPower(0.2, -1.0), 0.0);
        assertEquals(0.5, SpinPhase.rampTurnPower(0.5, 1.0), 0.0);
    }
}
