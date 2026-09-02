package controllers;

import static org.junit.Assert.assertEquals;
import org.junit.Test;
import controllers.PDSController.PDSCoefficients;

/** Verifies static-friction application around the endpoint deadband. */
public class PDSControllerTest {
    @Test
    public void linearStaticPowerIsFullOutsideQuarterInchDeadband() {
        PDSController controller = new PDSController(new PDSCoefficients(0.0, 0.0, 0.3));

        assertEquals(0.3, controller.calculate(0.26), 1e-9);
        assertEquals(0.0, controller.calculate(0.24), 1e-9);
        assertEquals(-0.3, controller.calculate(-0.26), 1e-9);
    }

    @Test
    public void angularStaticPowerIsFullOutsideThreeQuarterDegreeDeadband() {
        PDSController controller = new PDSController(new PDSCoefficients(0.0, 0.0, 0.3));
        controller.setAngularController();

        assertEquals(0.3, controller.calculate(Math.toRadians(0.76)), 1e-9);
        assertEquals(0.0, controller.calculate(Math.toRadians(0.74)), 1e-9);
        assertEquals(0.0, controller.calculate(Math.toRadians(0.75)), 1e-9);
        assertEquals(-0.3, controller.calculate(Math.toRadians(-0.76)), 1e-9);
        assertEquals(0.0, controller.calculate(Math.toRadians(-0.75)), 1e-9);
    }

    @Test
    public void proportionalControlStillWorksInsideDeadband() {
        PDSController controller = new PDSController(new PDSCoefficients(2.0, 0.0, 0.3));

        assertEquals(0.2, controller.calculate(0.1), 1e-9);
    }
}
