package Localizers.Constants;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pinpoint localizer constants class
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class PinpointConstants {
    // Hardware
    public String name = "pinpoint";

    // Unites
    public DistanceUnit distanceUnit = DistanceUnit.INCH;
    public AngleUnit angleUnit = AngleUnit.DEGREES;

    // Pod offsets
    public double xOffset = -3.31; // In distanceUnit
    public double yOffset = -6.61; // In distanceUnit

    // Pod directions
    public GoBildaPinpointDriver.EncoderDirection xPodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public GoBildaPinpointDriver.EncoderDirection yPodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // Encoder resolution settings
    public GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public double customEncoderResolution = 0; // In distanceUnit (Overrides encoderResolution if != 0)

    public double yawScalar = 0; // Overrides the default Pinpoint yaw scalar if != 0 (NOT RECOMMENDED TO CHANGE)


    /**
     * Constructor for the PinpointConstants class
     * Default constants are derived from the SensorGoBildaPinpoint SDK sample with untuned values
     */
    public PinpointConstants() {}

    /**
     * Sets the pinpoint hardware name.
     * @param name the hardware map name of the pinpoint
     * @return this instance for chaining
     */
    public PinpointConstants setName(String name) {
        this.name = name;
        return this;
    }

    /**
     * Sets the distance unit used by the pinpoint.
     * @param distanceUnit the distance unit
     * @return this instance for chaining
     */
    public PinpointConstants setDistanceUnit(DistanceUnit distanceUnit) {
        this.distanceUnit = distanceUnit;
        return this;
    }

    /**
     * Sets the angle unit used by the pinpoint.
     * @param angleUnit the angle unit
     * @return this instance for chaining
     */
    public PinpointConstants setAngleUnit(AngleUnit angleUnit) {
        this.angleUnit = angleUnit;
        return this;
    }

    /**
     * Sets the X offset of the pods.
     * @param xOffset the X offset in distanceUnit
     * @return this instance for chaining
     */
    public PinpointConstants setXOffset(double xOffset) {
        this.xOffset = xOffset;
        return this;
    }

    /**
     * Sets the Y offset of the pods.
     * @param yOffset the Y offset in distanceUnit
     * @return this instance for chaining
     */
    public PinpointConstants setYOffset(double yOffset) {
        this.yOffset = yOffset;
        return this;
    }

    /**
     * Sets the X pod encoder direction.
     * @param xPodDirection the direction of the X pod
     * @return this instance for chaining
     */
    public PinpointConstants setXPodDirection(GoBildaPinpointDriver.EncoderDirection xPodDirection) {
        this.xPodDirection = xPodDirection;
        return this;
    }

    /**
     * Sets the Y pod encoder direction.
     * @param yPodDirection the direction of the Y pod
     * @return this instance for chaining
     */
    public PinpointConstants setYPodDirection(GoBildaPinpointDriver.EncoderDirection yPodDirection) {
        this.yPodDirection = yPodDirection;
        return this;
    }

    /**
     * Sets the encoder resolution model.
     * @param encoderResolution the odometry pod model used
     * @return this instance for chaining
     */
    public PinpointConstants setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution) {
        this.encoderResolution = encoderResolution;
        return this;
    }

    /**
     * Sets a custom encoder resolution, overriding the model selection if is not 0.
     * @param customEncoderResolution the custom resolution in ticks per mm
     * @return this instance for chaining
     */
    public PinpointConstants setCustomEncoderResolution(double customEncoderResolution) {
        this.customEncoderResolution = customEncoderResolution;
        return this;
    }

    /**
    * Override your Pinpoint's tuned scalar if not 0. (NOT RECOMMENDED TO CHANGE)
    * @param yawScalar the multiplier for the yaw reading
    * @return this instance for chaining
    */
    public PinpointConstants setYawScalar(double yawScalar) {
        this.yawScalar = yawScalar;
        return this;
    }
}
