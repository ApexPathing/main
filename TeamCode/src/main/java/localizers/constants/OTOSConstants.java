package localizers.constants;

import com.qualcomm.robotcore.hardware.HardwareMap;

import localizers.OTOS;
import util.Angle;
import util.Distance;
import util.Pose;

/**
 * OTOS localizer constants class
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class OTOSConstants extends LocalizerConstants {
    // Hardware
    public String name = "sensor_otos";

    // The pose of the OTOS relative to the robot center (x, y, heading)
    public Pose offset = new Pose(0, 0, 0, Distance.Units.INCHES, Angle.Units.DEGREES);

    // Scalars
    public double linearScalar = 1.0;
    public double headingScalar = 1.0;

     /**
     * Constructor for the OTOSConstants class
     */
    public OTOSConstants() {}

    @Override
    public OTOS build(HardwareMap hardwareMap) {
        return new OTOS(hardwareMap, this);
    }

    /**
     * Sets the OTOS hardware name.
     * @param name the hardware map name of the OTOS
     * @return this instance for chaining
     */
    public OTOSConstants setName(String name) {
        this.name = name;
        return this;
    }

    /**
     * Sets the offset of the OTOS relative to the robot center.
     * @param offset the pose of the OTOS relative to the robot center
     * @return this instance for chaining
     */
    public OTOSConstants setOffset(Pose offset) {
        this.offset = offset;
        return this;
    }

    /**
     * Sets the linear scalar for the OTOS.
     * @param linearScalar the scalar to apply to translational measurements
     * @return this instance for chaining
     */
    public OTOSConstants setLinearScalar(double linearScalar) {
        this.linearScalar = linearScalar;
        return this;
    }

    /**
     * Sets the heading scalar for the OTOS.
     * @param headingScalar the scalar to apply to heading measurements
     * @return this instance for chaining
     */
    public OTOSConstants setHeadingScalar(double headingScalar) {
        this.headingScalar = headingScalar;
        return this;
    }
}
