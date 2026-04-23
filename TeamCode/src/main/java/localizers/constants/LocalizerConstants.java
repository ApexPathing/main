package localizers.constants;

import com.qualcomm.robotcore.hardware.HardwareMap;

import localizers.Localizer;

/**
 * Abstract base class for localizer constants.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public abstract class LocalizerConstants {
    /**
    * Constructor for the LocalizerConstants class
    */
    public LocalizerConstants() {}

    /**
     * Builds a Localizer instance based on the constants defined in this class.
     * This method must be implemented by subclasses to return the appropriate type of Localizer.
     * @param hardwareMap the HardwareMap to use for initializing the Localizer's sensors
     * @return a Localizer instance configured with the constants defined in this class
    */
    abstract public Localizer build(HardwareMap hardwareMap);
}