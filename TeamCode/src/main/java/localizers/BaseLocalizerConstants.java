package localizers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.json.JSONObject;

/**
 * Abstract class implemented by all localizer configuration classes
 *
 * <p>
 * When creating a localization configuration, you must extend this class and implement the build()
 * method to return an instance of the corresponding localizer class using your configuration class.
 * Your constants should have a public scope and be initialized with default values.
 * </p>
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@FunctionalInterface
public interface BaseLocalizerConstants<T extends BaseLocalizerConstants<T>> {
    /**
     * Builds and returns an instance of the corresponding localizer class using this configuration.
     */
    BaseLocalizer<?> build(HardwareMap hardwareMap);

    /**
     * Returns the geometry values which may be overridden by saved localization calibration.
     * Custom localizers may leave the default empty object and remain monitor/filter-only.
     */
    default JSONObject getCalibrationValues() { return new JSONObject(); }

    /** Applies validated geometry values loaded before the localizer is constructed. */
    default void applyCalibrationValues(JSONObject values) { }

    /** Stable identifier used to reject calibration saved for another localizer implementation. */
    default String getCalibrationType() { return getClass().getName(); }
}
