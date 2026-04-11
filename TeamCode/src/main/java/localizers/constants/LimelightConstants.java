package localizers.constants;

/**
 * Limelight localizer constants class
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class LimelightConstants {
    // Hardware
    public String name = "limelight";
    public int pipeline = 0;
    public boolean useMetaTag2 = true;

    /**
     * Constructor for the PinpointConstants class
     * Default constants are derived from the SensorGoBildaPinpoint SDK sample with untuned values
     */
    public LimelightConstants() {}

    /**
     * Sets the pinpoint hardware name.
     * @param name the hardware map name of the pinpoint
     * @return this instance for chaining
     */
    public LimelightConstants setName(String name) {
        this.name = name;
        return this;
    }

    /**
     * Sets the pipeline index to use for the Limelight.
     * @param pipeline the pipeline index to use
     * @return this instance for chaining
     */
    public LimelightConstants setPipeline(int pipeline) {
        this.pipeline = pipeline;
        return this;
    }

    /**
    * Sets whether to use MetaTag2 for AprilTag detection.
    * @param useMetaTag2 true to use MetaTag2, false to use standard AprilTag detection
    * @return this instance for chaining
    */
    public LimelightConstants setUseMetaTag2(boolean useMetaTag2) {
        this.useMetaTag2 = useMetaTag2;
        return this;
    }
}
