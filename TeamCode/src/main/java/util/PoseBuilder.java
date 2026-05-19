package util;

public class PoseBuilder {
    private Distance.Units distanceUnit;
    private Angle.Units angleUnit;
    private boolean mirror;

    // region Constructors
    /**
     * Constructor for the {@link PoseBuilder} class
     * @param distanceUnit the {@link Distance.Units} to use for pose positions
     * @param angleUnit the {@link Angle.Units} to use for pose headings
     * @param mirror whether to mirror the pose across the y-axis (for switching alliances)
     */
    public PoseBuilder(Distance.Units distanceUnit, Angle.Units angleUnit, boolean mirror) {
        this.distanceUnit = distanceUnit;
        this.angleUnit = angleUnit;
        this.mirror = mirror;
    }

    /**
     * Constructor for the {@link PoseBuilder} class with mirroring set to false
     * @param distanceUnit the {@link Distance.Units} to use for pose positions
     * @param angleUnit the {@link Angle.Units} to use for pose headings
     */
    public PoseBuilder(Distance.Units distanceUnit, Angle.Units angleUnit) {
        this(distanceUnit, angleUnit, false);
    }

    /**
     * Constructor for the {@link PoseBuilder} class with default units of inches and degrees,
     * and mirroring set to false
     */
    public PoseBuilder() { this(Distance.Units.INCHES, Angle.Units.DEGREES, false); }
    // endregion

    // region Builder methods
    /**
     * Builds a {@link Pose} with the specified x, y, and heading values in the builder's units
     * @param x the x component of the position in the builder's distance unit
     * @param y the y component of the position in the builder's distance unit
     * @param heading the heading of the pose as an angle value in the builder's angle unit
     * @return a new {@link Pose} object with the specified values and builder's units
     */
    public Pose build(double x, double y, double heading) {
        return new Pose(x, y, heading, this.distanceUnit, this.angleUnit, this.mirror);
    }

    /**
    * Builds a {@link Pose} with the specified x and y values in the builder's units
    * and a heading of 0
    * @param x the x component of the position in the builder's distance unit
    * @param y the y component of the position in the builder's distance unit
    * @return a new {@link Pose} object with the specified values and builder's units
    */
    public Pose build(double x, double y) { return build(x, y, 0.0); }

    /**
     * Builds a {@link Pose} with the given {@link Vector} and {@link Angle} in the builder's units
     * @param position the position of the pose as a {@link Vector}
     * @param heading the heading of the pose as an {@link Angle}
     * @return a new {@link Pose} object with the specified values and builder's units
     */
    public Pose build(Vector position, Angle heading) {
        return new Pose(
                position.getXComponent().get(this.distanceUnit),
                position.getYComponent().get(this.distanceUnit),
                heading.get(this.angleUnit),
                this.distanceUnit, this.angleUnit, this.mirror
        );
    }

    /**
    * Builds a {@link Pose} with the given {@link Vector} and a heading of 0 in the builder's units
    * @param position the position of the pose as a {@link Vector}
    * @return a new {@link Pose} object with the specified values and builder's units
    */
    public Pose build(Vector position) { return build(position, new Angle()); }
    // endregion

    // region Getters and setters for builder configuration
    /** @return the {@link Distance.Units} used by the builder for pose positions */
    public Distance.Units getDistanceUnit() { return this.distanceUnit; }

    /** @param distanceUnit the {@link Distance.Units} for the builder to use for positions */
    public void setDistanceUnit(Distance.Units distanceUnit) { this.distanceUnit = distanceUnit; }

    /** @return the {@link Angle.Units} used by the builder for pose headings */
    public Angle.Units getAngleUnit() { return this.angleUnit; }

    /** @param angleUnit the {@link Angle.Units} for the builder to use for headings */
    public void setAngleUnit(Angle.Units angleUnit) { this.angleUnit = angleUnit; }

    /** @return whether the builder is set to mirror poses across the y-axis */
    public boolean isMirror() { return this.mirror; }

    /** @param mirror whether the builder should mirror poses across the y-axis */
    public void setMirror(boolean mirror) { this.mirror = mirror; }

    /** Toggles the mirroring setting of the builder */
    public void toggleMirror() { this.mirror = !this.mirror; }
    // endregion
}