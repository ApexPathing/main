package localizers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Helper class for 2/3 wheel odometry and drive encoder localizers.
 *
 * @author Topher F. - 23571 alum
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class OdometryPod {
    private final String name;

    private final double ticksPerInch;
    private final double direction;
    private final DcMotorEx odometry;

    private int lastTicks;
    private int currentTicks;
    private double deltaTicks;

    public OdometryPod(HardwareMap hardwareMap, String name, double ticksPerInch) {
        this(hardwareMap, name, ticksPerInch, false);
    }

    /** Creates a pod with an optional software reversal relative to the SDK encoder reading. */
    public OdometryPod(HardwareMap hardwareMap, String name, double ticksPerInch,
                       boolean reversed) {
        this.name = name;
        this.odometry = hardwareMap.get(DcMotorEx.class, this.name);
        this.ticksPerInch = ticksPerInch;
        this.direction = reversed ? -1.0 : 1.0;
        reset();
    }

    public String getName() { return this.name; }

    public void update() {
        currentTicks = odometry.getCurrentPosition();
        deltaTicks = currentTicks - lastTicks;
        lastTicks = currentTicks;
    }

    public void reset() {
        currentTicks = odometry.getCurrentPosition();
        lastTicks = currentTicks;
        deltaTicks = 0.0;
    }

    /** @return the amount of inches the encoder has moved since the last reset */
    public double getInches() { return direction * currentTicks / ticksPerInch; }

    /** @return the amount of inches the encoder has moved since the last loop. */
    public double getDeltaInches() { return direction * deltaTicks / ticksPerInch; }

    /** Returns the current raw SDK encoder position without changing the motor run mode. */
    public int getTicks() { return currentTicks; }

    /** Returns raw encoder ticks accumulated during the most recent update. */
    public double getDeltaTicks() { return deltaTicks; }
}

