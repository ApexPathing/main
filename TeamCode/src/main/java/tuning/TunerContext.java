package tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.Locale;

/**
 * Shared interaction state for Apex tuning OpModes.
 *
 * <p>This class owns the common telemetry formatting and hidden debug-mode gesture. Hardware and
 * persistence remain the responsibility of the specific tuner context.</p>
 */
public abstract class TunerContext {
    private static final long LOOP_PERIOD_MILLIS = 20L;
    private static final long DEBUG_HOLD_NANOS = 1_500_000_000L;
    private static final DecimalFormat NORMAL_NUMBER_FORMAT = new DecimalFormat(
            "0.#####", DecimalFormatSymbols.getInstance(Locale.US));

    protected final LinearOpMode opMode;
    private boolean debugMode;
    private boolean debugHoldHandled;
    private long debugHoldStartedNanos;

    protected TunerContext(LinearOpMode opMode) { this.opMode = opMode; }

    /** Returns the Driver Station telemetry used by this tuner. */
    public Telemetry getTelemetry() { return opMode.telemetry; }

    /** Returns whether the expanded diagnostic display is active. */
    public boolean isDebugMode() { return debugMode; }

    /** Formats values compactly for normal telemetry and losslessly for debugging. */
    public String formatNumber(double value) {
        if (debugMode) { return Double.toString(value); }
        synchronized (NORMAL_NUMBER_FORMAT) {
            return NORMAL_NUMBER_FORMAT.format(value);
        }
    }

    /** Debug can be enabled only from a menu, but may be disabled from any screen. */
    public void updateDebugMode(boolean allowEnable) {
        boolean held = debugMode
                ? opMode.gamepad1.right_stick_button
                : allowEnable && opMode.gamepad1.left_stick_button;
        if (!held) {
            debugHoldStartedNanos = 0L;
            debugHoldHandled = false;
            return;
        }
        if (debugHoldHandled || (!allowEnable && !debugMode)) { return; }
        if (debugHoldStartedNanos == 0L) {
            debugHoldStartedNanos = System.nanoTime();
            return;
        }
        if (System.nanoTime() - debugHoldStartedNanos >= DEBUG_HOLD_NANOS) {
            debugMode = !debugMode;
            debugHoldHandled = true;
        }
    }

    /** Starts a normal tuner frame with the shared debug and context headers. */
    public void beginFrame() {
        updateDebugMode(false);
        getTelemetry().clearAll();
        addInterfaceHeader();
    }

    /** Maintains the common 50 Hz tuner cadence. */
    public void pauseLoop() { opMode.sleep(LOOP_PERIOD_MILLIS); }

    /** Adds the debug banner shared by all tuner screens. */
    protected void addDebugHeader() {
        if (!debugMode) { return; }
        getTelemetry().addLine("DEBUG MODE");
        getTelemetry().addLine("Hold Right Stick Button to exit debug mode.");
    }

    /** Adds controls and state which remain visible throughout this tuner. */
    public abstract void addInterfaceHeader();
}
