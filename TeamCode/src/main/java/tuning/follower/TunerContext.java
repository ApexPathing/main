package tuning.follower;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.json.JSONArray;
import org.json.JSONObject;

import core.ApexStorage;
import core.Follower;
import core.FollowerConstants;
import geometry.Pose;

/**
 * Provides a context for the tuner phases to operate in, including access th the OpMode, telemetry,
 * and the follower instance.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class TunerContext extends tuning.TunerContext {
    private Follower follower;
    public FollowerConstants constants;
    public TunerContext(LinearOpMode opMode) { super(opMode); }

    public void setFollower(Follower follower) {
        this.follower = follower;
        this.constants = follower.getConstants();
    }

    public Follower getFollower() { return follower; }

    boolean testButtonWasPressed() { return opMode.gamepad1.xWasPressed(); }

    boolean acceptButtonWasPressed() { return opMode.gamepad1.aWasPressed(); }

    boolean retuneButtonWasPressed() { return opMode.gamepad1.bWasPressed(); }

    /** Adds controls which must remain visible independently of the current phase. */
    public void addInterfaceHeader() {
        if (follower != null && follower.getDrivetrain() instanceof drivetrains.DualActuated) {
            getTelemetry().addData("Tuning profile", constants.getActiveProfile());
        }
        addDebugHeader();
        if (opMode.opModeIsActive()) {
            getTelemetry().addLine("Sticks: field-centric drive while tuner motion is idle.");
        }
        if (isDebugMode() || opMode.opModeIsActive()) { getTelemetry().addLine(); }
    }

    /** Teleports only FTCodeSim; real hardware must still be positioned by its operator. */
    public void positionRobotForSimulation(Pose pose) {
        if (!Boolean.getBoolean("apex.simulation.unlockTunerPhases")) { return; }
        follower.stop();
        follower.setPose(pose);
    }

    public boolean saveConstants() {
        JSONObject constantsJSON = new JSONObject();
        try {
            constantsJSON = constants.toJson();
            ApexStorage.saveConstants(constantsJSON.toString(4));
            return true;
        } catch (Exception e) {
            getTelemetry().addLine("WARNING: Values were not saved successfully");
            getTelemetry().addLine("Error: " + e.getMessage());

            JSONArray keys = constantsJSON.names();
            if (keys != null) {
                try {
                    for (int i = 0; i < keys.length(); i++) {
                        String key = keys.getString(i);
                        getTelemetry().addData(key, constantsJSON.get(key));
                    }
                } catch (Exception ex) {
                    getTelemetry().addLine("Error displaying constants: " + ex.getMessage());
                }
            } else {
                getTelemetry().addLine("No constants were found to display.");
            }

            getTelemetry().update();
            return false;
        }
    }
}
