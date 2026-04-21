package drivetrains.constants;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import hardware.MotorMetaData;

/**
 * Configuration for the swerve drivetrain.
 * Measure wheelbase and trackwidth on your actual robot.
 * Hardware naming convention assumed:
 *   Motors:   "fl", "fr", "bl", "br"
 *   Servos:   "flServo", "frServo", "blServo", "brServo"
 *   Encoders: "flEncoder", "frEncoder", "blEncoder", "brEncoder"
 * Change names/directions here to match your config.

 * @author Xander Haemel 31616 404 Not Found
 * @author Sohum Arora 22985 Paraducks
 */
public class SwerveConstants {
    public final MotorMetaData flData, frData, blData, brData;
    public final String flServo, frServo, blServo, brServo;
    public final String flEncoder, frEncoder, blEncoder, brEncoder;
    public final double wheelbase; // front-to-back pod spacing
    public final double trackwidth;  // left-to-right pod spacing
    public final double diagonalDistance;

    public SwerveConstants() {

        double wb = 30.0; // TODO: measure your bot (cm)
        double tw = 30.0; // TODO: measure your bot (cm)
        this.wheelbase = wb;
        this.trackwidth = tw;
        this.diagonalDistance = Math.sqrt(wb * wb + tw * tw);

        // ── Motors ────────────────────────────────────────────────
        // Flip direction on a motor if that wheel drives backwards
        flData = new MotorMetaData("fl", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frData = new MotorMetaData("fr", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blData = new MotorMetaData("bl", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brData = new MotorMetaData("br", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ── Servos ────────────────────────────────────────────────
        flServo = "flServo"; frServo = "frServo";
        blServo = "blServo"; brServo = "brServo";

        // ── Encoders ──────────────────────────────────────────────
        flEncoder = "flEncoder"; frEncoder = "frEncoder";
        blEncoder = "blEncoder"; brEncoder = "brEncoder";
    }
}