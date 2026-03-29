package Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Drivetrains.Mecanum;
import Localizers.PinpointLocalizer;
import Util.Pose;

/**
 * Test for the Mecanum drivetrain class using a Pinpoint localizer
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@TeleOp(name = "MecanumDrive Test", group = "Apex beta test")
public class MecanumDriveTest extends LinearOpMode {
    PinpointLocalizer localizer;
    String localizerName = "localizer"; //todo change as required
    double localizerXOffset = 0.0; //todo replace with actual offset
    double localizerYOffset = 0.0; //todo replace with actual offset

    @Override
    public void runOpMode() {
        // Init localizer
        localizer = new PinpointLocalizer(hardwareMap, localizerName, localizerXOffset, localizerYOffset);
        localizer.init();

        // Init drivetrain with constants
        Mecanum dt = new Mecanum(hardwareMap, Constants.driveConstants);

        while (opModeIsActive()) {
            // Update localizer and grab current pose
            localizer.update();
            Pose currentPose = localizer.getPose();

            // Read driver inputs (invert Y so forward stick = positive drive)
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // Start: emergency stop — cuts all motor power
            if (gamepad1.start) {
                dt.stop();
                telemetry.addLine("Motors stopped");
            } else {
                dt.drive(x, y, turn, currentPose.getHeading());
            }

            // Telemetry output
            telemetry.addData("Pose", currentPose.debug());
            telemetry.addData("X", currentPose.getX());
            telemetry.addData("Y ",currentPose.getY());
            telemetry.addData("heading", currentPose.getHeading());
            telemetry.update();
        }
    }
}