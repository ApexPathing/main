package drivetrains;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import drivetrains.constants.MecanumConstants;

/**
 * Mecanum Drivetrain controller class
 * @author Xander Haemel - 31616 - 404 Not Found
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class Mecanum extends Drivetrain {
    MecanumConstants constants;

    // Motors
    DcMotorEx lf;
    DcMotorEx lr;
    DcMotorEx rf;
    DcMotorEx rr;

    /**
     * Creates a mecanum drivetrain
     * @param hardwareMap the hardware map to use for motor initialization
     * @param constants MecanumConstants object containing all tunable values and motor names/directions
     */
    public Mecanum(HardwareMap hardwareMap, MecanumConstants constants){
        this.constants = constants;

        // Initialize motors
        lf = hardwareMap.get(DcMotorEx.class, constants.leftFrontMotorName);
        lr = hardwareMap.get(DcMotorEx.class, constants.leftRearMotorName);
        rf = hardwareMap.get(DcMotorEx.class, constants.rightFrontMotorName);
        rr = hardwareMap.get(DcMotorEx.class, constants.rightRearMotorName);

        // Set motor directions
        lf.setDirection(constants.leftFrontDirection);
        lr.setDirection(constants.leftRearDirection);
        rf.setDirection(constants.rightFrontDirection);
        rr.setDirection(constants.rightRearDirection);

        setBrakeMode(constants.useBrakingMode);
    }

    @Override
    protected void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        lf.setZeroPowerBehavior(behavior);
        lr.setZeroPowerBehavior(behavior);
        rf.setZeroPowerBehavior(behavior);
        rr.setZeroPowerBehavior(behavior);
    }

    /**
     * Sets the power for each motor, normalizing the powers if any exceed the maximum allowed power.
     * @param lfPower the power to set for the left front motor
     * @param lrPower the power to set for the left rear motor
     * @param rfPower the power to set for the right front motor
     * @param rrPower the power to set for the right rear motor
     */
    private void setPowers(double lfPower, double lrPower, double rfPower, double rrPower) {
        // Normalize powers from -1 to 1
        double max = Math.max(0, Math.abs(lfPower));
        max = Math.max(max, Math.abs(lrPower));
        max = Math.max(max, Math.abs(rfPower));
        max = Math.max(max, Math.abs(rrPower));
        if (max > constants.maxPower) {
            lfPower = (lfPower / max) * constants.maxPower;
            lrPower = (lrPower / max) * constants.maxPower;
            rfPower = (rfPower / max) * constants.maxPower;
            rrPower = (rrPower / max) * constants.maxPower;
        }

        lf.setPower(lfPower);
        lr.setPower(lrPower);
        rf.setPower(rfPower);
        rr.setPower(rrPower);
    }

    @Override
    public void moveWithVectors(double drive, double strafe, double turn) {
        double lfPower = drive + strafe + turn;
        double lrPower = drive - strafe + turn;
        double rfPower = drive -strafe - turn;
        double rrPower = drive + strafe - turn;

        setPowers(lfPower, lrPower, rfPower, rrPower);
    }

    @Override
    public void drive(double x, double y, double turn, double robotHeading) {
        double adjX, adjY, adjTurn;
        if (constants.robotCentric) {
            adjX = deadzone(x);
            adjY = deadzone(y);
        } else {
            double cos = Math.cos(-robotHeading);
            double sin = Math.sin(-robotHeading);
            adjX = deadzone(x * cos - y * sin);
            adjY = deadzone(x * sin + y * cos);
        }
        adjTurn = deadzone(turn);
        moveWithVectors(adjY, adjX, adjTurn);
    }

    @Override
    public void stop() {
        setPowers(0, 0, 0, 0);
    }

    @Override
    public void debug(Telemetry telemetry) {
        telemetry.addLine("---Power---");
        telemetry.addData("leftFront Power", lf.getPower());
        telemetry.addData("rightFront Power", rf.getPower());
        telemetry.addData("leftRear Power", lr.getPower());
        telemetry.addData("rightRear Power", rr.getPower());

        telemetry.addLine("---Velocity---");
        telemetry.addData("leftFront velocity", lf.getVelocity());
        telemetry.addData("rightFront velocity", rf.getVelocity());
        telemetry.addData("leftRear velocity", lr.getVelocity());
        telemetry.addData("rightRear velocity", rr.getVelocity());
    }
}
