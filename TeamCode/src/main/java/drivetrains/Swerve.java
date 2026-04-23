package drivetrains;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import drivetrains.constants.SwerveConstants;

import java.util.Locale;

/**
 * Swerve drivetrain controller class
 *
 * @author Xander Haemel - 31616 404 Not Found
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class Swerve extends Drivetrain {
    SwerveConstants constants;
    SwerveModule fl;
    SwerveModule bl;
    SwerveModule fr;
    SwerveModule br;

    /**
     * Creates a swerve drivetrain
     * @param hardwareMap the hardware map to use for module initialization
     * @param constants {@link SwerveConstants} object containing all tunable values and motor names/directions
     */
    public Swerve(HardwareMap hardwareMap, SwerveConstants constants){
        this.constants = constants;
        this.fl = constants.flModuleConstants.build(hardwareMap);
        this.bl = constants.blModuleConstants.build(hardwareMap);
        this.fr = constants.frModuleConstants.build(hardwareMap);
        this.br = constants.brModuleConstants.build(hardwareMap);
    }

    @Override
    protected void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        this.fl.setZeroPowerBehavior(behavior);
        this.bl.setZeroPowerBehavior(behavior);
        this.fr.setZeroPowerBehavior(behavior);
        this.br.setZeroPowerBehavior(behavior);
    }
    
    @Override
    protected boolean isRobotCentric() {
        return constants.robotCentric;
    }

    @Override
    public void moveWithVectors(double drive, double strafe, double turn){
        turn *= -1; // Make the turn angle clockwise, as it's normally Counter Clockwise

        // Swerve kinematics calculations
        double strafeRear = strafe - turn * this.constants.getWheelbaseRatio();
        double strafeFront = strafe + turn * this.constants.getWheelbaseRatio();
        double forwardRight = drive - turn * this.constants.getTrackWidthRatio();
        double forwardLeft = drive + turn * this.constants.getTrackWidthRatio();
        //calculate the motor powers
        double flPower = Math.sqrt(Math.pow(strafeFront, 2) + Math.pow(forwardLeft, 2));
        double blPower = Math.sqrt(Math.pow(strafeRear, 2) + Math.pow(forwardLeft, 2));
        double frPower = Math.sqrt(Math.pow(strafeFront, 2) + Math.pow(forwardRight, 2));
        double brPower = Math.sqrt(Math.pow(strafeRear, 2) + Math.pow(forwardRight, 2));

        //current limiting
        double currentRatio = getTotalCurrent()/constants.totalMaxCurrent;
        //normalizer current
        if(getTotalCurrent() > constants.totalMaxCurrent){
            flPower /= currentRatio;
            frPower /= currentRatio;
            blPower /= currentRatio;
            brPower /= currentRatio;
        }
        // Normalize powers from -maxPower to maxPower if any exceed the max
        double max = Math.max(0, Math.abs(flPower));
        max = Math.max(max, Math.abs(blPower));
        max = Math.max(max, Math.abs(frPower));
        max = Math.max(max, Math.abs(brPower));
        if (max > constants.maxPower) {
            //set new powers after normalization
            flPower = (flPower / max) * constants.maxPower;
            blPower = (blPower / max) * constants.maxPower;
            frPower = (frPower / max) * constants.maxPower;
            brPower = (brPower / max) * constants.maxPower;
        }

        // Set pod target angles and powers, update to apply
        this.fl.setTargets(Math.toDegrees(Math.atan2(strafeFront, forwardLeft)), flPower);
        this.bl.setTargets(Math.toDegrees(Math.atan2(strafeRear, forwardLeft)), blPower);
        this.fr.setTargets(Math.toDegrees(Math.atan2(strafeFront, forwardRight)), frPower);
        this.br.setTargets(Math.toDegrees(Math.atan2(strafeRear, forwardRight)), brPower);
        //apply update
        this.fl.update(); this.bl.update(); this.fr.update(); this.br.update();
    }

    @Override
    public void stop() {
        //stop method, cutting power from every swerve module
        this.fl.stop(); this.bl.stop(); this.fr.stop(); this.br.stop(); // Note: stop() calls update()
    }

    @Override
    public void debug(Telemetry telemetry) {
        telemetry.addData("Front Left Module", fl.toString());
        telemetry.addData("Back Left Module", bl.toString());
        telemetry.addData("Front Right Module", fr.toString());
        telemetry.addData("Back Right Module", br.toString());
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "Swerve(fl=%s, bl=%s, fr=%s, br=%s)",
                fl.toString(), bl.toString(), fr.toString(), br.toString());
    }

    /**
     * gets the total current of the drivetrain
     * @return the current in Amps
     */
    private double getTotalCurrent(){
        return fl.getCurrent() + fr.getCurrent() + bl.getCurrent() + br.getCurrent();
    }
}
