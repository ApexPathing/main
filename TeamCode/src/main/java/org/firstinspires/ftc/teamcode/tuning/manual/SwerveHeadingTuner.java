package org.firstinspires.ftc.teamcode.tuning.manual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;

import controllers.PDFLController;
import drivetrains.Drivetrain;
import drivetrains.Swerve;
import drivetrains.SwerveModule;
import drivetrains.constants.SwerveConstants;
import drivetrains.constants.SwerveModuleConstants;
import localizers.Localizer;
import util.Pose;

/**
 * @author Xander Haemel - UNFINISHED
 */
public class SwerveHeadingTuner extends LinearOpMode {
    SwerveConstants constants;
    SwerveModule fl;
    SwerveModule fr;
    SwerveModule bl;
    SwerveModule br;

    double flSteeringGain = 0;
    double frSteeringGain = 0;
    double blSteeringGain = 0;
    double brSteeringGain = 0;
    private enum tuning{
        FL, FR, BL, BR
    }
    tuning currentTuning = tuning.FL;

    @Override
    public void runOpMode() throws InterruptedException {
            constants = new SwerveConstants();
            fl = constants.flModuleConstants.build(hardwareMap);
            fr = constants.frModuleConstants.build(hardwareMap);
            bl = constants.blModuleConstants.build(hardwareMap);
            br = constants.brModuleConstants.build(hardwareMap);
            waitForStart();
            while (opModeIsActive()){
                switch (currentTuning){
                    case FL:
                        fl.setTargets(180, 0);
                        fl.update();

                }

            }
    }
}
