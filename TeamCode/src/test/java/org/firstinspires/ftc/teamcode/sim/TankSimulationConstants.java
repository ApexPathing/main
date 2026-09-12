package org.firstinspires.ftc.teamcode.sim;

import core.ApexConstants;
import drivetrains.BaseDrivetrainConstants;
import drivetrains.Motor;
import drivetrains.Tank;
import geometry.DistUnit;
import localizers.BaseLocalizerConstants;
import localizers.Pinpoint;

/** Robot configuration shared by the tank-physics simulator OpModes and tests. */
public final class TankSimulationConstants implements ApexConstants {
    @Override public BaseDrivetrainConstants<?> drivetrainConstants() {
        return new Tank.Constants()
                .setFrontLeftMotor(new Motor(ApexSimulation.FRONT_LEFT_MOTOR))
                .setFrontRightMotor(new Motor(ApexSimulation.FRONT_RIGHT_MOTOR).reverse())
                .setBackLeftMotor(new Motor(ApexSimulation.BACK_LEFT_MOTOR))
                .setBackRightMotor(new Motor(ApexSimulation.BACK_RIGHT_MOTOR).reverse())
                .setMaxPower(1.0);
    }

    @Override public BaseLocalizerConstants<?> localizerConstants() {
        // FTCodeSim's SimMotor does not currently expose encoder position, so the simulated
        // Pinpoint supplies ground-truth motion while the tank drivetrain supplies the physics.
        return new Pinpoint.Constants()
                .setName(ApexSimulation.PINPOINT)
                .setOffsets(0, 0, DistUnit.IN)
                .setEncoderDirections(Pinpoint.EncoderDirection.FORWARD,
                        Pinpoint.EncoderDirection.FORWARD)
                .setEncoderResolution(Pinpoint.GoBildaPods.goBILDA_4_BAR_POD);
    }
}
