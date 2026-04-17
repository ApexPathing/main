package localizers;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import localizers.constants.OTOSConstants;
import util.Angle;
import util.Distance;
import util.Pose;

public class OTOS extends Localizer {
    private final SparkFunOTOS otos;

    public OTOS(SparkFunOTOS otos, OTOSConstants constants, Pose startPose) {
        this.otos = otos;

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);
        otos.setOffset(toSparkfunPose2D(constants.offset));
        otos.setLinearScalar(constants.linearScalar);
        otos.setAngularScalar(constants.headingScalar);
        otos.calibrateImu();
        otos.resetTracking();
        this.setPose(startPose);
    }

    private SparkFunOTOS.Pose2D toSparkfunPose2D(Pose pose) {
        return new SparkFunOTOS.Pose2D(
                pose.getXComponent().getIn(),
                pose.getYComponent().getIn(),
                pose.getHeadingComponent().getRad()
        );
    }

    @Override
    public void update() {
        SparkFunOTOS.Pose2D pos = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D vel = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D acc = new SparkFunOTOS.Pose2D();
        otos.getPosVelAcc(pos, vel, acc);

        // NOTE: ADD ACCELERATION IF NEEDED
        currentPose = new Pose(
                pos.x, pos.y, pos.h,
                Distance.Units.INCHES, Angle.Units.RADIANS, false
        );
        currentVelocity = new Pose(
                vel.x, vel.y, vel.h,
                Distance.Units.INCHES, Angle.Units.RADIANS, false
        );
    }

    @Override
    public void setPose(Pose pose) {
        otos.setPosition(toSparkfunPose2D(pose));
    }
}