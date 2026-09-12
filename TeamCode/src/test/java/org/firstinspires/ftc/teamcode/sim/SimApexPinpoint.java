package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import org.codeblooded.ftcodesim.physics.MotionVector;
import org.codeblooded.ftcodesim.hardware.devices.SimHardwareDevice;
import org.codeblooded.ftcodesim.hardware.drivetrain.SimulatedDrivetrain;

import java.lang.reflect.Array;
import java.lang.reflect.Proxy;

import geometry.Angle;
import geometry.Dist;
import geometry.DistUnit;
import geometry.Pose;
import geometry.Vector;
import localizers.Pinpoint;

/**
 * FTCodeSim adapter for Apex Pathing's custom Pinpoint driver.
 *
 * <p>FTCodeSim's built-in Pinpoint implements the SDK's GoBildaPinpointDriver, while Apex Pathing
 * intentionally uses its own driver type. HardwareMap lookups are type-sensitive, so the built-in
 * device cannot be returned to Apex's localizer.</p>
 */
final class SimApexPinpoint extends Pinpoint.Driver implements SimHardwareDevice {
    private final SimulatedDrivetrain drivetrain;
    private final double fieldCenterInches;

    private Pose pose = Pose.zero();
    private Pose velocity = Pose.zero();
    private MotionVector previousSimPose;
    private double xTicks;
    private double yTicks;
    private double xOffsetInches;
    private double yOffsetInches;
    private double ticksPerInch = 19.89436789 * 25.4;
    private double yawScalar = 1.0;
    private double xDirection = 1.0;
    private double yDirection = 1.0;

    SimApexPinpoint(SimulatedDrivetrain drivetrain, double fieldCenterInches) {
        super(fakeI2c(), false);
        this.drivetrain = drivetrain;
        this.fieldCenterInches = fieldCenterInches;
        update(0.0);
    }

    @Override
    public void update(double deltaTime) {
        MotionVector simPose = drivetrain.position;
        MotionVector simVelocity = drivetrain.velocity;

        if (previousSimPose != null) {
            double deltaHeading = Angle.wrap(simPose.theta - previousSimPose.theta);
            double heading = previousSimPose.theta + deltaHeading / 2.0;
            double fieldX = simPose.x - previousSimPose.x;
            double fieldY = simPose.y - previousSimPose.y;
            double forward = fieldX * Math.cos(heading) + fieldY * Math.sin(heading);
            double strafe = -fieldX * Math.sin(heading) + fieldY * Math.cos(heading);
            xTicks += xDirection * (forward + xOffsetInches * deltaHeading) * ticksPerInch;
            yTicks += yDirection * (strafe + yOffsetInches * deltaHeading) * ticksPerInch;
        }
        previousSimPose = simPose;

        // Convert FTCodeSim's corner origin to Apex's centered origin. ApexSimulation mirrors the
        // simulator wheel slots so its Y and heading axes already match Apex's conventions.
        pose = apexPose(
                simPose.x - fieldCenterInches,
                simPose.y - fieldCenterInches,
                simPose.theta * yawScalar
        );
        velocity = apexPose(simVelocity.x, simVelocity.y, simVelocity.theta * yawScalar);
    }

    @Override
    public void update() {
        update(0.0);
    }

    @Override
    public Pose getPosition() {
        return pose;
    }

    @Override
    public Pose getVelocity() {
        return velocity;
    }

    @Override
    public void setPosition(Pose newPose) {
        drivetrain.setPosition(new MotionVector(
                newPose.getX().getIn() + fieldCenterInches,
                newPose.getY().getIn() + fieldCenterInches,
                newPose.getHeading().getRad()
        ));
        previousSimPose = drivetrain.position;
        update(0.0);
    }

    @Override
    public void resetPosAndIMU() {
        xTicks = 0.0;
        yTicks = 0.0;
        setPosition(Pose.zero());
    }

    @Override
    public void setOffsets(Vector offset) {
        xOffsetInches = offset.getX().getIn();
        yOffsetInches = offset.getY().getIn();
    }

    @Override
    public void setEncoderDirections(Pinpoint.EncoderDirection xEncoder,
                                     Pinpoint.EncoderDirection yEncoder) {
        xDirection = xEncoder == Pinpoint.EncoderDirection.REVERSED ? -1.0 : 1.0;
        yDirection = yEncoder == Pinpoint.EncoderDirection.REVERSED ? -1.0 : 1.0;
    }

    @Override
    public void setEncoderResolution(Pinpoint.GoBildaPods pods) {
        ticksPerInch = (pods == Pinpoint.GoBildaPods.goBILDA_SWINGARM_POD
                ? 13.26291192 : 19.89436789) * 25.4;
    }

    @Override
    public void setEncoderResolution(Dist ticksPerUnit) {
        ticksPerInch = ticksPerUnit.getMm() * 25.4;
    }

    @Override
    public void setYawScalar(double yawOffset) { yawScalar = yawOffset; }

    @Override
    public int[] getEncoderPositions() {
        return new int[] {(int) Math.round(xTicks), (int) Math.round(yTicks)};
    }

    @Override
    protected synchronized boolean doInitialize() {
        return true;
    }

    private static Pose apexPose(double xInches, double yInches, double headingRadians) {
        return new Pose(
                Vector.of(xInches, yInches, DistUnit.IN),
                Angle.fromRad(headingRadians)
        );
    }

    private static I2cDeviceSynchSimple fakeI2c() {
        return (I2cDeviceSynchSimple) Proxy.newProxyInstance(
                SimApexPinpoint.class.getClassLoader(),
                new Class<?>[] { I2cDeviceSynchSimple.class },
                (proxy, method, args) -> defaultValue(method.getReturnType())
        );
    }

    private static Object defaultValue(Class<?> type) {
        if (!type.isPrimitive()) {
            return type.isArray() ? Array.newInstance(type.getComponentType(), 0) : null;
        }
        if (type == boolean.class) { return false; }
        if (type == char.class) { return '\0'; }
        if (type == byte.class) { return (byte) 0; }
        if (type == short.class) { return (short) 0; }
        if (type == int.class) { return 0; }
        if (type == long.class) { return 0L; }
        if (type == float.class) { return 0.0f; }
        if (type == double.class) { return 0.0; }
        return null;
    }
}
