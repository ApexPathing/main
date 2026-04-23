package core;

import com.qualcomm.robotcore.hardware.HardwareMap;

import drivetrains.Drivetrain;
import localizers.Localizer;
import followers.Follower;
import drivetrains.constants.DrivetrainConstants;
import localizers.constants.LocalizerConstants;
import followers.constants.FollowerConstants;
import util.Pose;

/**
 * Abstract class for building a {@link Follower} object using custom drivetrain, localizer, and
 * follower constants. To use this class, create a new class that extends ApexBuilder and override
 * the setDrivetrainConstants(), setLocalizerConstants(), and setFollowerConstants() methods to
 * return your desired constants. Then, call the build() method with the hardware map and starting
 * pose to get a Follower object that you can use in your OpMode.
 *
 * @author Dylan B. 18597 RoboClovers - Delta
 */
public abstract class ApexBuilder {
    public DrivetrainConstants drivetrainConstants;
    public LocalizerConstants localizerConstants;
    public FollowerConstants followerConstants;

    public ApexBuilder() {
        this.drivetrainConstants = this.setDrivetrainConstants();
        this.localizerConstants = this.setLocalizerConstants();
        this.followerConstants = this.setFollowerConstants();
    }

    /**
     * Override this method to set the drivetrain constants for your robot.
     * This will be used to build the drivetrain in the build() method.
     * Here is an example of overriding this method to set mecanum drivetrain constants:
     *
     * <pre>
     * {@code
     * @Override
     * public DrivetrainConstants setDrivetrainConstants() { // Any DrivetrainConstants
     *      return new MecanumConstants() // This can be any class that extends DrivetrainConstants
     * }
     * }
     * </pre>
     *
     * @return any constants class that extends {@link DrivetrainConstants} with the appropriate values set.
     */
    public abstract DrivetrainConstants setDrivetrainConstants();

    /**
     * Override this method to set the localizer constants for your robot.
     * This will be used to build the localizer in the build() method.
     * Here is an example of overriding this method to set Pinpoint localizer constants:
     *
     * <pre>
     * {@code
     * @Override
     * public LocalizerConstants setLocalizerConstants() { // Any LocalizerConstants
     *      return new PinpointConstants() // This can be any class that extends LocalizerConstants
     * }
     * }
     * </pre>
     *
     * @return any constants class that extends {@link LocalizerConstants} with the appropriate values set.
     */
    public abstract LocalizerConstants setLocalizerConstants();

    /**
     * Override this method to set the follower constants for your robot.
     * This will be used to build the follower in the build() method.
     * Here is an example of overriding this method to set P2P follower constants:
     *
     * <pre>
     * {@code
     * @Override
     * public FollowerConstants setFollowerConstants() { // Any FollowerConstants
     *      return new P2PFollowerConstants() // This can be any class that extends FollowerConstants
     * }
     * }
     * </pre>
     *
     * @return any constants class that extends {@link FollowerConstants} with the appropriate values set.
     */
    public abstract FollowerConstants setFollowerConstants();

    /**
     * Builds the follower using the constants set in the overridden methods.
     * This is the method you should call to get a Follower object in your OpMode.
     * @param hardwareMap the {@link HardwareMap} object from the OpMode
     * @param startPose the starting {@link Pose} of the robot on the field
     * @return a {@link Follower} object built with the specified constants and hardware
     */
    public Follower build(HardwareMap hardwareMap, Pose startPose) {
        Drivetrain drivetrain = this.buildOnlyDrivetrain(hardwareMap);
        Localizer localizer = this.buildOnlyLocalizer(hardwareMap, startPose);

        return this.followerConstants.build(drivetrain, localizer);
    }

    /**
     * Builds only the drivetrain using the constants set in the overridden setDrivetrainConstants()
     * method. You generally shouldn't use this method as a user unless you know what you're doing.
     * @param hardwareMap the {@link HardwareMap} object from the OpMode
     * @return a {@link Drivetrain} object built with the specified constants and hardware
     */
    public Drivetrain buildOnlyDrivetrain(HardwareMap hardwareMap) {
        return this.drivetrainConstants.build(hardwareMap);
    }

    /**
     * Builds only the localizer using the constants set in the overridden setLocalizerConstants()
     * method and a starting pose. You generally shouldn't use this method as a user unless you know
     * what you're doing.
     * @param hardwareMap the {@link HardwareMap} object from the OpMode
     * @param startPose the starting {@link Pose} of the robot on the field
     * @return a {@link Localizer} object built with the specified constants and hardware
     */
    public Localizer buildOnlyLocalizer(HardwareMap hardwareMap, Pose startPose) {
        Localizer localizer = this.localizerConstants.build(hardwareMap);
        localizer.setPose(startPose);

        return localizer;
    }
}