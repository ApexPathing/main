# Tank drive encoder localization

Return `TankDriveEncoders.Constants` from your `ApexConstants.localizerConstants()`:

```java
return new TankDriveEncoders.Constants()
        .setIMUName("imu")
        .setHubOrientation(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD))
        .addLeftEncoder("leftFront", 100.0, false)
        .addLeftEncoder("leftRear", 100.0, false)
        .addRightEncoder("rightFront", 100.0, false)
        .addRightEncoder("rightRear", 100.0, false);
```

Import `localizers.TankDriveEncoders` and
`com.qualcomm.hardware.rev.RevHubOrientationOnRobot`. Replace the names,
orientation, and example ticks-per-inch values with your robot's configuration.

Add one or more encoder motors per side. You can use a subset of the drive
motors, and the side counts can differ. Each encoder is converted to inches
using its own positive ticks-per-inch value. The localizer averages each side,
then averages the two sides equally.

The final boolean reverses the SDK encoder reading only within localization.
SDK readings already reflect the motor's configured direction: do not reverse
them twice. With the drivetrain configured, verify that every selected encoder
reports positive distance when the robot moves forward. Do not change motor
directions or reset hardware encoder counts while this localizer is running.

Heading comes from the IMU, so track width is not required. X is forward at
heading zero, Y is left, and positive heading is counterclockwise. The localizer
starts at zero pose relative to the initial IMU yaw. `setPose()` rebases encoder
and yaw history without resetting motor encoders, motor modes, or IMU yaw.
Heading remains continuous across the IMU's wrap boundary. Update frequently
enough that rotation between valid readings is less than 180 degrees.

Velocity and acceleration use BaseLocalizer's shared filtering, currently
Kalman by default. Translation assumes no lateral slip; a tank can slide
sideways without this localizer observing that displacement. Arc integration
assumes approximately constant curvature within each update.

Initialize/configure the drivetrain before building the localizer. This class
reads any number of motors but does not configure or command them, and does not
extend the Tank drivetrain controller's motor-count support. Invalid IMU yaw
defers an update; recovery integrates accumulated encoder travel, so long
sensor gaps can reduce accuracy. Pose resets require a finite IMU reading.
