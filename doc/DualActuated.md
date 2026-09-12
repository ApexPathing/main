# Dual-actuated tuning and saved profiles

`DualActuated` has independent `TANK` and `HOLONOMIC` follower profiles. Each
contains the complete follower gains and motion limits. Other drivetrains use
one `DEFAULT` profile. Hardware settings (motor names, servo positions, initial
mode, and deployment time) remain in `DualActuated.Constants`.

## Tuning

Configure the robot's `drivetrainConstants()` to return `DualActuated.Constants`.
In Follower Tuner's phase picker, use Dpad Left/Right to choose the mode, then
A to choose a phase. Selecting a mode actuates the configured mechanism while
drive motors are stopped. The current profile is displayed throughout tuning.
Finish one mode, then run the tuner again for the other mode.

Traction mode skips lateral limit trials and centripetal compensation. Its
limit-completion check requires only forward and angular limits. Translation
test paths use tank-compatible heading interpolation, including reverse travel.

Mode-specific values are saved together in `FIRST/ApexPathing/constants.json`:

```json
{
  "schemaVersion": 2,
  "drivetrainType": "DUAL_ACTUATED",
  "profiles": {
    "TANK": { "translationKV": 0.02 },
    "HOLONOMIC": { "translationKV": 0.03 }
  }
}
```

This abbreviated example only illustrates the structure; actual saved profiles
contain all gains and limits. Profile presence does not imply every tuning phase
has been completed. The tuner shows completion separately for the selected mode.
Saving retains the previous file as `constants.json.bak`; failed phase saves
offer retry or exit without claiming success. Desktop storage still uses the
`apexpathing.storageDirectory` property.

## Existing files

Old single-mode files load as `DEFAULT` and migrate on the next save. Old
`DUAL_ACTUATED` files are kept as `unassignedLegacy`, because they do not identify
the mode that produced their values. In the tuner, select the correct mode and
press Y to explicitly assign and save the legacy values to that mode. This
replaces that mode's current values, leaves the other mode intact, and consumes
the unassigned legacy entry. It does not validate the imported calibration.

Normal dual-actuated operation rejects a mode with no available profile;
tuning mode permits starting from zero. Files for a different drivetrain type
are not applied to the configured drivetrain.

## Runtime selection

Tank and holonomic paths generate their motion profiles using their intended
mode's constants, without changing the active hardware mode. When following a
path, the follower selects the matching hardware mode, loads its gains and
limits, and resets controller history. An external mode change during an active
path with different kinematics stops the path and reports an error.

TurnBuilder captures the active profile by default. When constructing a future
turn for a different mode, select it before setting custom limits:

```java
Turn turn = new TurnBuilder(startPose)
        .setDriveProfile(FollowerConstants.Profile.TANK)
        .turnTo(Angle.fromDeg(90))
        .profiledBuild();
```

The turn retains that mode when reversed. A directly constructed unbound `Turn`
uses the drivetrain's current mode; use TurnBuilder for mode-specific profiles.

Set deployment time with `setTransitionSeconds(seconds)` on the drivetrain
constants. The default is 0.5 seconds; measure the appropriate interval for your
mechanism. Motor output is held at zero during that interval, including direct
`setPowers()` calls. This is a timed readiness estimate, not physical position
feedback. Localization continues updating while follower movement is held.

Use localization that remains valid in both physical modes. `TankDriveEncoders`
alone cannot observe holonomic travel; independent tracking/pods can avoid that
kinematic dependency when they maintain contact and their mounting is unchanged.
Mode-specific localization calibration is separate from these follower profiles.
