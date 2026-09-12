# Apex Pathing
Check out our site at https://www.apexpathing.com/ for info about the project!

## Run the OpModes in FTCodeSim

The shared simulator setup is in `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim`.
It registers the four drivetrain motors plus the custom Apex Pinpoint and telemetry adapters used
by all three current OpModes. Install and open
[AdvantageScope 26.0.2](https://github.com/Mechanical-Advantage/AdvantageScope/releases/tag/v26.0.2)
at least once before launching FTCodeSim. In PowerShell, run:

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest `
  --tests org.firstinspires.ftc.teamcode.sim.SimulateApexPathing
```

Before running it, set `RUN_INTERACTIVE_SIMULATOR` to `true` in
`SimulateApexPathing.java`. Set it back to `false` afterward so ordinary unit-test runs do not wait
indefinitely for the interactive simulator windows to close.

The simulated Driver Station presents `Apex Auto Test`, `Apex TeleOp Test`, `Follower Tuner`, and
`Localization Tuner`.
Select one, then press Init and Start; FTCodeSim initializes and runs only that selected OpMode.
Stop it before selecting another. This is an interactive JUnit test and remains active until the
simulator windows are closed. Tuner output from a simulation is stored under
`build/ftcodesim-data` instead of the Android device's external storage.

FTCodeSim's original keyboard controls are preserved: arrows are the D-pad, WASD is the left
stick, `;` is gamepad A, `[` is B, `P` is X, and `-` is Y. Follower Tuner always opens its phase
options menu and requires `;` to accept the highlighted phase. Use D-pad Left/Right to choose
automatic or manual mode, A to select or advance, and X to run tests. Pressing Start first keeps
the menu open until a phase is explicitly selected. After Start, the sticks drive the robot
field-centrically on menus, prompts, results, and idle manual-tuning screens; tuner-controlled
motion takes exclusive control until its test finishes. All phases remain selectable for retuning.
After a phase's results are accepted, Follower Tuner saves them and advances through every
remaining phase in order. Feedforward now runs first and uses Road Runner-style slow power ramps
to fit moving kS and kV together, separately for forward motion and counterclockwise turning.
Press A to start each ramp from rest. Power rises by 0.1 per second up to 0.9; X stops early for
review. Automatic travel cutoffs are 96 inches from the drive start or two turns, but the operator
must stop before obstacles and allow braking room. Controllers are disabled during measurement.
Use the sticks between runs to reposition. Review kS, kV, R-squared and sample count. Y excludes
the largest residual, B restores exclusions, X discards/retries the ramp, and A accepts a valid fit.
Acceptance requires at least 20 moving samples, a velocity span of 2 in/s (drive) or 0.2 rad/s
(turn), R-squared >= 0.90, kV > 0 and 0 <= kS < 0.9. Both axes must be accepted before constants
are applied. kA is left unchanged. The old breakaway-power stage and feedforward binary searches
have been removed; existing position-controller breakaway settings remain separate from moving
feedforward kS. Fits use Apex's normalized motor-power units and no battery-voltage compensation:
kS is power, drive kV is power/(in/s), and turn kV is power/(rad/s). Tune with a representative
battery. The simulation asks you to use the red Stop button only after the final Velocity Feedback
phase is complete.

Automatic PDS tuning uses repeated bounded point-to-point tests and central finite differences to
refine kP and kD from generic starting guesses. It runs immediately after feedforward ramp tuning
and before movement-limits tuning. It scores time-weighted squared position error, backs off after
worse updates, and restores the best measured gains before operator validation. PDS and feedforward runs save
graph-ready CSV files beside `constants.json` (`FIRST/ApexPathing` on the Robot Controller and
`build/ftcodesim-data` in desktop simulation). Feedforward ramp CSV rows include axis, time,
measured velocity, preceding applied power and sample eligibility. Separate accepted-fit CSVs
include the exclusion mask and fitted coefficients. PDS CSV rows include gains, trial cost, target,
position, error, velocity, commanded power, and safeguard status over time.

## Localization tuner

Run `Localization Tuner` before follower tuning. It follows the same phase-picker pattern as the
follower tuner and uses a consistent Prepare, Record, Review lifecycle. The available procedures
are a hardware check, forward and strafe distance scale, rotation geometry, velocity/acceleration
filtering, and a free-drive validation loop. Phases that do not apply to the configured drivetrain
or localizer are shown as `N/A`.

Distance procedures drive at fixed low power in both directions and ask for the physical distance
measured on the floor. Rotation uses a hardware-map IMU named `imu` by default. Its relative
quaternion measurement does not require the hub mounting orientation; Dpad Left/Right selects a
manual reference if the IMU is unavailable or broken. Manual rotation disables forward and strafe
input. Filter tuning lets the operator choose adaptive Kalman or moving average. Kalman collection
includes still, free-drive, and stopped intervals, then displays the learned R/Q values and records
a CSV beside the constants files. The tuner reports evidence and leaves the choice to the operator.

Accepted results are saved immediately to `FIRST/ApexPathing/localization.json`, with a backup at
`localization.json.bak`. Follower constants remain in `constants.json`. A DualActuated robot shares
one localizer across both modes by default. Override `usesSharedLocalizer()` and
`localizerConstants(Profile)` in the robot's `ApexConstants` implementation when tank and
holonomic modes need separate localizers.
