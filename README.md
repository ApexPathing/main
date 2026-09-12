# Apex Pathing
Check out our site at https://www.apexpathing.com/ for info about the project!

Use `GeometryFactory.tankPath(...)` or `GeometryFactory.holonomicPath(...)` to declare a path's
drive mode explicitly. The factory no longer chooses a builder from the drivetrain's current
state. On a dual-actuated drive, `Follower.follow(...)` switches hardware and tuning profiles to
match the built path; building the path itself does not actuate hardware. Turns can select their
profile with `setDriveProfile(...)`.

`AutoTest.java` defines separate tank and holonomic routes. `TankAutoTest` selects the tank route
and hardware, then runs the same `AutoTest` execution loop. There is no conditional factory path API.

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

### Tank-drive simulation

Run `SimulateTankApexPathing.tuneAndTestTankPathFollowing` as a JUnit test to launch FTCodeSim
with differential-drive physics. The simulated Driver Station adds `Tank Localization Tuner`,
`Tank Follower Tuner`, and `Tank Apex Auto Test` under `Apex Tank Simulation`; run them in that
order. Tank tuning is stored separately under `build/ftcodesim-tank-data`, so it does not replace
the mecanum simulator's saved constants.

Do not select these tank-only OpModes from `SimulateApexPathing`; that launcher uses mecanum
physics. The tank OpModes detect the wrong launcher, and `Tank Apex Auto Test` also checks that
positive motion limits were saved by the follower tuner before building profiled movements.

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest `
  --tests org.firstinspires.ftc.teamcode.sim.SimulateTankApexPathing
```

The tank simulator uses the drivetrain-backed Pinpoint adapter for localization because the
current FTCodeSim `SimMotor` reports no encoder position. Drivetrain motion is produced by
`SimulatedTank`, including its four tank motor channels and differential-drive kinematics.
The tank model uses 2-inch-radius wheels, a 14-inch track, and duty-independent back EMF
with moving-friction power 0.06, maximum speed 75 in/s, and maximum acceleration 150 in/s².
These are explicit simulation assumptions, not measurements of a physical robot. Rerun follower
tuning after this model change; constants from the previous high-friction model are incompatible.

FTCodeSim's original keyboard controls are preserved: arrows are the D-pad, WASD is the left
stick, `;` is gamepad A, `[` is B, `P` is X, and `-` is Y. Follower Tuner always opens its phase
options menu and requires `;` to accept the highlighted phase. Use D-pad Left/Right to choose
automatic or manual mode, A to select or advance, and X to run tests. Pressing Start first keeps
the menu open until a phase is explicitly selected. After Start, the sticks drive the robot
field-centrically on menus, prompts, results, and idle manual-tuning screens; tuner-controlled
motion takes exclusive control until its test finishes. All phases remain selectable for retuning.
After a phase's results are accepted, Follower Tuner saves them and advances through every
remaining phase in order. Feedforward runs first and uses measured power ramps
to fit moving kS and kV together, separately for forward motion and counterclockwise turning.
Press A to start each ramp from rest. Power rises by 0.1 per second, up to 0.9; X stops early for
review. Automatic travel cutoffs are 96 inches from the drive start or two turns, but the operator
must stop before obstacles and allow braking room. Controllers are disabled during measurement;
there is no automatic return or heading correction. Use the sticks between runs to reposition.
Review kS, kV, R-squared and sample count. Y excludes the largest residual, B restores exclusions,
X discards/retries that ramp, and A accepts a valid fit. Acceptance requires at least 20 moving
samples, a velocity span of 2 in/s (drive) or 0.2 rad/s (turn), R-squared >= 0.90, kV > 0 and
0 <= kS < 0.9. Both axes must be accepted before constants are applied. kA is left unchanged.
The old breakaway-power stage and feedforward binary searches have been removed; existing
position-controller breakaway settings remain separate from moving feedforward kS.
Unlike Road Runner's voltage coefficients, these fits use Apex's existing normalized motor-power
units: kS is power, drive kV is power/(in/s), and turn kV is power/(rad/s). Battery voltage is not
compensated by this tuner or Apex's existing feedforward model; tune with a representative battery.
Tank ramps switch to falling power after one third of the travel allowance. Their fit includes
acceleration as well as velocity, using 100 ms integrated measurement windows, so inertial power
is not mistaken for kS. At least ten accelerating and ten braking windows are required.
The inertia estimate is diagnostic; the following kA phase still validates kA independently.
Holonomic drives retain their original two-parameter ramp fit;
the CSV records delivered motor power after drivetrain limiting.
The simulation asks you to use the red Stop button only
after the final Velocity Feedback phase is complete.

Automatic PDS tuning uses repeated bounded point-to-point tests and central finite differences to
refine kP and kD from generic starting guesses. It runs immediately after feedforward ramp tuning
and before movement-limits tuning. It scores time-weighted squared position error, backs off after
worse updates, and restores the best measured gains before operator validation. PDS and feedforward runs save
graph-ready CSV files beside `constants.json` (`FIRST/ApexPathing` on the Robot Controller and
`build/ftcodesim-data` in desktop simulation). Feedforward ramp CSV rows include axis, time,
measured velocity, preceding applied power and sample eligibility. Separate accepted-fit CSVs
include the exclusion mask and fitted coefficients; graph velocity on X and power on Y.
PDS CSV rows include gains, trial cost, target,
position, error, velocity, commanded power, and safeguard status over time.

Acceleration feedforward runs after movement limits. Its automatic mode needs only A to start the
complete search and A to accept the results. It tests seven bounded kA candidates and confirms the
best one, separately for translation and rotation. Every candidate uses the same conservative
trapezoidal profile in both directions. Signed mean velocity error chooses the next half of the
search interval; the worse direction's time-weighted velocity RMS selects the best candidate.
Both acceleration and braking are scored; cruise remains unscored. Braking errors are sign-aligned with acceleration errors for the search direction. Translation profiles travel 60 inches each way. Angular sweeps are 50% larger than the
original profiles. Both use longer acceleration/braking ramps with a short 0.25-second cruise.
The scoring windows cover 12% through 90% of each acceleration and braking ramp. Peak speeds are capped at
30 in/s for translation, 3 rad/s for tank rotation, and 1.5 rad/s for holonomic rotation, and
remain limited by the measured motion limits. At those caps, translation accelerates for
1.75 seconds and rotation for 1.25 seconds, with the travel distances unchanged.
Manual mode uses Dpad Up/Down to change kA, Dpad Left/Right to change the increment, and A
to run the selected axis. No phase-specific X, Y, B, or bumper controls are used.
The kA search range is capped by both motor-power headroom and twice the inertia estimate from
the measured full-power acceleration limit. Velocity error uses the profile target at the
measurement timestamp. Confirmation permits an RMS increase of 25% or 2% of the test's peak
speed, whichever is larger, so a nearly zero search error does not make repeatability impossible.
Each nominal tank angular sweep is 4.5 radians (about 258 degrees).

## Localization tuner

Run `Localization Tuner` before follower tuning. It follows the same phase-picker pattern as the
follower tuner with compact prompts for the Driver Station. The available procedures
are a direction check, forward and strafe distance scale, rotation geometry, velocity/acceleration
filtering, and a live pose test. Phases that do not apply to the configured drivetrain
or localizer are shown as `N/A`.

The direction check saves the required encoder/pod reversals into localization calibration.
Distance procedures drive at fixed low power in both directions and ask for the physical distance
measured on the floor. Rotation ramps power until it first reaches 90 degrees per second, holds
that discovered power, then cuts power after five counterclockwise rotations. It uses a
hardware-map IMU named `imu` by default. Its relative
quaternion measurement does not require the hub mounting orientation; Dpad Left/Right selects a
manual reference if the IMU is unavailable or broken. Filter tuning lets the operator choose
adaptive Kalman or moving average. Kalman collection
includes still, free-drive, and stopped intervals, then displays the learned R/Q values and records
a CSV beside the constants files. The tuner reports evidence and leaves the choice to the operator.

Accepted results are saved immediately to `FIRST/ApexPathing/localization.json`, with a backup at
`localization.json.bak`. Follower constants remain in `constants.json`. A DualActuated robot shares
one localizer across both modes by default. Override `usesSharedLocalizer()` and
`localizerConstants(Profile)` in the robot's `ApexConstants` implementation when tank and
holonomic modes need separate localizers.
