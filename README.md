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

The simulated Driver Station presents `Apex Auto Test`, `Apex TeleOp Test`, and `Follower Tuner`.
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
remaining phase in order. Manual feedforward verification drives forward 48 inches and then turns
180 degrees, reporting separate drive and angular velocity RMS errors. The simulation asks you to
use the red Stop button only after the final Velocity Feedback phase is complete.

Automatic PDS tuning uses repeated bounded point-to-point tests and central finite differences to
refine kP and kD from generic starting guesses. It runs immediately after breakaway-power tuning
and before movement-limits tuning. It scores time-weighted squared position error, backs off after
worse updates, and restores the best measured gains before operator validation. PDS and feedforward runs save
graph-ready CSV files beside `constants.json` (`FIRST/ApexPathing` on the Robot Controller and
`build/ftcodesim-data` in desktop simulation). Feedforward CSV rows include measured velocity and
acceleration plus fitted power and residuals; PDS CSV rows include gains, trial cost, target,
position, error, velocity, commanded power, and safeguard status over time.
