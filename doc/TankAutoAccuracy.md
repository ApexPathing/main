# Tank AutoTest accuracy experiment

The tank path follower now preserves requested turn power and clamps forward power to the
remaining motor budget. Previously, shared wheel normalization reduced turn power whenever
translation and steering together exceeded that budget. This change applies to tank path
following, including reverse and endpoint capture; point turns use their existing executor.

Ramsete B=8.0 and zeta=0.7, maximum velocity, and deceleration remain unchanged.
Tank cross-track diagnostics now report signed lateral distance to the nearest path tangent;
the previous implementation left this value at zero, masking tracking error in AutoTest.

## Measurement

Run on September 12, 2026 using FTCodeSim tank physics, the actual TankAutoTest outbound,
turn and reverse route, and the fixed tuning snapshot in
`TeamCode/src/test/resources/tank-accuracy-constants.json`. The test uses fresh hardware and
isolated storage on each run. Saved interactive tuning is not overwritten. Localization uses
the simulator's ground-truth Pinpoint adapter; these results do not establish physical-robot
accuracy. Physics runs in at most 5 ms steps and AutoTest controls at approximately 20 ms.

Outbound cross-track metrics are calculated from the logged samples (RMS is sample-weighted).

| Configuration | Peak error (in) | RMS error (in) | Outbound time (s) |
| --- | ---: | ---: | ---: |
| Original allocation, corrected diagnostic | 4.743 | 1.603 | 7.343 |
| Steering priority | 1.691 | 0.826 | 7.062 |
| Steering priority + braking limit at 60% | 1.70 | 0.76 | 7.430 |
| Steering priority repeat 1 | 1.799 | 0.831 | 7.156 |
| Steering priority repeat 2 | 1.504 | 0.718 | 7.108 |

The original outbound trace saturated motor output in 48% of samples. Steering priority
substantially improves accuracy without reducing maximum velocity. Earlier braking offered
little peak-error improvement and took longer, so that experiment was reverted. Timing and
sampled peaks vary with desktop scheduling; the small runtime difference is not a speed claim.

## Regression checks

`TankAutoAccuracySimulationTest` runs the complete route twice, requires all three stages to
pass, and enforces outbound peak error below 2.2 inches and RMS below 1 inch, with a 12-second
whole-route runtime budget. A nonzero measurement assertion prevents silently returning to
empty cross-track diagnostics. `TankPathFollowingSimulationTest` additionally verifies the
known 4-inch offset, corrective rotation, and curve/turn/reverse completion.

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest `
  --tests org.firstinspires.ftc.teamcode.sim.TankAutoAccuracySimulationTest `
  --tests org.firstinspires.ftc.teamcode.sim.TankPathFollowingSimulationTest `
  --tests ExampleAutoProfileRegressionTest `
  --tests PathProfileConvergenceTest
```

If another simulator holds build JARs open, run with a Gradle init script that sets
`allprojects { layout.buildDirectory = file("build/tank-accuracy") }` and pass its path with
`-I`. Test reports then appear below `TeamCode/build/tank-accuracy/reports/tests/`.

Final verification after test-fixture isolation and the known-offset assertion: all 14 selected
tests passed (zero failures/errors). The two final outbound runs measured 1.481/1.645 inches
peak error and 0.716/0.774 inches RMS error, completing the whole route in 8.892/8.953 seconds.
