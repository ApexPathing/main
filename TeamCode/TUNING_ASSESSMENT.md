# AutoTest tuning assessment — 2026-09-08

AutoTest.java was executed through AutoTestSimulationTest using the saved simulator
constants (`APEX_USE_SAVED_TUNER_CONSTANTS=true`), without the bench-feedforward
override. Existing workspace changes were retained. These are simulator results,
not measurements from a physical robot.

## Comparison

| Configuration | Runtime (s) | Braking velocity RMSE (in/s) | Acceleration peak overspeed (in/s) |
| --- | ---: | ---: | ---: |
| Baseline: feedback 0.025, no braking KA | 17.17 | 11.87 | 17.55 |
| Saved braking KA 0.00810694 enabled | 21.64 (fails 18 s limit) | 10.87 | 5.37 |
| Braking KA 0.002 | 17.45 | 8.59 | 11.72 |
| No braking KA, feedback 0.05 | 17.99 | Not recorded | 4.87 |

Braking samples have target acceleration below -5 in/s²; acceleration samples
have target acceleration above +5 in/s². Endpoint-controlled samples are excluded.
Each comparison is one run, so small runtime differences are not significant.

The separate braking gain was removed following review of its modest benefit and
added tuning complexity. No alternative feedback gain was saved. Acceleration KA
is applied only while speeding up, with velocity feedback handling braking.
The legacy `translationBrakeKA` JSON key is ignored on load and no longer emitted
on save. The simulator's saved copy also has that key removed.

## Tuner diagnosis

The latest saved feedforward-validation CSV already exhibits overspeed, before
AutoTest or path feedback is involved:

- Translation ramp: mean error +2.26 in/s, RMSE 2.30 in/s, peak +2.71 in/s.
- Translation hold: mean error +2.56 in/s, RMSE 2.61 in/s, peak +3.62 in/s.
- Both directions passed. The tuner permits RMS error up to 5% of the forward
  speed limit (about 3.23 in/s), pools ramp and hold errors, and has no separate
  bias or peak-overspeed acceptance bound.

`FeedforwardTuner.runHoldingOrValidation` validates only a 0.7 s ramp to 40% of
the speed limit followed by a hold. It does not validate deceleration or repeated
high-speed transitions. `fitSeparated` deliberately excludes braking windows.
The fitted model therefore passes despite persistent overspeed and without
evidence that it generalizes to AutoTest's trajectory.

The holding data also show affine-model approximation error: near 22.9 in/s,
the saved kS + kV*v predicts approximately 0.277 power versus measured holding
power around 0.271. That is consistent with excess holding speed at the nearby
validation target. This supports a model/validation problem; the evidence does
not isolate an incorrectly fitted acceleration KA as the sole cause.

In baseline AutoTest, the largest acceleration-labelled overspeed is inherited
from braking: at 9.646 s, target/actual are 18.23/41.44 in/s while decelerating;
at 9.707 s, target/actual are 17.07/34.61 in/s with positive target acceleration.
Actual velocity is still falling. Treating every positive-target-acceleration
sample as acceleration-induced overshoot would misdiagnose that event.

The velocity-feedback tuner additionally selects on central-profile RMS from
straight paths, excluding the first/last 10% and endpoint blending. Its acceptance
check uses return RMS, directional agreement and saturation; it does not bound
velocity overshoot. This further limits its ability to catch the AutoTest behavior.

## Implemented tuner correction

The holding fit now keeps its fitted slope while lowering kS so that it does not
overpredict the measured holding powers. Acceleration KA is then refitted against
those holding terms. Independent ramp and hold validation each require at least
10 samples, RMS below the existing tolerance, and absolute signed bias below half
that tolerance. One phase can no longer hide another phase's persistent bias.

If a physical candidate fails validation, the tuner can refine kS using the two
directions' measured holding-speed bias (`kS -= kV * meanBias`). It performs at
most two refinements, reruns feedforward-only validation with fresh measurements,
and applies neither axis unless both pass. Telemetry exposes ramp and hold bias.

Velocity-feedback tests now travel 48 inches, giving braking time to affect the
score before endpoint capture. Translational overspeed contributes four times its
squared error to the selection score. Angular scoring is unchanged. No braking KA
was added; braking remains controlled by velocity feedback.

The test harness also had a timing bug: restarting its physics timestamp at every
short pump call discarded wall time between batches. Both AutoTest and the full
tuner test now retain that timestamp between calls. The original table above is
historical and must not be compared directly with corrected-clock runtimes.

| Corrected-clock configuration | Runtime (s) | Acceleration peak overspeed (in/s) | Braking RMS (in/s) |
| --- | ---: | ---: | ---: |
| Original saved tuning | 13.55 | 19.13 | 12.15 |
| Full revised tuner, first complete run | 14.18 | 4.40 | 6.44 |
| Full revised tuner, repeat with regression assertions | 14.10 | 4.27 | 4.88 |

These changes substantially reduce the large overspeed excursion; they do not
promise zero tracking error or establish hardware performance. The complete
workflow test runs every tuning phase, loads its saved output into AutoTest, and
requires runtime below 18 seconds, acceleration peak overspeed below 10 in/s,
and braking RMS below 8 in/s. The experimental saved constants remain separate
from the user's existing simulator constants.

## Verification and artifacts

Run with PowerShell:

```powershell
$env:APEX_USE_SAVED_TUNER_CONSTANTS='true'
./gradlew.bat :TeamCode:testDebugUnitTest --tests core.FollowerVectorTest --tests 'feedforward.*' --tests tuning.FeedforwardTunerTest --tests org.firstinspires.ftc.teamcode.sim.AutoTestSimulationTest

# Full tuner followed by AutoTest, with tracking-quality assertions:
./gradlew.bat :TeamCode:testDebugUnitTest --tests org.firstinspires.ftc.teamcode.sim.TunedAutoTestSimulationTest
```

AutoTestSimulationTest now reports braking bias, RMSE and peak overspeed alongside
the existing acceleration metrics and checks that both phases have trace samples.
Final verification after removal: 23 tests passed. AutoTest completed in 17.21 s
versus the 17.17 s baseline; braking RMSE was 11.84 in/s versus 11.87 in/s.
Baseline constants, trace and experiment logs are in `build/braking-assessment`.
Original tuner validation evidence is in
`build/ftcodesim-data/feedforward_validation_20260908_114054_590.csv`.
Full-workflow output is in `build/tuner-e2e-data`; the first successful corrected
run is preserved in `build/tuner-fix-pass-1`.
The final suite passed all 57 tests, covering tuning, follower vectors, profiles,
OpMode sequences and the complete tuner-to-AutoTest workflow. Restart the simulator
and rerun tuning to use the revised code; the existing saved simulator constants
were preserved. Velocity-feedback tuning now requires a clear 48-inch lane.

## Feedforward simplification experiment — 2026-09-08

The automatic feedforward phase now uses only the active separated model: six holding
points per axis (three speeds in both directions) establish conservative kS/kV, and
dynamic acceleration windows establish kA. The legacy sample-level regression,
cross-validation, R² acceptance/reporting, and the parallel combined acceleration/braking
integral fit were removed. Controlled reverse-power braking data collection was also
removed; every dynamic run now stops and retains the existing settle interval.

CSV and telemetry now report the candidates, holding evidence, acceleration-window
residuals, and independent ramp/hold validation RMS, signed bias, sample counts, and
saturation. The active fit also requires at least four valid acceleration windows per axis.

Removing the four quasistatic runs held up experimentally. Dynamic-only characterization
produced 10 valid acceleration windows per axis and comparable results:

| Configuration | Runs | Runtime (s) | Acceleration peak overspeed (in/s) | Braking RMS (in/s) |
| --- | ---: | ---: | ---: | ---: |
| Earlier corrected tuner reference | 24 | 14.10 | 4.27 | 4.88 |
| Simplified, quasistatic retained | 24 | 14.158 | 4.581 | 4.774 |
| Simplified, dynamic-only | 20 | 14.158 | 4.407 | 4.836 |

The final combined verification repeated the 20-run workflow at 14.046 seconds,
4.571 in/s acceleration peak overspeed, and 6.538 in/s braking RMS. All 27 selected
unit/integration tests passed; the variation remained within the established limits.

The 20-run sequence was retained. No proposed simplification failed this experiment.
Holding-speed coverage, both directions, stopping/settling, independent validation,
bounded bias refinement, all-or-nothing application, manual tuning, simulator timing,
and velocity-feedback behavior remain unchanged. No separate braking KA was introduced.
Experimental output remains under `build/tuner-e2e-data` and does not replace the user's
saved simulator constants.
