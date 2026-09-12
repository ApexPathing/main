# Tank deceleration diagnosis and fixes

## What the measurements showed

September 12, 2026. All experiments use FTCodeSim tank physics, the actual AutoTest route,
20 ms follower updates, and physics steps no larger than 5 ms. Results are simulation evidence,
not a physical robot calibration. Original tuning is retained in
`TeamCode/src/test/resources/tank-accuracy-constants.json`; measured tuning output is in
`tank-tuned-constants.json`. Ramsete B/zeta and maximum velocity/acceleration limits are unchanged.

The original kA tuner scored acceleration only. Its chosen translation kA (0.005665) tracked
its positive acceleration ramp closely but left 1.58–1.86 in/s mean overspeed during unscored
braking, in both travel directions. This is 9.6% below the simulator's known inertia (0.006267).
The existing kS/kV ramp also failed its tank physics regression: kS=0.10145 versus 0.06.
A two-parameter fit to accelerating motion was attributing inertial power to static friction.

## Sequential experiments

RMSE values below are speed errors in in/s, against the controller's own target. Interior
samples exclude the last eight inches, targets at/below the 6 in/s floor, and acceleration
magnitudes at/below 5 in/s². Values are sample-weighted. Endpoint metrics use the last eight
inches. Desktop timing causes variation, so small differences are not treated as conclusive.

| Experiment | Acceleration RMSE | Interior braking RMSE | Endpoint braking RMSE | Completion speed (in/s) |
| --- | ---: | ---: | ---: | ---: |
| Fresh original baseline, two runs | 2.52–2.55 | 2.32–2.39 | 6.54–6.77 | 11.84–14.19 |
| Change only kA to physical model value | 1.33–1.34 | 1.31–1.39 | 5.51–6.24 | 15.48–15.59 |
| Then change only kS to physical model value | 1.42–1.43 | 1.35–1.44 | 6.01–6.61 | 13.18–13.68 |
| Revised kA tuner output, original other settings | 1.20–1.21 | 1.39–1.46 | 4.34–4.91 | 13.72–15.20 |
| Full corrected tuner sequence, original follower | 0.94 | 1.27 | 6.38 | 16.06 |
| Whole-path distance interpolation experiment, two runs | 0.74–0.79 | 2.06–2.09 | 3.91–4.12 | 12.96–14.88 |
| Then preserve stronger braking during capture | 0.74–0.90 | 2.00–2.06 | 0.69–0.96 | 5.09–7.23 |
| Then rerun velocity-feedback tuner | 0.95 | 1.69 | 0.92 | 6.70 |
| Retain terminal-only interpolation and settled completion, two runs | 0.89–1.02 | 1.23–1.28 | 1.28–1.41 | 6.71–7.51 |

The physical-value substitutions were diagnostic only and were restored. The retained values
come from running the actual tuner state machines. Whole-path interpolation was rejected after it worsened interior braking and regressed cross-track accuracy with the original tuning. Restricting the correction to the final segment preserves the existing curve behavior and improves all three regions with measured tuning. Final verification also checks completion velocity, rather than only pose accuracy.

## Retained implementation

1. **Tank kS/kV ramp:** power rises and then falls. The fit includes inertial power as a nuisance
   term: `power = kS + kV*v + kA*a`. It integrates delivered power and raw velocity over 100 ms
   windows and uses velocity change over the same interval, avoiding per-sample differentiation.
   Acceptance requires evidence from acceleration and braking and independent regression
   variables. The inertia estimate is logged; this phase still applies only kS/kV. Holonomic
   drives retain the existing ramp. Both tank axes recovered the known simulated coefficients.
2. **kA tuner:** scores the central 12–90% of both ramps in both directions. Braking error is
   sign-aligned for binary-search decisions. Each direction must supply enough acceleration
   and braking samples. Confirmation still measures the worse direction's time-weighted RMS.
3. **Tank terminal distance lookup:** uses `v² = v0² + 2*a*distance` in the final interval ending at rest, with consistent heading rates. Interior curve interpolation is retained. Previously velocity and acceleration were blended independently: near the
   final stop one speed transition implied -217 in/s² while the logged target was -116 in/s².
   Time-based turn lookup and existing holonomic lookup behavior remain unchanged.
4. **Endpoint braking:** analogous to the holonomic braking override, capture does not weaken
   a profile command that is braking an overspeed robot. The new left/right command and blend
   CSV columns exposed a case where -0.62 requested braking became approximately -0.11.
5. **Tank completion:** the existing settled-velocity thresholds (8 in/s and 0.25 rad/s) must be met. Pose dwell alone
   no longer permits a tank path to complete while still moving above that threshold.

## Reproduction

`TankTuningWorkflowSimulationTest.measuredTuningImprovesTankTracking` runs kS/kV, kA and velocity
feedback tuners in sequence, then runs AutoTest on fresh hardware using the resulting JSON.
It uses the old limits and position-controller settings, does not inject ideal feedforward
coefficients, and isolates all tuning storage. `retunedFeedbackTracksConsistentProfile` checks
the final feedback stage independently against the recorded measured feedforward values.
`TankAutoAccuracySimulationTest.tunedAutoTestBrakesAndSettles` repeats the calibrated route and
checks acceleration, interior braking, endpoint braking, cross-track accuracy and finish speed.

Run these with `:TeamCode:testDebugUnitTest --tests <class or class.method>`. The local experiment
uses `-I work/tank-accuracy.init.gradle` to keep build outputs separate from the interactive
simulator's locked JARs. Raw experiment traces and reports are retained in `work/tank-deceleration`.


## Final verification and simulator settings

All 45 distinct targeted checks passed across the final verification runs. This includes both
kA simulations, both kS/kV simulations, a fresh complete tank tuner workflow, the holonomic
AutoTest, original-tuning tank routes, repeated newly tuned tank routes, offset/curve/turn/reverse
tracking, and lookup/profile regressions. Combined-run failures exposed test-global constants
leaking into the next drivetrain test and a skipped terminal log sample; both were corrected
and the affected sequence passed on rerun. No acceptance limits were relaxed.

The fresh complete tuner workflow measured acceleration/interior-braking/endpoint-braking speed
RMSE of 0.723/1.038/0.533 in/s. Two follow-up routes using its saved output measured
0.746–0.807/0.915–0.941/0.704–0.745 in/s, with peak cross-track error 0.659–0.679 inches and
completion speed 7.72–7.77 in/s. Completion meets the existing 8 in/s threshold; this is not a
claim of being motionless at handoff. The outbound takes approximately 7.45 seconds versus
7.09–7.13 seconds for the original baseline.

The measured output was installed in `TeamCode/build/ftcodesim-tank-data/constants.json` after
checking that the live file still matched the original. The previous file is retained as
`constants.before-deceleration-20260912_112827.json` in that directory. Restart the tank simulator
to load the changed code and tuning. The ordinary mecanum simulator tuning file was not changed.
The fresh tuner selected translation velocity feedback 0.10 and angular velocity feedback
0.1875; these are measured tuner outputs, not Ramsete gain changes. Translation kS/kV/kA are
0.0593086/0.0125603/0.00654420; angular kS/kV/kA are 0.0600457/0.0877097/0.0454658.
Position-controller gains and all velocity/acceleration limits retain their original values.
