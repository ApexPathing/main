# Corner tracking investigation

## Follow-up: overshoot reproduced after retuning and corrected

The user confirmed the regression was visible in FTCodeSim after rerunning the
tuner. The earlier geometry-only correction and fresh-tuning completion check
were insufficient evidence that corner tracking was fixed.

For this follow-up, the latest saved simulator constants were frozen before
testing (`src/test/resources/corner-tracking-constants.json`). Their SHA-256 is
`08197DB44D8EA36230C592B432178997418C1ABED732F4506BBC8B71243006D8`.
The before/after runs below use those identical constants; there is no retuning
between them. The user's saved `build/ftcodesim-data/constants.json` was not changed.

Three controller problems were identified:

1. Path heading damping used `-measuredAngularVelocity`, treating a moving
   tangent-heading target as stationary. Even with zero heading error and a
   perfectly matched turn rate of 2 rad/s, the saved D gain requested about
   1.54 power opposing that turn. A regression reproduced this failure before
   the fix. Damping now uses target rate minus measured rate; profiled paths
   use the bounded angular target already used by velocity tracking.
2. After feedforward cancellation, residual overspeed braking was allocated
   after heading/cross-track/centripetal corrections. A saturated correction
   stage left no power for that brake. Residual braking now joins the corrective
   stage so it cannot be erased merely because steering consumes that stage.
3. The follower omitted the negative acceleration feedforward that the planned
   deceleration needed. Corner-entry measurements reached roughly 28 in/s
   against a 6.6 in/s target. The planner and follower now both use the existing
   signed acceleration coefficient for braking. No extra braking gain was added.

Applying full braking at every spatial lookup position can itself stop the robot
before it advances to the next row. Therefore braking acceleration is scaled by
the square of the achieved/target speed ratio, clamped to [0, 1]. It vanishes at
rest, retains the planned value at target speed, and does not grow above the
planned value during overspeed. Launch acceleration remains available at rest.
This is applied to both holonomic and tank spatial following.

| Frozen latest tuning | Before this follow-up | After controller correction |
| --- | ---: | ---: |
| Peak error over entire outbound path (in) | 2.798 | 0.644 |
| Peak error on curves (in) | 2.798 | 0.303 |
| Curve RMS error (in) | 1.337 | 0.158 |
| Peak curve heading error (deg) | 64.51 | 16.14 |

Trace files are in `build/corner-followup`:

- Before: `auto-test-outbound-velocity_20260908_214911_126.csv`
- After: `auto-test-outbound-velocity_20260908_215434_406.csv`

The corrected run completed all five movements in 14.39 seconds, inside the
existing 18-second limit. A separate frozen-fixture regression repeated the
result: 0.624 in outbound peak, 0.304 in curve peak, 16.03 degrees heading peak,
and 14.36 seconds total. Braking velocity RMS was 5.44 in/s on that repeat.

All 38 focused tests passed, covering follower vectors, geometry, controllers,
centripetal tuning geometry, default AutoTest, and the new saved-tuning corner
regression. The latter checks corner exits and the final straight as well as
curved samples, and fails for the reproduced baseline errors. Tests run in an
isolated build directory because the user's running simulator locks the normal
build jars.

The complete automatic tuner subsequently finished and saved a new set of
constants in the separate `build/tuner-e2e-data` directory. The combined test
then encountered `ClassNotFoundException` because compiled test classes had
disappeared before its AutoTest step. After rebuilding outside the normal
build-clean directories (`work/corner-followup-build`), AutoTest passed with
those newly generated constants: **13.86 seconds**, acceleration peak overspeed
**4.36 in/s**, braking RMS **3.04 in/s**. These satisfy the remaining combined
test's velocity bounds (peak <10, braking RMS <8). The final 38-test run also
passed, including another frozen-fixture repeat (0.301 in curve peak).

Stop, rebuild, and relaunch FTCodeSim to load the changed follower. Keep the
current saved simulator tuning for the first comparison; the primary regression
was verified with that exact tuning, not a replacement set.

The geometry initialization fix below remains in place, but it was not a
sufficient fix for the reported overshoot. The following sections record that
earlier investigation and its limitations.

## Confirmed defect

`PathSegment` constructed its lookup table before assigning `this.length`.
While constructing each row it called `getCurvatureDerivative(t)`, which reads
that field to select a finite-difference window. Java's initial field value is
zero, so every cached derivative used the maximum window, `dt = 0.05`.
After construction, the same method uses `dt = 0.1 / length` (subject to clamps).
The planner therefore received different curvature derivatives from runtime
evaluation of the same geometry at the same parameter.

This is not a centripetal sign or units error. Curvature derivative feeds the
angular acceleration term `alpha = curvatureDerivative * v^2 + curvature * a`
for tangent heading, and therefore affects speed/power planning at corner
entrances and exits. The centripetal tuner measures the middle of a nearly
constant-curvature 32-inch arc, where this error is much less exposed.

The fix calculates the complete distance table and assigns length before
constructing the curvature-bearing lookup rows. Path points, distance keys,
and curve shape remain the same. No tuned gain was manually changed.

## Reproduction and validation

The new `cachedCurvatureDerivativeMatchesRuntimeGeometry` regression failed
before the fix and passed after it. A second regression covers the exact
outbound AutoTest waypoint geometry, including its four ArcPose corners.

AutoTest is `AutoTest.java` in this checkout (OpMode name `Apex Auto Test`).
The following are simulator measurements, not physical robot measurements.
Curve statistics include outbound CSV samples with `abs(curvature) > 0.01 / in`.

| Same saved tuner constants | Curve cross-track RMS (in) | Peak (in) | Heading RMS (deg) | Peak (deg) | All five movements |
| --- | ---: | ---: | ---: | ---: | --- |
| Before geometry fix | 0.442 | 1.070 | 35.84 | 58.37 | Pass |
| After geometry fix | 1.156 | 2.406 | 41.84 | 65.42 | Pass |

Traces in `build/ftcodesim-data`:

- Before: `auto-test-outbound-velocity_20260908_212926_550.csv`
- After: `auto-test-outbound-velocity_20260908_213259_260.csv`

These results prove the geometry mismatch was corrected, but **do not show a
corner-tracking improvement with the old constants**. Completion alone is not
evidence that the drift was solved. The initialization defect must not be
presented as the proven sole cause of the observed robot behavior.

A separate experiment giving translation and heading feedforward a shared
power budget also worsened tracking and was reverted. Existing user edits
to the follower, feedforward tuner, and AutoTest were preserved.

## Fresh tuning workflow

`TunedAutoTestSimulationTest` passed after the fix (4 minutes 30 seconds for
the complete Gradle run). It runs every automatic tuner phase, loads the
resulting constants, completes all five AutoTest movements within the existing
18-second limit, and checks acceleration/braking velocity tracking.

The resulting trace is
`build/tuner-e2e-data/auto-test-outbound-velocity_20260908_213821_693.csv`.
On 104 curved samples, cross-track RMS was **0.123 in**, peak **0.428 in**;
heading RMS was **21.65 degrees**, peak **34.17 degrees**.
The freshly tuned centripetal gain was `0.0023242250309941214`, compared with
`0.007709420006964843` in the older saved simulator constants.

This is an encouraging end-to-end result, but changes to multiple tuned
constants mean it is not an isolated measurement of the geometry fix's effect.
The controlled old-constants regression above remains relevant: regenerate
profiles and retune before assessing this change on the robot. No simulator
constants were copied into robot configuration.

The final focused run passed all 26 tests in `PathSegmentTest`,
`CentripetalPhaseTest`, and `FollowerVectorTest`. The exact AutoTest corner
geometry now has planner/runtime derivative agreement at every lookup row.
