package org.firstinspires.ftc.teamcode.tuning.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.Map;
import java.util.Objects;
import java.util.concurrent.TimeUnit;

import localizers.Localizer;
import util.MotorMetaData;
import util.Pose;


/**
 * OpMode for automatically determining the direction and position of each motor on a mecanum drivetrain.
 * The robot will run each motor individually while measuring the resulting movement with the localizer,
 * then use that data to determine which motor is which and whether any motors are wired backwards.
 *
 * @author Joel - 7842 Browncoats Alumni
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@TeleOp(name = "Auto Motor Direction + Assignment", group = "Apex Pathing Tuning")
public class AutoMotorDirectionAndAssignment extends LinearOpMode {

    // --- Constants & Lookups ---
    private static final Map<WheelTendencies, WheelPos> MOTOR_LUT = Map.of(
            new WheelTendencies(MovementDirection.NORTH_EAST, Rotation.CW), WheelPos.FRONT_LEFT,
            new WheelTendencies(MovementDirection.NORTH_WEST, Rotation.CCW), WheelPos.FRONT_RIGHT,
            new WheelTendencies(MovementDirection.NORTH_WEST, Rotation.CW), WheelPos.BACK_LEFT,
            new WheelTendencies(MovementDirection.NORTH_EAST, Rotation.CCW), WheelPos.BACK_RIGHT
    );

    // --- Hardware & State ---
    private DcMotorEx m0, m1, m2, m3;
    private DcMotorEx[] motorArray;
    private Localizer localizer;
    private TuningState state = TuningState.POSITIVE_POWER;

    // region Main Loop
    @Override
    public void runOpMode() {
        localizer = new Constants().buildOnlyLocalizer(hardwareMap, Pose.zero());

        // TODO: Replace with either user-defined names or automatic XML reader
        m0 = new MotorMetaData("m0").build(hardwareMap);
        m1 = new MotorMetaData("m1").build(hardwareMap);
        m2 = new MotorMetaData("m2").build(hardwareMap);
        m3 = new MotorMetaData("m3").build(hardwareMap);

        motorArray = new DcMotorEx[]{m0, m1, m2, m3};

        Map<WheelPos, DcMotorEx> assignedMotors = new java.util.EnumMap<>(WheelPos.class);
        String[] telemetrySummary = new String[motorArray.length];
        EnhancedTimer timer = new EnhancedTimer();
        int currentMotorIndex = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (currentMotorIndex < motorArray.length) {
                telemetry.addLine("--- Tuning Complete ---");
                for (String result : telemetrySummary) {
                    if (result != null) {
                        telemetry.addLine(result);
                    }
                }
                telemetry.update();
                // TODO: Overwrite the motor in the XML file with the metadata
                continue; // Skip the rest of the loop once we're done tuning all motors
            }

            DcMotorEx motor = motorArray[currentMotorIndex];

            // Update localizer at the start of every loop
            localizer.update();

            switch (state) {
                case POSITIVE_POWER:
                    // Reset pose and timer once, then move to a waiting state
                    localizer.setPose(new Pose(0, 0, 0));
                    motor.setPower(0.6);
                    timer.setTarget(750);
                    state = TuningState.WAIT_POSITIVE;
                    break;

                case WAIT_POSITIVE:
                    if (timer.isFinished()) {
                        state = TuningState.CALCULATE;
                    }
                    break;

                case CALCULATE:
                    // Measure the results
                    MovementDirection direction = MovementDirection.fromAngle(localizer.getPose().getPositionComponent().getTheta());
                    Rotation rotation = (localizer.getPose().getHeading() > 0) ? Rotation.CCW : Rotation.CW;

                    // Identify the wheel based on behavior
                    WheelTendencies detected = new WheelTendencies(direction, rotation);
                    WheelPos pos = MOTOR_LUT.get(detected);
                    boolean needsReversing = false;

                    // Fallback: Check if the motor is wired backwards
                    if (pos == null) {
                        pos = MOTOR_LUT.get(detected.getOpposite());
                        if (pos != null) {
                            needsReversing = true;
                        }
                    }

                    // Assign and report
                    if (pos != null) {
                        if (needsReversing) {
                            motor.setDirection(DcMotorEx.Direction.REVERSE);
                        }
                        assignedMotors.put(pos, motor);
                        telemetrySummary[currentMotorIndex] = "Motor m" + currentMotorIndex + " is " + pos.name() + (needsReversing ? " (REVERSED)" : " (FORWARD)");
                    } else {
                        telemetrySummary[currentMotorIndex] = "Motor m" + currentMotorIndex + " ERROR: No match found.";
                    }

                    // Cut power and prepare to wait for deceleration
                    motor.setPower(0);
                    state = TuningState.FIRST_DECELERATE;
                    break;

                case FIRST_DECELERATE:
                    if (localizer.getVelocity().getPositionComponent().getMagnitudeSquared() < 0.5) {
                        state = TuningState.NEGATIVE_POWER;
                    }
                    break;

                case NEGATIVE_POWER:
                    // Apply reverse power and move to a waiting state
                    motor.setPower(-0.6);
                    timer.setTarget(750);
                    state = TuningState.WAIT_NEGATIVE;
                    break;

                case WAIT_NEGATIVE:
                    if (timer.isFinished()) {
                        motor.setPower(0.0);
                        state = TuningState.SECOND_DECELERATE;
                    }
                    break;

                case SECOND_DECELERATE:
                    if (localizer.getVelocity().getPositionComponent().getMagnitudeSquared() < 0.5) {
                        // We finished this motor! Increment the index and reset the state.
                        currentMotorIndex++;
                        state = TuningState.POSITIVE_POWER;
                    }
                    break;
            }

            // Show live tuning progress on the driver station
            telemetry.addData("Identifying Motor", "m" + currentMotorIndex + " / " + motorArray.length);
            telemetry.addData("Current State", state);
            telemetry.update();
        }
    }
    // endregion

    // region Enums and Helper Classes
    public enum MovementDirection {
        NORTH_EAST(-Math.PI / 4.0),
        SOUTH_EAST(-3.0 * Math.PI / 4.0),
        SOUTH_WEST(3.0 * Math.PI / 4.0),
        NORTH_WEST(Math.PI / 4.0);

        private final double angle;

        MovementDirection(double angle) {
            this.angle = angle;
        }

        public static MovementDirection fromAngle(double targetAngle) {
            MovementDirection closest = null;
            double minDifference = Double.MAX_VALUE;

            for (MovementDirection dir : MovementDirection.values()) {
                double diff = Math.abs(targetAngle - dir.angle);

                if (diff > Math.PI) {
                    diff = 2 * Math.PI - diff;
                }

                if (diff < minDifference) {
                    minDifference = diff;
                    closest = dir;
                }
            }
            return closest;
        }

        public double getAngle() {
            return this.angle;
        }
    }

    private enum WheelPos {
        FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
    }

    private enum TuningState {
        POSITIVE_POWER,
        WAIT_POSITIVE,
        CALCULATE,
        FIRST_DECELERATE,
        NEGATIVE_POWER,
        WAIT_NEGATIVE,
        SECOND_DECELERATE
    }

    private static class EnhancedTimer extends ElapsedTime {
        private int target = -1;
        public void setTarget(int ms) {
            reset();
            target = ms;
        }
        public boolean isFinished() {
            return target - time(TimeUnit.MILLISECONDS) <= 0 && target > 0;
        }
    }

    private static class WheelTendencies {
        public final MovementDirection direction;
        public final Rotation rotation;

        public WheelTendencies(MovementDirection direction, Rotation rotation) {
            this.direction = direction;
            this.rotation = rotation;
        }

        public WheelTendencies getOpposite() {
            MovementDirection oppDir =
                    direction == MovementDirection.NORTH_EAST ? MovementDirection.SOUTH_WEST :
                            direction == MovementDirection.NORTH_WEST ? MovementDirection.SOUTH_EAST :
                                    direction == MovementDirection.SOUTH_WEST ? MovementDirection.NORTH_EAST :
                                            MovementDirection.NORTH_WEST;

            Rotation oppRot = rotation == Rotation.CW ? Rotation.CCW : Rotation.CW;
            return new WheelTendencies(oppDir, oppRot);
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (!(o instanceof WheelTendencies)) return false;
            WheelTendencies that = (WheelTendencies) o;
            return direction == that.direction && rotation == that.rotation;
        }

        @Override
        public int hashCode() {
            return Objects.hash(direction, rotation);
        }
    }
    // endregion
}