package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Finds TURRET_MIN_POSITION and TURRET_MAX_POSITION. Only the turret servo is used.
 *
 * <p>When START is pressed the turret moves to TURRET_HOLD_POSITION. Gamepad1 d-pad left/right
 * then moves it -/+ TURRET_LIMIT_TEST_STEP (0.01, about 4.0 deg of turret at the measured
 * ~403 deg per 1.0 of servo position) per press, within 0-1. This test deliberately ignores the current TURRET_MIN/MAX_POSITION so it can find them.
 * The servo is relaxed (PWM off) when the OpMode stops.
 */
@TeleOp(name = "Turret Limit Test", group = "Test")
public class TurretLimitTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        ServoImplEx turret = hardwareMap.get(ServoImplEx.class, "turret");
        RobotHardware.configureTurretServo(turret);

        double position = Constants.TURRET_HOLD_POSITION;

        telemetry.addLine("Turret Limit Test: turret moves to "
                + Constants.TURRET_HOLD_POSITION + " when START is pressed.");
        telemetry.addLine("Make sure it can turn freely and cables have slack.");
        telemetry.addLine("GP1 d-pad left/right: -/+ " + Constants.TURRET_LIMIT_TEST_STEP);
        telemetry.update();

        waitForStart();

        try {
            if (isStopRequested()) return;
            turret.setPwmEnable();
            turret.setPosition(position);

            boolean leftPreviouslyPressed = false;
            boolean rightPreviouslyPressed = false;
            while (opModeIsActive()) {
                double next = position;
                if (gamepad1.dpad_left && !leftPreviouslyPressed) {
                    next -= Constants.TURRET_LIMIT_TEST_STEP;
                }
                if (gamepad1.dpad_right && !rightPreviouslyPressed) {
                    next += Constants.TURRET_LIMIT_TEST_STEP;
                }
                leftPreviouslyPressed = gamepad1.dpad_left;
                rightPreviouslyPressed = gamepad1.dpad_right;

                // Round to avoid drift like 0.6499999 after repeated steps; stay within 0-1.
                next = Math.max(0.0, Math.min(1.0, Math.round(next * 1000.0) / 1000.0));
                if (next != position) {
                    position = next;
                    turret.setPosition(position);
                }

                telemetry.addData("Turret position", "%.3f", position);
                telemetry.addData("Est. turret angle from hold (deg)", "%.1f",
                        (position - Constants.TURRET_HOLD_POSITION)
                                * Constants.TURRET_DEG_PER_SERVO_UNIT);
                telemetry.addData("Current constants", "MIN %.3f / MAX %.3f",
                        Constants.TURRET_MIN_POSITION, Constants.TURRET_MAX_POSITION);
                telemetry.addLine("D-pad left: -" + Constants.TURRET_LIMIT_TEST_STEP
                        + "   D-pad right: +" + Constants.TURRET_LIMIT_TEST_STEP);
                telemetry.update();
                idle();
            }
        } finally {
            turret.setPwmDisable();
        }
    }
}
