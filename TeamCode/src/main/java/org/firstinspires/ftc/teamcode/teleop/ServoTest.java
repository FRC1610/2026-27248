package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Finds the flower wedge up and down positions and tests the turret servo.
 *
 * <p>Both wedge servos drive the same mechanism from opposite sides, so the right servo is
 * reversed: one shared setpoint makes them turn in opposite directions. When INIT is pressed the
 * wedge moves to FLOWER_WEDGE_UP and the turret to SERVO_TEST_START_POSITION.
 *
 * <p>Gamepad1: A toggles the wedge between up and down; d-pad up/down fine-tunes the wedge
 * setpoint by +/-0.05 per press; d-pad right/left changes the turret setpoint by +/-0.05 per press.
 */
@TeleOp(name = "ServoTest", group = "Test")
public class ServoTest extends LinearOpMode {

    private static final double STEP = 0.05;

    private ServoImplEx flowerWedgeLeft;
    private ServoImplEx flowerWedgeRight;
    private ServoImplEx turret;

    private double wedgeSetpoint = Constants.FLOWER_WEDGE_UP;
    private boolean wedgeUp = true;
    private double turretSetpoint = Constants.SERVO_TEST_START_POSITION;

    @Override
    public void runOpMode() {
        flowerWedgeLeft = hardwareMap.get(ServoImplEx.class, "flowerWedgeLeft");
        flowerWedgeRight = hardwareMap.get(ServoImplEx.class, "flowerWedgeRight");
        turret = hardwareMap.get(ServoImplEx.class, "turret");

        // Mounted opposite each other on the same mechanism, so they must turn opposite ways.
        // Directions match RobotHardware and ServoPositionTest.
        flowerWedgeLeft.setDirection(Servo.Direction.FORWARD);
        flowerWedgeRight.setDirection(Servo.Direction.REVERSE);
        RobotHardware.configureTurretServo(turret); // full-range PWM, same as Main 2026

        try {
            applySetpoints();
            telemetry.addLine("ServoTest ready: all servos at init position.");
            addTelemetry();
            telemetry.update();

            waitForStart();

            boolean previousA = false;
            boolean previousUp = false;
            boolean previousDown = false;
            boolean previousRight = false;
            boolean previousLeft = false;

            while (opModeIsActive()) {
                boolean currentA = gamepad1.a;
                boolean currentUp = gamepad1.dpad_up;
                boolean currentDown = gamepad1.dpad_down;
                boolean currentRight = gamepad1.dpad_right;
                boolean currentLeft = gamepad1.dpad_left;

                // A toggles between the preset positions (discarding any d-pad fine-tuning).
                if (currentA && !previousA) {
                    wedgeUp = !wedgeUp;
                    wedgeSetpoint = wedgeUp ? Constants.FLOWER_WEDGE_UP : Constants.FLOWER_WEDGE_DOWN;
                }
                if (currentUp && !previousUp) {
                    wedgeSetpoint += STEP;
                }
                if (currentDown && !previousDown) {
                    wedgeSetpoint -= STEP;
                }
                if (currentRight && !previousRight) {
                    turretSetpoint += STEP;
                }
                if (currentLeft && !previousLeft) {
                    turretSetpoint -= STEP;
                }
                previousA = currentA;
                previousUp = currentUp;
                previousDown = currentDown;
                previousRight = currentRight;
                previousLeft = currentLeft;

                applySetpoints();

                addTelemetry();
                telemetry.addLine();
                telemetry.addLine("A: wedge toggle UP/DOWN preset");
                telemetry.addLine("UP/DOWN: wedge setpoint +/-0.05");
                telemetry.addLine("RIGHT/LEFT: turret setpoint +/-0.05");
                telemetry.update();
            }
        } finally {
            flowerWedgeLeft.setPwmDisable();
            flowerWedgeRight.setPwmDisable();
            turret.setPwmDisable();
        }
    }

    /** Clamps both setpoints to [0, 1], rounds away floating-point drift, and commands servos. */
    private void applySetpoints() {
        wedgeSetpoint = clampAndRound(wedgeSetpoint);
        turretSetpoint = clampAndRound(turretSetpoint);
        flowerWedgeLeft.setPosition(wedgeSetpoint);
        flowerWedgeRight.setPosition(wedgeSetpoint);
        turret.setPosition(turretSetpoint);
    }

    private static double clampAndRound(double value) {
        return Math.round(Math.max(0.0, Math.min(1.0, value)) * 100.0) / 100.0;
    }

    private void addTelemetry() {
        telemetry.addLine("===== FLOWER WEDGES =====");
        telemetry.addData("Wedge preset", "%s (up %.2f / down %.2f)", wedgeUp ? "UP" : "DOWN",
                Constants.FLOWER_WEDGE_UP, Constants.FLOWER_WEDGE_DOWN);
        telemetry.addData("Wedge setpoint", "%.2f", wedgeSetpoint);
        telemetry.addData("Left setpoint / position", "%.2f / %.2f",
                wedgeSetpoint, flowerWedgeLeft.getPosition());
        // Right is reversed: same logical position, mirrored raw servo position (1 - setpoint).
        telemetry.addData("Right setpoint / position (reversed)", "%.2f / %.2f",
                wedgeSetpoint, flowerWedgeRight.getPosition());
        telemetry.addData("Right raw position", "%.2f", 1.0 - flowerWedgeRight.getPosition());
        telemetry.addLine();
        telemetry.addLine("===== TURRET =====");
        telemetry.addData("Turret setpoint / position", "%.2f / %.2f",
                turretSetpoint, turret.getPosition());
    }
}
