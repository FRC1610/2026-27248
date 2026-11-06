package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.Locale;

/**
 * Finds safe servo positions for the flowerWedge and turret. No servo moves until A is pressed.
 *
 * Gamepad1: d-pad left/right selects target, A enables it at 0.5, X disables (relaxes) it,
 * d-pad up/down nudges +/-0.01, right/left bumper nudges +/-0.001.
 */
@TeleOp(name = "Servo Position Test", group = "Test")
public class ServoPositionTest extends LinearOpMode {
    private static final double START_POSITION = Constants.SERVO_TEST_START_POSITION;
    private static final double COARSE_STEP = 0.01;
    private static final double FINE_STEP = 0.001;
    private static final double TURRET_DEG_PER_SERVO_UNIT = Constants.TURRET_DEG_PER_SERVO_UNIT;

    private static final String[] TARGETS = {
            "flowerWedge pair", "flowerWedgeLeft", "flowerWedgeRight", "turret"};
    private static final int PAIR = 0;
    private static final int LEFT = 1;
    private static final int RIGHT = 2;
    private static final int TURRET = 3;

    private ServoImplEx wedgeLeft;
    private ServoImplEx wedgeRight;
    private ServoImplEx turret;

    /** Last commanded position per servo (index LEFT, RIGHT, TURRET); NaN = not enabled. */
    private final double[] positions = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};

    @Override
    public void runOpMode() {
        wedgeLeft = hardwareMap.get(ServoImplEx.class, "flowerWedgeLeft");
        wedgeRight = hardwareMap.get(ServoImplEx.class, "flowerWedgeRight");
        turret = hardwareMap.get(ServoImplEx.class, "turret");

        // Mounted opposite each other: reversing the right servo should make the same
        // position move both sides of the wedge the same way. Verify with the single-servo targets.
        wedgeLeft.setDirection(Servo.Direction.FORWARD);
        wedgeRight.setDirection(Servo.Direction.REVERSE);
        turret.setDirection(Servo.Direction.FORWARD);

        telemetry.addLine("No servo moves until you press A.");
        telemetry.addLine("Before enabling, make sure the mechanism can safely reach 0.5.");
        telemetry.update();

        waitForStart();

        try {
            int selected = PAIR;
            boolean prevLeft = false, prevRight = false, prevUp = false, prevDown = false;
            boolean prevA = false, prevX = false, prevRb = false, prevLb = false;

            while (opModeIsActive()) {
                if (gamepad1.dpad_right && !prevRight) selected = (selected + 1) % TARGETS.length;
                if (gamepad1.dpad_left && !prevLeft) {
                    selected = (selected + TARGETS.length - 1) % TARGETS.length;
                }

                if (gamepad1.a && !prevA) enable(selected);
                if (gamepad1.x && !prevX) disable(selected);

                double delta = 0;
                if (gamepad1.dpad_up && !prevUp) delta += COARSE_STEP;
                if (gamepad1.dpad_down && !prevDown) delta -= COARSE_STEP;
                if (gamepad1.right_bumper && !prevRb) delta += FINE_STEP;
                if (gamepad1.left_bumper && !prevLb) delta -= FINE_STEP;
                if (delta != 0) nudge(selected, delta);

                prevLeft = gamepad1.dpad_left;
                prevRight = gamepad1.dpad_right;
                prevUp = gamepad1.dpad_up;
                prevDown = gamepad1.dpad_down;
                prevA = gamepad1.a;
                prevX = gamepad1.x;
                prevRb = gamepad1.right_bumper;
                prevLb = gamepad1.left_bumper;

                telemetry.addData("Selected", TARGETS[selected]);
                telemetry.addLine("A enable | X relax | dpad U/D +/-0.01 | RB/LB +/-0.001");
                telemetry.addData("flowerWedgeLeft", format(positions[LEFT]));
                telemetry.addData("flowerWedgeRight (reversed)", format(positions[RIGHT]));
                telemetry.addData("turret", format(positions[TURRET]));
                if (!Double.isNaN(positions[TURRET])) {
                    telemetry.addData("turret est. deg from 0.5", "%.1f",
                            (positions[TURRET] - 0.5) * TURRET_DEG_PER_SERVO_UNIT);
                }
                telemetry.update();
                idle();
            }
        } finally {
            wedgeLeft.setPwmDisable();
            wedgeRight.setPwmDisable();
            turret.setPwmDisable();
        }
    }

    private void enable(int target) {
        if (target == PAIR) {
            // Start both sides from the same position so they don't fight.
            double start = !Double.isNaN(positions[LEFT]) ? positions[LEFT] : START_POSITION;
            set(LEFT, start);
            set(RIGHT, start);
        } else if (Double.isNaN(positions[target])) {
            set(target, START_POSITION);
        }
    }

    private void disable(int target) {
        if (target == PAIR || target == LEFT) relax(LEFT);
        if (target == PAIR || target == RIGHT) relax(RIGHT);
        if (target == TURRET) relax(TURRET);
    }

    private void nudge(int target, double delta) {
        if (target == PAIR) {
            if (Double.isNaN(positions[LEFT]) || Double.isNaN(positions[RIGHT])) return;
            double next = positions[LEFT] + delta;
            set(LEFT, next);
            set(RIGHT, next);
        } else if (!Double.isNaN(positions[target])) {
            set(target, positions[target] + delta);
        }
    }

    private void set(int index, double position) {
        double p = Math.max(0.0, Math.min(1.0, position));
        ServoImplEx servo = servoFor(index);
        servo.setPwmEnable();
        servo.setPosition(p);
        positions[index] = p;
    }

    private void relax(int index) {
        servoFor(index).setPwmDisable();
        positions[index] = Double.NaN;
    }

    private ServoImplEx servoFor(int index) {
        if (index == LEFT) return wedgeLeft;
        if (index == RIGHT) return wedgeRight;
        return turret;
    }

    private static String format(double position) {
        return Double.isNaN(position) ? "off" : String.format(Locale.US, "%.3f", position);
    }
}
