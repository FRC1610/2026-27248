package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * Finds the cause of the slow first flower wedge move (suspected: the Swyft servos' ~10 s stall
 * protection reducing power after holding UP against a hard stop or against each other).
 *
 * <p>The wedge moves to UP when START is pressed. The screen shows how long it has been holding
 * its current position. All values below are editable live in Panels (Configurables):
 * <ul>
 *   <li>UP / DOWN: wedge positions (start at the Constants values).
 *   <li>RIGHT_TRIM: added to the right servo's position only, to make the two servos agree.
 *   <li>RELAX_WHEN_UP_SEC: if >= 0, turn both servos' PWM off this long after reaching UP, so
 *       they stop pushing while holding (the wedge must stay up on its own).
 * </ul>
 *
 * <p>Gamepad1: Y toggles UP/DOWN (both servos); d-pad left / right moves only the left / right
 * servo to the other position (the other servo is relaxed), to check each one alone; X relaxes
 * both servos. The servos are relaxed when the OpMode stops.
 */
@Configurable
@Utility(name = "Flower Wedge Test",
        description = "Diagnose the slow first flower wedge move (hold time, trim, relax)")
public class FlowerWedgeTest extends LinearOpMode {
    public static double UP = Constants.FLOWER_WEDGE_UP;
    public static double DOWN = Constants.FLOWER_WEDGE_DOWN;
    /** Added to the right servo's position (after its REVERSE direction) to match the left. */
    public static double RIGHT_TRIM = 0.0;
    /** Seconds after commanding UP to relax both servos; negative = never relax. */
    public static double RELAX_WHEN_UP_SEC = -1;

    @Override
    public void runOpMode() {
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        ServoImplEx left = hardwareMap.get(ServoImplEx.class, "flowerWedgeLeft");
        ServoImplEx right = hardwareMap.get(ServoImplEx.class, "flowerWedgeRight");
        // Same directions as RobotHardware.
        left.setDirection(Servo.Direction.FORWARD);
        right.setDirection(Servo.Direction.REVERSE);

        telemetry.addLine("Flower Wedge Test: the wedge moves to UP when START is pressed.");
        telemetry.addLine("Y: up/down (both). D-pad L/R: left/right servo only. X: relax both.");
        telemetry.addLine("Test: hold UP > 15 s, then Y. Then repeat holding UP only ~3 s.");
        telemetry.update();

        waitForStart();

        try {
            boolean up = true;
            boolean leftOn = true, rightOn = true;
            String lastMove = "both -> UP (START)";
            ElapsedTime held = new ElapsedTime();
            command(left, right, true, true, up);

            boolean prevY = false, prevX = false, prevL = false, prevR = false;
            while (opModeIsActive()) {
                if (gamepad1.y && !prevY) {
                    up = !up;
                    leftOn = rightOn = true;
                    command(left, right, true, true, up);
                    lastMove = "both -> " + (up ? "UP" : "DOWN");
                    held.reset();
                }
                if (gamepad1.dpad_left && !prevL) {
                    up = !up;
                    leftOn = true;
                    rightOn = false;
                    command(left, right, true, false, up);
                    lastMove = "LEFT only -> " + (up ? "UP" : "DOWN");
                    held.reset();
                }
                if (gamepad1.dpad_right && !prevR) {
                    up = !up;
                    leftOn = false;
                    rightOn = true;
                    command(left, right, false, true, up);
                    lastMove = "RIGHT only -> " + (up ? "UP" : "DOWN");
                    held.reset();
                }
                if (gamepad1.x && !prevX) {
                    leftOn = rightOn = false;
                    left.setPwmDisable();
                    right.setPwmDisable();
                    lastMove = "relaxed both";
                }
                prevY = gamepad1.y;
                prevX = gamepad1.x;
                prevL = gamepad1.dpad_left;
                prevR = gamepad1.dpad_right;

                // Optional: stop pushing once UP has been reached.
                if (up && RELAX_WHEN_UP_SEC >= 0 && held.seconds() > RELAX_WHEN_UP_SEC
                        && (leftOn || rightOn)) {
                    leftOn = rightOn = false;
                    left.setPwmDisable();
                    right.setPwmDisable();
                    lastMove += " (auto-relaxed)";
                }

                telemetry.addData("Wedge", up ? "UP" : "DOWN");
                telemetry.addData("Last move", lastMove);
                telemetry.addData("Holding for (s)", "%.1f", held.seconds());
                telemetry.addData("Left servo", leftOn ? String.format("%.3f", left.getPosition()) : "relaxed");
                telemetry.addData("Right servo", rightOn ? String.format("%.3f (trim %+.3f)",
                        right.getPosition(), RIGHT_TRIM) : "relaxed");
                telemetry.addData("UP / DOWN", "%.3f / %.3f", UP, DOWN);
                telemetry.addData("Relax when UP (s)", RELAX_WHEN_UP_SEC < 0 ? "off"
                        : String.format("%.1f", RELAX_WHEN_UP_SEC));
                telemetry.update();
                idle();
            }
        } finally {
            left.setPwmDisable();
            right.setPwmDisable();
        }
    }

    private static void command(ServoImplEx left, ServoImplEx right, boolean useLeft,
                                boolean useRight, boolean up) {
        double p = up ? UP : DOWN;
        if (useLeft) {
            left.setPosition(clip(p)); // position before enable (hub NACKs otherwise)
            left.setPwmEnable();
        } else {
            left.setPwmDisable();
        }
        if (useRight) {
            right.setPosition(clip(p + RIGHT_TRIM));
            right.setPwmEnable();
        } else {
            right.setPwmDisable();
        }
    }

    private static double clip(double v) {
        return Math.max(0, Math.min(1, v));
    }
}
