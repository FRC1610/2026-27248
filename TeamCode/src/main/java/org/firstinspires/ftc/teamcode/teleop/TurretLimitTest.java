package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.TurretTracker;

/**
 * Finds TURRET_MIN_POSITION and TURRET_MAX_POSITION, and tests Limelight turret tracking.
 * Uses only the turret servo, the Limelight and allianceButton (no drive or Pinpoint).
 *
 * <p>When START is pressed the turret moves to TURRET_HOLD_POSITION. Gamepad1:
 * <ul>
 *   <li>D-pad left/right (tracking off): -/+ TURRET_LIMIT_TEST_STEP (0.01, about 4.0 deg of
 *       turret) per press, within 0-1. This deliberately ignores TURRET_MIN/MAX_POSITION so it
 *       can find them.
 *   <li>A: toggles tracking. While on, TurretTracker centers the turret on Limelight tx
 *       (tx &gt; 0 = turret too far left, so it turns right; tx &lt; 0 = too far right, so it turns
 *       left), always within TURRET_MIN/MAX_POSITION. Uses the alliance's default pipeline
 *       (allianceButton). There is no Pinpoint assist here, so if the target is lost the turret
 *       holds where it is. If the Limelight stops sending data (LIMELIGHT_LOST_TIMEOUT_MS), the
 *       turret returns to TURRET_HOLD_POSITION until data comes back.
 * </ul>
 * The servo is relaxed (PWM off) and the Limelight stopped when the OpMode stops.
 */
@TeleOp(name = "Turret Limit Test", group = "Test")
public class TurretLimitTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        ServoImplEx turret = hardwareMap.get(ServoImplEx.class, "turret");
        RobotHardware.configureTurretServo(turret);

        // Same alliance -> pipeline rule as RobotHardware: allianceButton low = red.
        DigitalChannel allianceButton = hardwareMap.get(DigitalChannel.class, "allianceButton");
        allianceButton.setMode(DigitalChannel.Mode.INPUT);
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(Constants.LIMELIGHT_POLL_RATE_HZ);

        double position = Constants.TURRET_HOLD_POSITION;

        telemetry.addLine("Turret Limit Test: turret moves to "
                + Constants.TURRET_HOLD_POSITION + " when START is pressed.");
        telemetry.addLine("Make sure it can turn freely and cables have slack.");
        telemetry.addLine("GP1 d-pad left/right: -/+ " + Constants.TURRET_LIMIT_TEST_STEP
                + " (tracking off); A: tracking on/off.");
        telemetry.update();

        try {
            int pipeline = selectPipeline(allianceButton, limelight, -1);
            limelight.start();

            waitForStart();
            if (isStopRequested()) return;
            turret.setPwmEnable();
            turret.setPosition(position);

            boolean tracking = false;
            TurretTracker tracker = null;
            boolean leftPreviouslyPressed = false;
            boolean rightPreviouslyPressed = false;
            boolean aPreviouslyPressed = false;
            ElapsedTime loopTimer = new ElapsedTime();
            double lastLoopSec = loopTimer.seconds();

            while (opModeIsActive()) {
                double nowSec = loopTimer.seconds();
                double dtSec = nowSec - lastLoopSec;
                lastLoopSec = nowSec;

                pipeline = selectPipeline(allianceButton, limelight, pipeline);
                LLResult result = limelight.getLatestResult();
                boolean targetVisible = result != null && result.isValid()
                        && result.getPipelineIndex() == pipeline
                        && result.getStaleness() <= Constants.TURRET_MAX_VISION_AGE_MS;
                double tx = targetVisible ? result.getTx() : Double.NaN;

                // A toggles tracking. A fresh tracker starts from the current position (pulled
                // inside the configured limits if the d-pad left it outside them).
                if (gamepad1.a && !aPreviouslyPressed) {
                    tracking = !tracking;
                    tracker = tracking ? new TurretTracker(position) : null;
                }
                aPreviouslyPressed = gamepad1.a;

                double next = position;
                boolean limelightReceiving = TurretTracker.isLimelightReceiving(result);
                if (tracking) {
                    // No Pinpoint here (heading NaN); no manual input. The tracker only returns
                    // angles inside TURRET_MIN/MAX_POSITION; clamp again to be sure. With no
                    // Limelight data, go back to the INIT (hold) position.
                    next = limelightReceiving
                            ? tracker.update(Double.NaN, targetVisible, tx,
                                    result.getControlHubTimeStampNanos(), 0, dtSec)
                            : tracker.limelightLost();
                    next = Math.max(Constants.TURRET_MIN_POSITION,
                            Math.min(Constants.TURRET_MAX_POSITION, next));
                } else {
                    if (gamepad1.dpad_left && !leftPreviouslyPressed) {
                        next -= Constants.TURRET_LIMIT_TEST_STEP;
                    }
                    if (gamepad1.dpad_right && !rightPreviouslyPressed) {
                        next += Constants.TURRET_LIMIT_TEST_STEP;
                    }
                    // Round to avoid drift like 0.6499999 after repeated steps; stay within 0-1.
                    next = Math.max(0.0, Math.min(1.0, Math.round(next * 1000.0) / 1000.0));
                }
                leftPreviouslyPressed = gamepad1.dpad_left;
                rightPreviouslyPressed = gamepad1.dpad_right;

                if (next != position) {
                    position = next;
                    turret.setPosition(position);
                }

                telemetry.addData("Tracking (A)", tracking ? "ON - " + tracker.getState() : "OFF");
                telemetry.addData("Turret position", "%.3f", position);
                telemetry.addData("Turret angle (deg, + = left of forward)", "%.1f",
                        TurretTracker.positionToAngle(position));
                telemetry.addData("Limits", "MIN %.3f / MAX %.3f",
                        Constants.TURRET_MIN_POSITION, Constants.TURRET_MAX_POSITION);
                telemetry.addData("Limelight", "%s, %s, pipeline %d (actual %s)",
                        limelightReceiving ? "connected" : "NO DATA",
                        allianceButton.getState() ? "BLUE" : "RED", pipeline,
                        result == null ? "-" : String.valueOf(result.getPipelineIndex()));
                if (targetVisible) {
                    telemetry.addData("tx (deg)", "%.2f  (%s)", tx,
                            tx > 0 ? "turret too far left" : tx < 0 ? "turret too far right"
                                    : "centered");
                } else {
                    telemetry.addData("tx (deg)", "no fresh target");
                }
                if (!tracking) {
                    telemetry.addLine("D-pad left: -" + Constants.TURRET_LIMIT_TEST_STEP
                            + "   D-pad right: +" + Constants.TURRET_LIMIT_TEST_STEP);
                }
                telemetry.update();
                idle();
            }
        } finally {
            turret.setPwmDisable();
            limelight.stop();
        }
    }

    /** Switches to the alliance's default pipeline if it changed; returns the selected one. */
    private static int selectPipeline(DigitalChannel allianceButton, Limelight3A limelight,
                                      int current) {
        boolean red = !allianceButton.getState();
        int pipeline = red ? Constants.RED_VISION_PIPELINE : Constants.BLUE_VISION_PIPELINE;
        if (pipeline != current) limelight.pipelineSwitch(pipeline);
        return pipeline;
    }
}
