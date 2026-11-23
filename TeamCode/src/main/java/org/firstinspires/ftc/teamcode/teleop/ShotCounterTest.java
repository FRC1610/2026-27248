package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Constants;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Tests the feeding and counting logic planned for the autonomous "shoot 4" command.
 *
 * <p>Counting:
 * <ul>
 *   <li><b>Beam passes:</b> indexBeam1 (at the indexer entrance) going blocked -> clear, i.e. a
 *       ball has moved past the sensor into the indexer.
 *   <li><b>Shots:</b> the flywheel dropping more than SHOT_DROP_RPM below its setpoint (each
 *       launched ball takes ~260-600 RPM). Detection only starts once the flywheel has first
 *       reached speed (spinning up no longer counts as a shot) and re-arms once the speed is back
 *       within SHOT_REARM_RPM.
 * </ul>
 *
 * <p>Feeding (hold RB; the intake runs whenever RB is held):
 * <ul>
 *   <li><b>WAIT FOR RECOVERY</b> (default, the planned autonomous behavior): the indexer starts
 *       once the flywheel is no more than READY_TOLERANCE_RPM <i>below</i> its setpoint (running
 *       fast does not block it) and then keeps running steadily until a shot is detected; it
 *       stops until the flywheel recovers again. (The first version also stopped whenever the
 *       speed read high, which chopped the indexer on and off.)
 *   <li><b>CONTINUOUS</b>: the indexer runs whenever RB is held.
 * </ul>
 *
 * <p><b>Stuck-ball recovery</b> (B toggles, default on): if a ball has passed the beam and no shot
 * follows within STUCK_TIMEOUT_SEC of the indexer running forward, the indexer reverses at
 * UNJAM_POWER for UNJAM_SEC and then feeds again (the intake keeps pushing the next ball).
 *
 * <p>Gamepad1: A flywheel on/off; hold RB feed; hold LB reverse everything; Y feed mode; B stuck
 * recovery on/off; X resets counters; d-pad right/left indexer speed +/-5%.
 *
 * <p>Uses the tuned flywheel PIDF from Constants and bulk reads for a faster loop. Logs every
 * loop to /sdcard/FIRST/data/shots_yyyyMMdd_HHmmss.csv. Thresholds are editable live in Panels
 * (Configurables).
 */
@Configurable
@Utility(name = "Shot Counter Test",
        description = "Counts balls (indexer beam) and shots (flywheel dips); CSV log")
public class ShotCounterTest extends LinearOpMode {
    /** A shot is detected when the flywheel falls this far below its setpoint (RPM). */
    public static double SHOT_DROP_RPM = 150;
    /** Shot detection (re-)arms once the flywheel is back within this of its setpoint (RPM). */
    public static double SHOT_REARM_RPM = 100;
    /** WAIT FOR RECOVERY starts the indexer once the flywheel is at most this far below its
     *  setpoint (RPM). Running above the setpoint never blocks feeding. */
    public static double READY_TOLERANCE_RPM = Constants.LIGHTS_FLYWHEEL_READY_TOLERANCE_RPM;
    /** Intake power while feeding (RB) or reversing (LB). */
    public static double FEED_INTAKE_POWER = 1.0;
    /** Stuck ball: no shot this long (s, indexer running forward) after a ball passed the beam. */
    public static double STUCK_TIMEOUT_SEC = 0.4;
    /**
     * Unjam: indexer power (negative = reverse) and duration (s). About 0.08 s of the duration is
     * spent braking the indexer from full forward speed, so 0.35 s gives ~0.25 s of real reverse
     * (0.15 s at -0.6 only reversed for ~0.07 s in testing).
     */
    public static double UNJAM_POWER = -1.0;
    public static double UNJAM_SEC = 0.35;

    private static final double ENCODER_FAULT_RPM = 50;
    private static final double ENCODER_FAULT_SEC = 2.0;

    @Override
    public void runOpMode() {
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        // Bulk reads: all encoder/digital reads on a hub come from one transaction per loop.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        DcMotorEx indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        DcMotorEx intakeLeft = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        DcMotorEx intakeRight = hardwareMap.get(DcMotorEx.class, "intakeRight");
        DigitalChannel beam = hardwareMap.get(DigitalChannel.class, "indexBeam1");
        beam.setMode(DigitalChannel.Mode.INPUT);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        // Same directions and modes as RobotHardware.
        configure(flywheel, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT);
        configure(indexer, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        configure(intakeLeft, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        configure(intakeRight, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        if (Constants.FLYWHEEL_USE_TUNED_PIDF) {
            flywheel.setVelocityPIDFCoefficients(Constants.FLYWHEEL_PIDF_P,
                    Constants.FLYWHEEL_PIDF_I, Constants.FLYWHEEL_PIDF_D, Constants.FLYWHEEL_PIDF_F);
        }

        telemetry.addLine("Shot Counter Test: flywheel, indexer and intake only.");
        telemetry.addLine("GP1: A flywheel, hold RB feed, hold LB reverse, Y feed mode,");
        telemetry.addLine("B stuck recovery, X reset, d-pad R/L indexer speed. Load 4 balls.");
        telemetry.update();

        BufferedWriter log = null;
        String logPath;
        try {
            waitForStart();
            if (isStopRequested()) return;

            try {
                File dir = AppUtil.ROBOT_DATA_DIR;
                //noinspection ResultOfMethodCallIgnored
                dir.mkdirs();
                File file = new File(dir, "shots_"
                        + new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date())
                        + ".csv");
                log = new BufferedWriter(new FileWriter(file));
                log.write("t_s,loop_ms,beam_blocked,beam_passes,flywheel_on,setpoint_rpm,"
                        + "actual_rpm,shots,feed_held,wait_mode,feed_ready,ball_pending,"
                        + "recovery_on,unjamming,unjams,indexer_cmd,indexer_rpm,intake_power,"
                        + "battery_v\n");
                logPath = file.getAbsolutePath();
            } catch (IOException e) {
                logPath = "LOG FAILED: " + e.getMessage();
            }

            boolean flywheelOn = false;
            boolean waitForRecovery = true;
            boolean stuckRecovery = true;
            double setpointRpm = Constants.FLYWHEEL_DEFAULT_RPM;
            double indexerPercent = Constants.INDEXER_DEFAULT_SPEED_PERCENT;
            boolean prevA = false, prevY = false, prevX = false, prevB = false;
            boolean prevRight = false, prevLeft = false;

            // Counting state.
            int beamPasses = 0;
            int shots = 0;
            boolean shotArmed = false;      // only armed after the flywheel first reaches speed
            boolean beamWasBlocked = beam.getState() == Constants.INDEX_BEAM_BALL_STATE;
            double beamChangeT = 0;
            double lastBlockedSec = Double.NaN, lastClearGapSec = Double.NaN;
            double minClearGapSec = Double.NaN;

            // Feeding / stuck-ball state.
            boolean feedReady = false;      // WAIT mode latch: set on recovery, cleared by a shot
            boolean ballPending = false;    // a ball passed the beam and has not been shot yet
            double pendingSinceT = 0;
            double stuckTimer = 0;          // indexer-forward time since the pending ball passed
            double unjamUntilT = -1;
            int unjams = 0;
            double lastLatencySec = Double.NaN, maxLatencySec = Double.NaN;

            double noSpeedSec = 0;
            double volts = battery.getVoltage();
            String fault = "";
            double loopMsMax = 0, loopMsSum = 0;
            long loops = 0;

            ElapsedTime clock = new ElapsedTime();
            double lastT = 0, lastFlushT = 0;

            while (opModeIsActive()) {
                double now = clock.seconds();
                double dt = now - lastT;
                lastT = now;
                double loopMs = dt * 1000.0;
                if (loops > 0) {
                    loopMsSum += loopMs;
                    loopMsMax = Math.max(loopMsMax, loopMs);
                }
                loops++;

                // ---- Buttons ----
                if (gamepad1.a && !prevA) {
                    flywheelOn = !flywheelOn;
                    shotArmed = false;          // re-arm only after reaching speed again
                    feedReady = false;
                    fault = "";
                    noSpeedSec = 0;
                }
                if (gamepad1.y && !prevY) waitForRecovery = !waitForRecovery;
                if (gamepad1.b && !prevB) stuckRecovery = !stuckRecovery;
                if (gamepad1.x && !prevX) {
                    beamPasses = 0;
                    shots = 0;
                    ballPending = false;
                    stuckTimer = 0;
                    unjams = 0;
                    minClearGapSec = Double.NaN;
                    lastLatencySec = Double.NaN;
                    maxLatencySec = Double.NaN;
                    loopMsMax = 0;
                }
                if (gamepad1.dpad_right && !prevRight) {
                    indexerPercent = Math.min(100, indexerPercent + Constants.INDEXER_SPEED_STEP_PERCENT);
                }
                if (gamepad1.dpad_left && !prevLeft) {
                    indexerPercent = Math.max(0, indexerPercent - Constants.INDEXER_SPEED_STEP_PERCENT);
                }
                prevA = gamepad1.a;
                prevY = gamepad1.y;
                prevB = gamepad1.b;
                prevX = gamepad1.x;
                prevRight = gamepad1.dpad_right;
                prevLeft = gamepad1.dpad_left;
                boolean feedHeld = gamepad1.right_bumper;
                boolean reverseHeld = gamepad1.left_bumper && !feedHeld;

                // ---- Sensors (bulk-cached) ----
                double actualRpm = flywheel.getVelocity() * 60.0 / Constants.FLYWHEEL_TICKS_PER_REV;
                double indexerRpm = indexer.getVelocity() * 60.0 / Constants.INDEXER_TICKS_PER_REV;
                boolean beamBlocked = beam.getState() == Constants.INDEX_BEAM_BALL_STATE;
                if (loops % 25 == 0) volts = battery.getVoltage(); // not part of the bulk read

                // ---- Beam: count blocked -> clear (a ball moved past the sensor) ----
                if (beamBlocked != beamWasBlocked) {
                    double held = now - beamChangeT;
                    if (beamWasBlocked) {
                        beamPasses++;
                        lastBlockedSec = held;
                        if (!ballPending) {
                            ballPending = true;
                            pendingSinceT = now;
                            stuckTimer = 0;
                        }
                    } else if (beamPasses > 0) {
                        lastClearGapSec = held;
                        minClearGapSec = Double.isNaN(minClearGapSec)
                                ? held : Math.min(minClearGapSec, held);
                    }
                    beamChangeT = now;
                    beamWasBlocked = beamBlocked;
                }

                // ---- Flywheel: on/off, encoder fault check ----
                double commandedRpm = flywheelOn ? setpointRpm : 0;
                if (flywheelOn && Math.abs(actualRpm) < ENCODER_FAULT_RPM) {
                    noSpeedSec += dt;
                    if (noSpeedSec > ENCODER_FAULT_SEC) {
                        flywheelOn = false;
                        commandedRpm = 0;
                        fault = "FLYWHEEL STOPPED: encoder read ~0 RPM for 2 s while on. Check the cable.";
                    }
                } else {
                    noSpeedSec = 0;
                }
                if (commandedRpm > 0) {
                    flywheel.setVelocity(commandedRpm * Constants.FLYWHEEL_TICKS_PER_REV / 60.0);
                } else {
                    flywheel.setPower(0);
                    shotArmed = false;
                    feedReady = false;
                }

                // ---- Shot detection (only after the flywheel has reached speed) ----
                double underspeed = commandedRpm - actualRpm;   // > 0 = too slow
                if (commandedRpm > 0) {
                    if (shotArmed && underspeed > SHOT_DROP_RPM) {
                        shots++;
                        shotArmed = false;
                        feedReady = false;
                        if (ballPending) {
                            lastLatencySec = now - pendingSinceT;
                            maxLatencySec = Double.isNaN(maxLatencySec)
                                    ? lastLatencySec : Math.max(maxLatencySec, lastLatencySec);
                        }
                        ballPending = false;
                        stuckTimer = 0;
                    } else if (!shotArmed && underspeed < SHOT_REARM_RPM) {
                        shotArmed = true;
                    }
                    // WAIT mode latch: ready once no more than READY_TOLERANCE_RPM too slow.
                    if (!feedReady && shotArmed && underspeed <= READY_TOLERANCE_RPM) {
                        feedReady = true;
                    }
                }

                // ---- Feed ----
                boolean indexerAllowed = flywheelOn && actualRpm >= Constants.INDEXER_MIN_FLYWHEEL_RPM;
                boolean wantForward = feedHeld && indexerAllowed && (!waitForRecovery || feedReady);

                // Stuck ball: indexer has been feeding forward with a ball inside but no shot.
                boolean unjamming = now < unjamUntilT;
                if (stuckRecovery && wantForward && ballPending && !unjamming) {
                    stuckTimer += dt;
                    if (stuckTimer > STUCK_TIMEOUT_SEC) {
                        unjamUntilT = now + UNJAM_SEC;
                        unjamming = true;
                        unjams++;
                        stuckTimer = 0;
                    }
                }

                double indexerCmd; // logged: RPM when forward, power (negative) when reversing
                if (reverseHeld) {
                    indexer.setPower(-1);
                    indexerCmd = -1;
                } else if (feedHeld && unjamming) {
                    indexer.setPower(UNJAM_POWER);
                    indexerCmd = UNJAM_POWER;
                } else if (wantForward) {
                    indexerCmd = Constants.INDEXER_MAX_RPM * indexerPercent / 100.0;
                    indexer.setVelocity(indexerCmd * Constants.INDEXER_TICKS_PER_REV / 60.0);
                } else {
                    indexer.setPower(0);
                    indexerCmd = 0;
                }
                double intakePower = feedHeld ? FEED_INTAKE_POWER
                        : reverseHeld ? -FEED_INTAKE_POWER : 0;
                intakeLeft.setPower(intakePower);
                intakeRight.setPower(intakePower);

                // ---- Log ----
                if (log != null) {
                    try {
                        log.write(String.format(Locale.US,
                                "%.4f,%.2f,%d,%d,%d,%.0f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.1f,%.2f,%.2f\n",
                                now, loopMs, beamBlocked ? 1 : 0, beamPasses, flywheelOn ? 1 : 0,
                                commandedRpm, actualRpm, shots, feedHeld ? 1 : 0,
                                waitForRecovery ? 1 : 0, feedReady ? 1 : 0, ballPending ? 1 : 0,
                                stuckRecovery ? 1 : 0, (feedHeld && unjamming) ? 1 : 0, unjams,
                                indexerCmd, indexerRpm, intakePower, volts));
                        if (now - lastFlushT > 1.0) {
                            log.flush();
                            lastFlushT = now;
                        }
                    } catch (IOException e) {
                        logPath = "LOG WRITE FAILED: " + e.getMessage();
                        log = null;
                    }
                }

                // ---- Telemetry (plain numbers graph in Panels) ----
                if (!fault.isEmpty()) telemetry.addLine(fault);
                telemetry.addData("Beam passes (balls)", beamPasses);
                telemetry.addData("Shots (flywheel dips)", shots);
                telemetry.addData("Unjams", unjams);
                telemetry.addData("Ball in indexer, not shot", ballPending ? 1.0 : 0.0);
                telemetry.addData("Beam -> shot (ms) last / max", "%d / %d",
                        Math.round(lastLatencySec * 1000), Math.round(maxLatencySec * 1000));
                telemetry.addData("Beam blocked", beamBlocked ? 1.0 : 0.0);
                telemetry.addData("Last ball in beam (ms)", Math.round(lastBlockedSec * 1000));
                telemetry.addData("Last gap between balls (ms)", Math.round(lastClearGapSec * 1000));
                telemetry.addData("Shortest gap (ms)", Math.round(minClearGapSec * 1000));
                telemetry.addData("Loop (ms) avg / max", "%.1f / %.1f",
                        loops > 1 ? loopMsSum / (loops - 1) : 0, loopMsMax);
                telemetry.addData("Flywheel setpoint (RPM)", commandedRpm);
                telemetry.addData("Flywheel actual (RPM)", (double) Math.round(actualRpm));
                telemetry.addData("Feed mode (Y)", waitForRecovery ? "WAIT FOR RECOVERY" : "CONTINUOUS");
                telemetry.addData("Stuck recovery (B)", stuckRecovery ? "ON" : "OFF");
                telemetry.addData("Feeding", feedHeld
                        ? (unjamming ? "UNJAMMING"
                        : wantForward ? "INDEXER RUNNING"
                        : !indexerAllowed ? "WAITING (flywheel < "
                        + (int) Constants.INDEXER_MIN_FLYWHEEL_RPM + ")" : "WAITING FOR RECOVERY")
                        : reverseHeld ? "REVERSE" : "off");
                telemetry.addData("Indexer speed (%)", indexerPercent);
                telemetry.addData("Indexer actual (RPM)", (double) Math.round(indexerRpm));
                telemetry.addData("Battery (V)", volts);
                telemetry.addData("Log", logPath);
                telemetry.update();
            }
        } finally {
            flywheel.setPower(0);
            indexer.setPower(0);
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
            if (log != null) {
                try {
                    log.close();
                } catch (IOException ignored) {
                    // Nothing else to do at stop.
                }
            }
        }
    }

    private static void configure(DcMotorEx motor, DcMotorSimple.Direction direction,
                                  DcMotor.ZeroPowerBehavior zeroPower) {
        motor.setPower(0);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(zeroPower);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
