package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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
 * Flywheel/uptake tuning: live Panels graphs, live-editable flywheel PIDF gains (Panels
 * Configurables), and a CSV log on the Control Hub for offline analysis.
 *
 * <p>Uses only the flywheel, uptake, intake motors, indexBeam1 and the hub voltage sensor
 * (no drive, turret or vision).
 *
 * <p>Gamepad1: A flywheel on/off, d-pad up/down setpoint +/-FLYWHEEL_RPM_STEP, B uptake on/off
 * (forward only while the flywheel measures >= UPTAKE_MIN_FLYWHEEL_RPM, as in Main 2026),
 * RT/LT intake forward/reverse.
 *
 * <p>Gains: kP/kI/kD/kF start at the hub's current values and can be edited in Panels while the
 * OpMode runs; changes are applied to the flywheel immediately and logged. Copy the final values
 * into Constants.FLYWHEEL_PIDF_* and set FLYWHEEL_USE_TUNED_PIDF = true for Main 2026.
 *
 * <p>Log: /sdcard/FIRST/data/launcher_yyyyMMdd_HHmmss.csv (one row per loop).
 *
 * <p>Safety: if the flywheel is on but its encoder reads under ENCODER_FAULT_RPM for
 * ENCODER_FAULT_SEC, the flywheel is turned off (with no encoder feedback the velocity loop would
 * run it at full power).
 */
@Configurable
@Utility(name = "Launcher Tuning",
        description = "Flywheel/uptake PIDF tuning with Panels graphs and CSV log")
public class LauncherTuning extends LinearOpMode {
    /** Flywheel velocity PIDF gains (hub units). Negative = not set yet (filled from the hub). */
    public static double kP = -1;
    public static double kI = -1;
    public static double kD = -1;
    public static double kF = -1;

    private static final double ENCODER_FAULT_RPM = 50;
    private static final double ENCODER_FAULT_SEC = 2.0;

    @Override
    public void runOpMode() {
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        DcMotorEx uptake = hardwareMap.get(DcMotorEx.class, "uptake");
        DcMotorEx intakeLeft = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        DcMotorEx intakeRight = hardwareMap.get(DcMotorEx.class, "intakeRight");
        DigitalChannel indexBeam1 = hardwareMap.get(DigitalChannel.class, "indexBeam1");
        indexBeam1.setMode(DigitalChannel.Mode.INPUT);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        // Same directions and modes as RobotHardware.
        configure(flywheel, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT);
        configure(uptake, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        configure(intakeLeft, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        configure(intakeRight, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);

        // Start from the tuned gains in Constants (or the hub's current gains if tuning is off),
        // unless they were already set in Panels.
        PIDFCoefficients hub = flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        if (kP < 0 || kI < 0 || kD < 0 || kF < 0) {
            if (Constants.FLYWHEEL_USE_TUNED_PIDF) {
                kP = Constants.FLYWHEEL_PIDF_P;
                kI = Constants.FLYWHEEL_PIDF_I;
                kD = Constants.FLYWHEEL_PIDF_D;
                kF = Constants.FLYWHEEL_PIDF_F;
            } else {
                kP = hub.p;
                kI = hub.i;
                kD = hub.d;
                kF = hub.f;
            }
        }
        double[] applied = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};

        telemetry.addLine("Launcher Tuning: flywheel, uptake and intake only.");
        telemetry.addData("Hub PIDF at INIT", "P %.3f I %.3f D %.3f F %.3f",
                hub.p, hub.i, hub.d, hub.f);
        telemetry.addLine("GP1: A flywheel, dpad U/D setpoint, B uptake, RT/LT intake.");
        telemetry.addLine("Edit kP/kI/kD/kF in Panels > Configurables while running.");
        telemetry.update();

        BufferedWriter log = null;
        String logPath = "(not open)";
        try {
            waitForStart();
            if (isStopRequested()) return;

            try {
                File dir = AppUtil.ROBOT_DATA_DIR;
                //noinspection ResultOfMethodCallIgnored
                dir.mkdirs();
                File file = new File(dir, "launcher_"
                        + new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date())
                        + ".csv");
                log = new BufferedWriter(new FileWriter(file));
                log.write("t_s,flywheel_on,setpoint_rpm,actual_rpm,error_rpm,flywheel_amps,"
                        + "battery_v,uptake_on,uptake_cmd_rpm,uptake_rpm,ball,intake_power,"
                        + "kP,kI,kD,kF\n");
                logPath = file.getAbsolutePath();
            } catch (IOException e) {
                logPath = "LOG FAILED: " + e.getMessage();
            }

            boolean flywheelOn = false;
            boolean uptakeOn = false;
            double setpointRpm = Constants.FLYWHEEL_DEFAULT_RPM;
            boolean prevA = false, prevB = false, prevUp = false, prevDown = false;
            double noSpeedSec = 0;
            String fault = "";
            ElapsedTime clock = new ElapsedTime();
            double lastSec = 0, lastFlushSec = 0;

            while (opModeIsActive()) {
                double now = clock.seconds();
                double dt = now - lastSec;
                lastSec = now;

                // Apply gains edited in Panels.
                if (kP != applied[0] || kI != applied[1] || kD != applied[2] || kF != applied[3]) {
                    if (kP >= 0 && kI >= 0 && kD >= 0 && kF >= 0) {
                        flywheel.setVelocityPIDFCoefficients(kP, kI, kD, kF);
                        applied = new double[]{kP, kI, kD, kF};
                    }
                }

                if (gamepad1.a && !prevA) {
                    flywheelOn = !flywheelOn;
                    fault = "";
                    noSpeedSec = 0;
                }
                if (gamepad1.b && !prevB) uptakeOn = !uptakeOn;
                if (gamepad1.dpad_up && !prevUp) setpointRpm += Constants.FLYWHEEL_RPM_STEP;
                if (gamepad1.dpad_down && !prevDown) setpointRpm -= Constants.FLYWHEEL_RPM_STEP;
                prevA = gamepad1.a;
                prevB = gamepad1.b;
                prevUp = gamepad1.dpad_up;
                prevDown = gamepad1.dpad_down;
                setpointRpm = Math.max(0, Math.min(Constants.FLYWHEEL_MAX_RPM, setpointRpm));

                double actualRpm = flywheel.getVelocity() * 60.0 / Constants.FLYWHEEL_TICKS_PER_REV;

                // Encoder fault check: commanded on, but no measured speed.
                if (flywheelOn && setpointRpm > 0 && Math.abs(actualRpm) < ENCODER_FAULT_RPM) {
                    noSpeedSec += dt;
                    if (noSpeedSec > ENCODER_FAULT_SEC) {
                        flywheelOn = false;
                        fault = String.format(Locale.US,
                                "FLYWHEEL STOPPED: encoder read < %.0f RPM for %.1f s while on. "
                                        + "Check the flywheel encoder cable (Control Hub motor port 3).",
                                ENCODER_FAULT_RPM, ENCODER_FAULT_SEC);
                    }
                } else {
                    noSpeedSec = 0;
                }

                double commandedRpm = flywheelOn ? setpointRpm : 0;
                if (commandedRpm > 0) {
                    flywheel.setVelocity(commandedRpm * Constants.FLYWHEEL_TICKS_PER_REV / 60.0);
                } else {
                    flywheel.setPower(0);
                }

                // Uptake: same forward interlock as Main 2026.
                boolean uptakeAllowed = flywheelOn
                        && actualRpm >= Constants.UPTAKE_MIN_FLYWHEEL_RPM;
                double uptakeCmdRpm = uptakeOn && uptakeAllowed
                        ? Constants.UPTAKE_MAX_RPM * Constants.UPTAKE_DEFAULT_SPEED_PERCENT / 100.0
                        : 0;
                if (uptakeCmdRpm > 0) {
                    uptake.setVelocity(uptakeCmdRpm * Constants.UPTAKE_TICKS_PER_REV / 60.0);
                } else {
                    uptake.setPower(0);
                }

                double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
                if (Math.abs(intakePower) < Constants.TRIGGER_DEADBAND) intakePower = 0;
                intakeLeft.setPower(intakePower);
                intakeRight.setPower(intakePower);

                double uptakeRpm = uptake.getVelocity() * 60.0 / Constants.UPTAKE_TICKS_PER_REV;
                double amps = flywheel.getCurrent(CurrentUnit.AMPS);
                double volts = battery.getVoltage();
                boolean ball = indexBeam1.getState() == Constants.INDEX_BEAM_BALL_STATE;

                if (log != null) {
                    try {
                        log.write(String.format(Locale.US,
                                "%.3f,%d,%.0f,%.1f,%.1f,%.2f,%.2f,%d,%.0f,%.1f,%d,%.2f,%.4f,%.4f,%.4f,%.4f\n",
                                now, flywheelOn ? 1 : 0, commandedRpm, actualRpm,
                                commandedRpm - actualRpm, amps, volts, uptakeOn ? 1 : 0,
                                uptakeCmdRpm, uptakeRpm, ball ? 1 : 0, intakePower,
                                applied[0], applied[1], applied[2], applied[3]));
                        if (now - lastFlushSec > 1.0) {
                            log.flush();
                            lastFlushSec = now;
                        }
                    } catch (IOException e) {
                        logPath = "LOG WRITE FAILED: " + e.getMessage();
                        log = null;
                    }
                }

                // Plain numbers so Panels can graph them.
                if (!fault.isEmpty()) telemetry.addLine(fault);
                telemetry.addData("Flywheel setpoint (RPM)", commandedRpm);
                telemetry.addData("Flywheel actual (RPM)", (double) Math.round(actualRpm));
                telemetry.addData("Flywheel error (RPM)", (double) Math.round(commandedRpm - actualRpm));
                telemetry.addData("Flywheel current (A)", amps);
                telemetry.addData("Battery (V)", volts);
                telemetry.addData("Uptake cmd (RPM)", uptakeCmdRpm);
                telemetry.addData("Uptake actual (RPM)", (double) Math.round(uptakeRpm));
                telemetry.addData("Ball", ball ? 1.0 : 0.0);
                telemetry.addData("Uptake", uptakeOn
                        ? (uptakeAllowed ? "ON" : "ON (BLOCKED: flywheel < "
                        + (int) Constants.UPTAKE_MIN_FLYWHEEL_RPM + " RPM)") : "OFF");
                telemetry.addData("Applied PIDF", "P %.3f I %.3f D %.3f F %.3f",
                        applied[0], applied[1], applied[2], applied[3]);
                telemetry.addData("Log", logPath);
                telemetry.update();
            }
        } finally {
            flywheel.setPower(0);
            uptake.setPower(0);
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
