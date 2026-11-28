package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;

/**
 * Live-tunable shooter values for Main 2026, editable in Panels > Configurables
 * (http://192.168.43.1:8001) while the OpMode runs. Defaults come from {@link Constants}.
 *
 * <p>Panels edits are static: they persist across OpMode runs until the Robot Controller app
 * restarts. Main 2026 shows "!! Panels tuning active" on the Driver Station whenever a value
 * differs from Constants. Copy good values into Constants to keep them.
 *
 * <p>Gamepad2 d-pad up/down (flywheel RPM) and right/left (uptake feed %) change
 * FLYWHEEL_RPM and UPTAKE_FEED_PERCENT here, so Panels and the gamepad stay in sync.
 */
@Configurable
public final class ShootTuning {
    private ShootTuning() {
    }

    // ---- Flywheel ----
    /** Flywheel setpoint (RPM) used when the flywheel is on (gamepad2 B). */
    public static double FLYWHEEL_RPM = Constants.FLYWHEEL_DEFAULT_RPM;
    /** Hub velocity PIDF; re-applied to the motor as soon as one changes. */
    public static double FLYWHEEL_P = Constants.FLYWHEEL_PIDF_P;
    public static double FLYWHEEL_I = Constants.FLYWHEEL_PIDF_I;
    public static double FLYWHEEL_D = Constants.FLYWHEEL_PIDF_D;
    public static double FLYWHEEL_F = Constants.FLYWHEEL_PIDF_F;

    // ---- Uptake / intake speeds ----
    /** Uptake speed (% of UPTAKE_MAX_RPM) while feeding a ball to the flywheel. */
    public static double UPTAKE_FEED_PERCENT = Constants.UPTAKE_DEFAULT_SPEED_PERCENT;
    /** Uptake speed (%) while loading a ball up to the beam. */
    public static double UPTAKE_LOAD_PERCENT = Constants.UPTAKE_LOAD_SPEED_PERCENT;
    /** Intake power while the shoot sequence loads the next ball. */
    public static double SHOOT_INTAKE_POWER = Constants.SHOOT_INTAKE_POWER;

    // ---- Shot detection and feed timing ----
    /** Shot = flywheel more than this below its setpoint (RPM). */
    public static double SHOT_DROP_RPM = Constants.SHOT_DROP_RPM;
    /** Shot detection re-arms within this of the setpoint (RPM). */
    public static double SHOT_REARM_RPM = Constants.SHOT_REARM_RPM;
    /** Feed the next ball once the flywheel is back within this below its setpoint (RPM). */
    public static double SHOOT_READY_TOLERANCE_RPM = Constants.SHOOT_READY_TOLERANCE_RPM;
    /** Extra wait (s) after the flywheel is ready before feeding. 0 = feed immediately. */
    public static double FEED_DELAY_AFTER_READY_SEC = 0.0;
    /**
     * Pause (s) after a shot before the intake/uptake load the next ball. 0 = load
     * immediately.
     */
    public static double LOAD_DELAY_AFTER_SHOT_SEC = 0.0;
    /** Shoot starts with a ball past the beam: reverse it to the beam first, then feed. */
    public static boolean PRESTAGE_PENDING_BALL = Constants.SHOOT_PRESTAGE_PENDING_BALL;

    // ---- Intake nudge (ball in the beam but not on the uptake rollers) ----
    public static double NUDGE_DELAY_SEC = Constants.SHOOT_NUDGE_DELAY_SEC;
    public static double NUDGE_PULSE_SEC = Constants.SHOOT_NUDGE_PULSE_SEC;
    public static double NUDGE_INTAKE_POWER = Constants.SHOOT_NUDGE_INTAKE_POWER;

    // ---- Stuck-ball recovery ----
    public static boolean STUCK_RECOVERY_ENABLED = Constants.UPTAKE_STUCK_RECOVERY_ENABLED;
    /** Feeding this long (s) with no shot = stuck. */
    public static double STUCK_TIMEOUT_SEC = Constants.UPTAKE_STUCK_TIMEOUT_SEC;
    /** Uptake power while backing a stuck ball up to the beam (negative = reverse). */
    public static double UNJAM_POWER = Constants.UPTAKE_UNJAM_POWER;
    public static double STUCK_REVERSE_MAX_SEC = Constants.UPTAKE_STUCK_REVERSE_MAX_SEC;
    public static double STUCK_PUSH_POWER = Constants.UPTAKE_STUCK_PUSH_POWER;
    public static double STUCK_PUSH_MAX_SEC = Constants.UPTAKE_STUCK_PUSH_MAX_SEC;
    public static int MAX_UNJAMS_PER_BALL = Constants.UPTAKE_MAX_UNJAMS_PER_BALL;

    /** Current values, in a fixed order (for logs and change detection). */
    public static Map<String, Double> values() {
        Map<String, Double> m = new LinkedHashMap<>();
        m.put("FLYWHEEL_RPM", FLYWHEEL_RPM);
        m.put("FLYWHEEL_P", FLYWHEEL_P);
        m.put("FLYWHEEL_I", FLYWHEEL_I);
        m.put("FLYWHEEL_D", FLYWHEEL_D);
        m.put("FLYWHEEL_F", FLYWHEEL_F);
        m.put("UPTAKE_FEED_PERCENT", UPTAKE_FEED_PERCENT);
        m.put("UPTAKE_LOAD_PERCENT", UPTAKE_LOAD_PERCENT);
        m.put("SHOOT_INTAKE_POWER", SHOOT_INTAKE_POWER);
        m.put("SHOT_DROP_RPM", SHOT_DROP_RPM);
        m.put("SHOT_REARM_RPM", SHOT_REARM_RPM);
        m.put("SHOOT_READY_TOLERANCE_RPM", SHOOT_READY_TOLERANCE_RPM);
        m.put("FEED_DELAY_AFTER_READY_SEC", FEED_DELAY_AFTER_READY_SEC);
        m.put("LOAD_DELAY_AFTER_SHOT_SEC", LOAD_DELAY_AFTER_SHOT_SEC);
        m.put("PRESTAGE_PENDING_BALL", PRESTAGE_PENDING_BALL ? 1.0 : 0.0);
        m.put("NUDGE_DELAY_SEC", NUDGE_DELAY_SEC);
        m.put("NUDGE_PULSE_SEC", NUDGE_PULSE_SEC);
        m.put("NUDGE_INTAKE_POWER", NUDGE_INTAKE_POWER);
        m.put("STUCK_RECOVERY_ENABLED", STUCK_RECOVERY_ENABLED ? 1.0 : 0.0);
        m.put("STUCK_TIMEOUT_SEC", STUCK_TIMEOUT_SEC);
        m.put("UNJAM_POWER", UNJAM_POWER);
        m.put("STUCK_REVERSE_MAX_SEC", STUCK_REVERSE_MAX_SEC);
        m.put("STUCK_PUSH_POWER", STUCK_PUSH_POWER);
        m.put("STUCK_PUSH_MAX_SEC", STUCK_PUSH_MAX_SEC);
        m.put("MAX_UNJAMS_PER_BALL", (double) MAX_UNJAMS_PER_BALL);
        return m;
    }

    /** The Constants defaults, same keys and order as {@link #values()}. */
    public static Map<String, Double> defaults() {
        Map<String, Double> m = new LinkedHashMap<>();
        m.put("FLYWHEEL_RPM", Constants.FLYWHEEL_DEFAULT_RPM);
        m.put("FLYWHEEL_P", Constants.FLYWHEEL_PIDF_P);
        m.put("FLYWHEEL_I", Constants.FLYWHEEL_PIDF_I);
        m.put("FLYWHEEL_D", Constants.FLYWHEEL_PIDF_D);
        m.put("FLYWHEEL_F", Constants.FLYWHEEL_PIDF_F);
        m.put("UPTAKE_FEED_PERCENT", Constants.UPTAKE_DEFAULT_SPEED_PERCENT);
        m.put("UPTAKE_LOAD_PERCENT", Constants.UPTAKE_LOAD_SPEED_PERCENT);
        m.put("SHOOT_INTAKE_POWER", Constants.SHOOT_INTAKE_POWER);
        m.put("SHOT_DROP_RPM", Constants.SHOT_DROP_RPM);
        m.put("SHOT_REARM_RPM", Constants.SHOT_REARM_RPM);
        m.put("SHOOT_READY_TOLERANCE_RPM", Constants.SHOOT_READY_TOLERANCE_RPM);
        m.put("FEED_DELAY_AFTER_READY_SEC", 0.0);
        m.put("LOAD_DELAY_AFTER_SHOT_SEC", 0.0);
        m.put("PRESTAGE_PENDING_BALL", Constants.SHOOT_PRESTAGE_PENDING_BALL ? 1.0 : 0.0);
        m.put("NUDGE_DELAY_SEC", Constants.SHOOT_NUDGE_DELAY_SEC);
        m.put("NUDGE_PULSE_SEC", Constants.SHOOT_NUDGE_PULSE_SEC);
        m.put("NUDGE_INTAKE_POWER", Constants.SHOOT_NUDGE_INTAKE_POWER);
        m.put("STUCK_RECOVERY_ENABLED", Constants.UPTAKE_STUCK_RECOVERY_ENABLED ? 1.0 : 0.0);
        m.put("STUCK_TIMEOUT_SEC", Constants.UPTAKE_STUCK_TIMEOUT_SEC);
        m.put("UNJAM_POWER", Constants.UPTAKE_UNJAM_POWER);
        m.put("STUCK_REVERSE_MAX_SEC", Constants.UPTAKE_STUCK_REVERSE_MAX_SEC);
        m.put("STUCK_PUSH_POWER", Constants.UPTAKE_STUCK_PUSH_POWER);
        m.put("STUCK_PUSH_MAX_SEC", Constants.UPTAKE_STUCK_PUSH_MAX_SEC);
        m.put("MAX_UNJAMS_PER_BALL", (double) Constants.UPTAKE_MAX_UNJAMS_PER_BALL);
        return m;
    }

    /** "NAME=value;..." for every value that differs from Constants ("" if none). */
    public static String changedFromDefaults() {
        Map<String, Double> defaults = defaults();
        StringBuilder sb = new StringBuilder();
        for (Map.Entry<String, Double> e : values().entrySet()) {
            if (Math.abs(e.getValue() - defaults.get(e.getKey())) > 1e-9) {
                if (sb.length() > 0) sb.append(';');
                sb.append(e.getKey()).append('=').append(format(e.getValue()));
            }
        }
        return sb.toString();
    }

    /** "NAME=value;..." for every value. */
    public static String snapshot() {
        StringBuilder sb = new StringBuilder();
        for (Map.Entry<String, Double> e : values().entrySet()) {
            if (sb.length() > 0) sb.append(';');
            sb.append(e.getKey()).append('=').append(format(e.getValue()));
        }
        return sb.toString();
    }

    public static String format(double v) {
        return String.format(Locale.US, "%.4g", v);
    }
}
