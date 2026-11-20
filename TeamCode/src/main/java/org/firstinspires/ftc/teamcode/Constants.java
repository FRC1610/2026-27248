package org.firstinspires.ftc.teamcode;

/** Robot-wide tuning values and physical constants shared by OpModes and RobotHardware. */
public final class Constants {
    private Constants() {}

    // ---- Drive ----
    public static final double DRIVE_FULL_SPEED = 0.85;
    /** Speed scale used when the left-stick-click slow mode is active. */
    public static final double DRIVE_SLOW_SPEED = 0.5;
    /** Multiplier on strafe input to counter mecanum strafe losses. */
    public static final double STRAFE_CORRECTION = 1.1;
    /**
     * Seconds for a drive wheel's power to ramp from 0 to full (1.0), to limit current spikes
     * and brownouts. Slowing down/stopping is not ramped. Starting value; tune on the robot.
     */
    public static final double DRIVE_RAMP_UP_SECONDS = 0.4;

    // ---- Starting heading (last year's sideways start: blue faces right, red faces left) ----
    public static final double BLUE_START_HEADING_OFFSET_RAD = Math.PI / 2.0;
    public static final double RED_START_HEADING_OFFSET_RAD = -Math.PI / 2.0;

    // ---- Pinpoint odometry: forward pod's sideways offset and strafe pod's forward offset (in) ----
    public static final double FORWARD_POD_Y = -3.375;
    public static final double STRAFE_POD_X = 5.5625;

    // ---- Limelight pipelines ----
    public static final int BLUE_VISION_PIPELINE = 0;
    public static final int RED_VISION_PIPELINE = 4;
    public static final int LIMELIGHT_POLL_RATE_HZ = 100;

    // ---- Flywheel: goBILDA 5203 Yellow Jacket 6000 RPM (1:1), direct drive to the flywheel ----
    public static final double FLYWHEEL_TICKS_PER_REV = 28.0;
    public static final double FLYWHEEL_DEFAULT_RPM = 2575.0;
    public static final double FLYWHEEL_MAX_RPM = 6000.0;
    public static final double FLYWHEEL_RPM_STEP = 100.0;

    // ---- Indexer (values from the original TestMode) ----
    public static final double INDEXER_TICKS_PER_REV = 103.6;
    public static final double INDEXER_MAX_RPM = 1620.0;
    public static final double INDEXER_DEFAULT_SPEED_PERCENT = 75.0;
    public static final double INDEXER_SPEED_STEP_PERCENT = 5.0;
    /**
     * The indexer may only run forward while the flywheel is commanded on and measures at least
     * this speed. Reverse is always allowed.
     */
    public static final double INDEXER_MIN_FLYWHEEL_RPM = 2000.0;

    // ---- Indexer beam sensor (indexBeam1) ----
    /**
     * indexBeam1 reading (DigitalChannel.getState()) that means a ball is blocking the beam.
     * PLACEHOLDER: false (LOW), typical for break-beam receivers. Verify with Flywheel Intake
     * Test, which shows the raw HIGH/LOW reading.
     */
    public static final boolean INDEX_BEAM_BALL_STATE = false;

    // ---- Driver controls ----
    public static final double TRIGGER_DEADBAND = 0.05;

    // ---- Turret: Axon MAX (programmed 355 deg) -> 1:3.2 overdrive kit -> 2.75:1 turret ----
    public static final double TURRET_SERVO_RANGE_DEG = 355.0;
    public static final double TURRET_OVERDRIVE_RATIO = 3.2;
    public static final double TURRET_REDUCTION_RATIO = 2.75;
    /** Turret degrees per 1.0 of servo position (~413), assuming the full-range PWM below. */
    public static final double TURRET_DEG_PER_SERVO_UNIT =
            TURRET_SERVO_RANGE_DEG * TURRET_OVERDRIVE_RATIO / TURRET_REDUCTION_RATIO;
    /**
     * Turret PWM pulse range (microseconds), applied in code. Same as the SDK's built-in
     * "Full Range Servo" type (500-2500); a plain "Servo" defaults to 600-2400, which would
     * only give ~90% of the Axon's programmed travel.
     */
    public static final double TURRET_PWM_MIN_US = 500.0;
    public static final double TURRET_PWM_MAX_US = 2500.0;
    /**
     * Safe turret servo limits; every turret command is clipped to this range. PLACEHOLDERS:
     * both equal TURRET_HOLD_POSITION (turret locked) until measured with Turret Limit Test.
     */
    public static final double TURRET_MIN_POSITION = 0.5;
    public static final double TURRET_MAX_POSITION = 0.5;
    /** Turret Limit Test step per d-pad press (0.01 x ~413 deg/unit = ~4.1 deg of turret). */
    public static final double TURRET_LIMIT_TEST_STEP = 0.01;

    // ---- Turret tracking (TurretTracker). Angles are robot-relative, CCW-positive (deg). ----
    /** Servo position where the turret points straight forward. PLACEHOLDER: verify. */
    public static final double TURRET_FORWARD_POSITION = 0.5;
    /**
     * +1 if increasing servo position turns the turret counter-clockwise (left, seen from
     * above), -1 if clockwise. PLACEHOLDER: verify with Turret Limit Test.
     */
    public static final int TURRET_DIRECTION_SIGN = 1;
    /**
     * +1 if Limelight tx is positive when the target is to the RIGHT of the crosshair (the
     * usual Limelight convention; not stated in the docs), -1 if left. Verify on the robot.
     */
    public static final int TURRET_TX_SIGN = 1;
    /**
     * Fraction of the Limelight error corrected per loop. Below 1 because the servo has no
     * position feedback and the image is slightly old. Starting value; tune.
     */
    public static final double TURRET_VISION_GAIN = 0.5;
    /** Ignore Limelight errors smaller than this (deg) to stop jitter. */
    public static final double TURRET_TX_DEADBAND_DEG = 0.5;
    /** Ignore Limelight results older than this (ms). */
    public static final double TURRET_MAX_VISION_AGE_MS = 100.0;
    /** Manual override (GP2 right stick X) turret speed at full stick (deg/s). */
    public static final double TURRET_MANUAL_RATE_DEG_PER_SEC = 120.0;
    public static final double TURRET_MANUAL_DEADBAND = 0.1;
    /**
     * Estimated turret swing speed (deg/s), used to ignore the Limelight while the turret is
     * still swinging after a large move (e.g. wrapping to the other end). Conservative estimate;
     * goBILDA lists ~267 RPM at the kit output at 6 V, i.e. ~580 deg/s at the turret.
     */
    public static final double TURRET_SWING_SPEED_DEG_PER_SEC = 400.0;
    /**
     * Robot turn rate (deg/s) above which, if the target is outside the turret's reach, the
     * turret waits at the end the target will come around to (instead of the nearest end).
     */
    public static final double TURRET_WRAP_MIN_TURN_RATE_DEG_PER_SEC = 10.0;

    // ---- Servo positions ----
    /** Position the servo tests start the turret at; turret positions TBD from those tests. */
    public static final double SERVO_TEST_START_POSITION = 0.5;
    /** Flower wedge positions found with ServoTest (right servo is reversed in code). */
    public static final double FLOWER_WEDGE_UP = 0.90;
    public static final double FLOWER_WEDGE_DOWN = 0.22;
    /** Turret position Main 2026 holds until turret control is added. */
    public static final double TURRET_HOLD_POSITION = 0.5;
}
