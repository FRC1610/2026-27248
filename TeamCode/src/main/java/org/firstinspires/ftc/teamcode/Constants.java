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

    // ---- Starting heading: robot front faces the drivers' RIGHT at INIT (both alliances) ----
    // Robot heading at INIT in the drivers' frame (CCW+, 0 = away from drivers), so right = -90.
    // Added to the Pinpoint heading in Main 2026 until a Back/Share heading reset.
    public static final double BLUE_START_HEADING_OFFSET_RAD = -Math.PI / 2.0;
    public static final double RED_START_HEADING_OFFSET_RAD = -Math.PI / 2.0;

    // ---- Pinpoint odometry: forward pod's sideways offset and strafe pod's forward offset (in) ----
    public static final double FORWARD_POD_Y = -3.375;
    public static final double STRAFE_POD_X = 5.5625;

    // ---- Limelight pipelines (slots configured in the Limelight web interface) ----
    public static final int PIPELINE_RED_SCORING_SIDE = 0;
    public static final int PIPELINE_RED_AUDIENCE_SIDE = 1;
    public static final int PIPELINE_BLUE_AUDIENCE_SIDE = 2;
    public static final int PIPELINE_BLUE_SCORING_SIDE = 3;
    public static final int PIPELINE_ALL_RED_TAGS = 8;
    public static final int PIPELINE_ALL_BLUE_TAGS = 9;
    /** Default pipeline for each alliance, chosen from allianceButton (low = red, high = blue). */
    public static final int RED_VISION_PIPELINE = PIPELINE_ALL_RED_TAGS;
    public static final int BLUE_VISION_PIPELINE = PIPELINE_ALL_BLUE_TAGS;
    public static final int LIMELIGHT_POLL_RATE_HZ = 100;
    /**
     * If the Limelight sends no new result for this long (ms), it is treated as disconnected and
     * turret tracking returns the turret to TURRET_HOLD_POSITION. Normal result age is well
     * under 100 ms. Starting value.
     */
    public static final double LIMELIGHT_LOST_TIMEOUT_MS = 500.0;

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

    // ---- goBILDA Prism LED driver (I2C, configured as "prism") ----
    /**
     * Artboards saved on the Prism by Prism Light Setup (run once). Artboard 0 is the power-on
     * default; the TeleOps only switch between these.
     */
    public static final int PRISM_ARTBOARD_POWER_ON = 0;      // yellow snakes (power on / idle)
    public static final int PRISM_ARTBOARD_RED = 1;           // red snakes (red alliance)
    public static final int PRISM_ARTBOARD_BLUE = 2;          // blue snakes (blue alliance)
    public static final int PRISM_ARTBOARD_BALL_SPINUP = 3;   // purple snakes
    public static final int PRISM_ARTBOARD_BALL_READY = 4;    // green snakes
    /** LEDs in the chain (indexes 0 to PRISM_LED_COUNT - 1); every animation covers all of them. */
    public static final int PRISM_LED_COUNT = 48;
    /** LED brightness (0-100) for all artboards. */
    public static final int PRISM_BRIGHTNESS = 50;
    /** Snake speed (0-1) for all artboards. */
    public static final float PRISM_SNAKE_SPEED = 0.3f;
    /** LEDs between consecutive snakes on all artboards. */
    public static final int PRISM_SNAKE_SPACING = 4;
    /** Flywheel counts as "at speed" (green) within this many RPM of its setpoint. */
    public static final double LIGHTS_FLYWHEEL_READY_TOLERANCE_RPM = 50.0;

    // ---- Driver controls ----
    public static final double TRIGGER_DEADBAND = 0.05;

    // ---- Turret: Axon MAX (programmed 355 deg) -> 1:3.2 overdrive kit -> 2.75:1 turret ----
    public static final double TURRET_SERVO_RANGE_DEG = 355.0;
    public static final double TURRET_OVERDRIVE_RATIO = 3.2;
    public static final double TURRET_REDUCTION_RATIO = 2.75;
    /** Turret degrees per 1.0 of servo position from the specs above (~413); reference only. */
    public static final double TURRET_SPEC_DEG_PER_SERVO_UNIT =
            TURRET_SERVO_RANGE_DEG * TURRET_OVERDRIVE_RATIO / TURRET_REDUCTION_RATIO;
    /**
     * Measured turret degrees per 1.0 of servo position (~403), with the full-range PWM below:
     * 0.00 = 90 deg left, 0.23 = forward, 0.45 = 90 deg right, 0.67 = 180 deg (rear), so
     * 270 deg over 0.67. All four points fit within ~3 deg (less than one 0.01 step).
     */
    public static final double TURRET_DEG_PER_SERVO_UNIT = 270.0 / 0.67;
    /**
     * Turret PWM pulse range (microseconds), applied in code. Same as the SDK's built-in
     * "Full Range Servo" type (500-2500); a plain "Servo" defaults to 600-2400, which would
     * only give ~90% of the Axon's programmed travel.
     */
    public static final double TURRET_PWM_MIN_US = 500.0;
    public static final double TURRET_PWM_MAX_US = 2500.0;
    /**
     * Safe turret servo limits; every turret command is clipped to this range. Measured:
     * 0.00 = counter-clockwise limit, 90 deg left of forward;
     * 0.87 = clockwise limit, ~258 deg right of forward (past the rear).
     * About 350 deg of travel in total, so there is a ~10 deg gap the turret can't reach,
     * just behind 90 deg left; TurretTracker wraps around it.
     */
    public static final double TURRET_MIN_POSITION = 0.00;
    public static final double TURRET_MAX_POSITION = 0.87;
    /** Turret Limit Test step per d-pad press (0.01 x ~403 deg/unit = ~4.0 deg of turret). */
    public static final double TURRET_LIMIT_TEST_STEP = 0.01;

    // ---- Turret tracking (TurretTracker). Angles are robot-relative, CCW-positive (deg). ----
    /**
     * Whether Main 2026 starts with turret tracking on. Gamepad2 X toggles tracking during the
     * match, and a ball newly reaching the indexer beam turns it on. While off, the turret holds
     * TURRET_HOLD_POSITION (no Limelight tracking, no manual stick).
     */
    public static final boolean TURRET_TRACKING_ON_AT_START = false;
    /** Servo position where the turret points straight forward, toward the intake (measured). */
    public static final double TURRET_FORWARD_POSITION = 0.23;
    /**
     * +1 if increasing servo position turns the turret counter-clockwise (left, seen from
     * above), -1 if clockwise. Measured: 0.00 is 90 deg left, 0.45 is 90 deg right, so
     * increasing = clockwise.
     */
    public static final int TURRET_DIRECTION_SIGN = -1;
    /**
     * +1 if Limelight tx is positive when the target is to the RIGHT of the crosshair, -1 if
     * left. Team spec: tx > 0 = turret too far left (turn clockwise/right), tx < 0 = turret too
     * far right (turn counter-clockwise/left), which is +1.
     */
    public static final int TURRET_TX_SIGN = 1;
    /**
     * Fraction of the Limelight error corrected per correction. Corrections are made only on a
     * new Limelight frame taken after the turret has settled from the previous one, so each
     * tx is used once. Slightly below 1 to avoid overshoot. Starting value; tune.
     */
    public static final double TURRET_VISION_GAIN = 0.8;
    /**
     * Extra wait (s) after a correction's estimated servo travel time before trusting the
     * Limelight again: covers camera capture/processing latency and servo settling.
     */
    public static final double TURRET_VISION_SETTLE_SEC = 0.15;
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
    /** Position the servo tests start the turret at (same as TURRET_HOLD_POSITION). */
    public static final double SERVO_TEST_START_POSITION = 0.23;
    /** Flower wedge positions found with ServoTest (right servo is reversed in code). */
    public static final double FLOWER_WEDGE_UP = 0.90;
    public static final double FLOWER_WEDGE_DOWN = 0.22;
    /**
     * Turret starting position: facing forward, toward the intake (= TURRET_FORWARD_POSITION).
     * Main 2026 starts here and returns here when aiming is off.
     */
    public static final double TURRET_HOLD_POSITION = 0.23;
}
