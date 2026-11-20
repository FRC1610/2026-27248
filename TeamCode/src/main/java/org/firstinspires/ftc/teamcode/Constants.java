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

    // ---- Driver controls ----
    public static final double TRIGGER_DEADBAND = 0.05;

    // ---- Turret: Axon MAX (programmed 355 deg) -> 1:3.2 overdrive kit -> 2.75:1 turret ----
    public static final double TURRET_SERVO_RANGE_DEG = 355.0;
    public static final double TURRET_OVERDRIVE_RATIO = 3.2;
    public static final double TURRET_REDUCTION_RATIO = 2.75;
    /** Turret degrees per 1.0 of servo position (~413). */
    public static final double TURRET_DEG_PER_SERVO_UNIT =
            TURRET_SERVO_RANGE_DEG * TURRET_OVERDRIVE_RATIO / TURRET_REDUCTION_RATIO;

    // ---- Servo positions ----
    /** Position the servo tests start the turret at; turret positions TBD from those tests. */
    public static final double SERVO_TEST_START_POSITION = 0.5;
    /** Flower wedge positions found with ServoTest (right servo is reversed in code). */
    public static final double FLOWER_WEDGE_UP = 0.90;
    public static final double FLOWER_WEDGE_DOWN = 0.22;
    /** Turret position Main 2026 holds until turret control is added. */
    public static final double TURRET_HOLD_POSITION = 0.5;
}
