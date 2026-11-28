package org.firstinspires.ftc.teamcode;

/**
 * Turret starting direction for a launch (robot-relative). The turret starts here and tracking
 * begins from here; it also returns here if the Limelight stops sending data.
 */
public enum TurretDirection {
    /** Straight forward, toward the intake: the normal match-start position (0.23). */
    FORWARD(0.0),
    /**
     * Straight back (rear). -180 deg = turned clockwise, which is inside the turret's travel
     * (servo ~0.67); +180 (counter-clockwise) would be past the 90 deg left limit.
     */
    REVERSE(-180.0);

    /** Turret angle (deg, CCW+, 0 = forward). */
    public final double angleDeg;

    TurretDirection(double angleDeg) {
        this.angleDeg = angleDeg;
    }

    /** Servo position for this direction, kept within TURRET_MIN/MAX_POSITION. */
    public double servoPosition() {
        return Math.max(Constants.TURRET_MIN_POSITION, Math.min(Constants.TURRET_MAX_POSITION,
                TurretTracker.angleToPosition(angleDeg)));
    }
}
