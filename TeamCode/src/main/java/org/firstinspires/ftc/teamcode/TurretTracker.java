package org.firstinspires.ftc.teamcode;

/**
 * Keeps the turret (with the Limelight mounted on it) pointed at the selected AprilTag target.
 * Pure logic, no hardware: call {@link #update} once per loop and send the returned position to
 * the turret servo.
 *
 * <p>All angles are robot-relative turret angles in degrees, counter-clockwise positive, with 0
 * at TURRET_FORWARD_POSITION.
 *
 * <ul>
 *   <li><b>Vision:</b> when a fresh target is visible, tx is the turret's aim error (the camera
 *       turns with the turret); a fraction of it (TURRET_VISION_GAIN) is corrected each loop.
 *   <li><b>Pinpoint assist:</b> each loop the robot's heading change is subtracted from the aim,
 *       so the turret keeps pointing the same way on the field while the robot turns, including
 *       while the target is out of view. The Limelight then only corrects what is left.
 *   <li><b>Wrap-around:</b> the aim is moved to whichever equivalent angle (+/-360) the turret
 *       can reach, closest to where it is. When the target passes one end of the turret's travel,
 *       the turret swings around to the other end. If the target is in the unreachable gap, the
 *       turret waits at the end the target will come around to while the robot keeps turning, and
 *       the Limelight picks up the track when it comes into view.
 *   <li><b>Manual override:</b> a stick input turns the turret directly; the new direction is
 *       then held on the field.
 * </ul>
 *
 * <p>Until a target has been seen (or the turret moved manually) the turret holds its starting
 * position. The servo has no position feedback, so the last commanded angle is used as the
 * turret's angle.
 */
public class TurretTracker {
    public enum State {
        /** No target seen yet: holding the starting position. */
        WAITING_FOR_TARGET,
        /** Following a fresh Limelight target. */
        TRACKING,
        /** Target not visible: holding the last field direction using the Pinpoint. */
        HOLDING,
        /** Swinging to the other end after passing a limit; vision ignored until it settles. */
        SWINGING,
        /** Target out of the turret's reach: waiting at the end where it will reappear. */
        WAITING_AT_LIMIT,
        /** Operator is turning the turret with the stick. */
        MANUAL
    }

    private final double minAngleDeg;
    private final double maxAngleDeg;

    private double commandedDeg;
    private double aimDeg;
    private boolean hasAim = false;
    private double lastHeadingDeg = Double.NaN;
    private double swingTimeLeftSec = 0;
    private State state = State.WAITING_FOR_TARGET;

    /** @param startPosition servo position the turret is at when tracking starts */
    public TurretTracker(double startPosition) {
        double a = positionToAngle(Constants.TURRET_MIN_POSITION);
        double b = positionToAngle(Constants.TURRET_MAX_POSITION);
        minAngleDeg = Math.min(a, b);
        maxAngleDeg = Math.max(a, b);
        commandedDeg = clamp(positionToAngle(startPosition), minAngleDeg, maxAngleDeg);
        aimDeg = commandedDeg;
    }

    /** Turret angle (deg, CCW+, 0 = forward) for a servo position. */
    public static double positionToAngle(double position) {
        return (position - Constants.TURRET_FORWARD_POSITION)
                * Constants.TURRET_DEG_PER_SERVO_UNIT * Constants.TURRET_DIRECTION_SIGN;
    }

    /** Servo position for a turret angle (deg, CCW+, 0 = forward). */
    public static double angleToPosition(double angleDeg) {
        return Constants.TURRET_FORWARD_POSITION
                + angleDeg / (Constants.TURRET_DEG_PER_SERVO_UNIT * Constants.TURRET_DIRECTION_SIGN);
    }

    /**
     * Call after the Pinpoint heading is reset (or jumps for any other reason) so the jump is
     * not treated as the robot turning.
     */
    public void resetHeadingReference() {
        lastHeadingDeg = Double.NaN;
    }

    /**
     * Runs one control step.
     *
     * @param headingRad    Pinpoint heading (radians, CCW+); NaN/infinite disables the assist
     * @param targetVisible true only for a valid, fresh result from the selected pipeline
     * @param txDeg         Limelight tx (deg); ignored unless targetVisible
     * @param manualInput   -1..1, positive turns the turret counter-clockwise (left)
     * @param dtSec         seconds since the previous call
     * @return servo position to command
     */
    public double update(double headingRad, boolean targetVisible, double txDeg,
                         double manualInput, double dtSec) {
        dtSec = Math.max(0, Math.min(dtSec, 0.1));

        // Robot rotation since the last loop (deg, CCW+), from the Pinpoint.
        double rotationDeg = 0;
        double headingDeg = Math.toDegrees(headingRad);
        if (Double.isFinite(headingDeg)) {
            if (!Double.isNaN(lastHeadingDeg)) rotationDeg = wrap180(headingDeg - lastHeadingDeg);
            lastHeadingDeg = headingDeg;
        } else {
            lastHeadingDeg = Double.NaN;
        }

        // Where the turret is now relative to the field direction it pointed last loop.
        double turretNowDeg = commandedDeg - rotationDeg;
        if (hasAim) aimDeg -= rotationDeg;

        swingTimeLeftSec = Math.max(0, swingTimeLeftSec - dtSec);
        boolean visionUsable = targetVisible && Double.isFinite(txDeg) && swingTimeLeftSec == 0;

        if (Math.abs(manualInput) > Constants.TURRET_MANUAL_DEADBAND) {
            aimDeg = commandedDeg + manualInput * Constants.TURRET_MANUAL_RATE_DEG_PER_SEC * dtSec;
            hasAim = true;
            state = State.MANUAL;
        } else if (visionUsable) {
            // Camera turns with the turret: target to the right (tx > 0) means turn clockwise.
            double errorDeg = -Constants.TURRET_TX_SIGN * txDeg;
            if (Math.abs(errorDeg) < Constants.TURRET_TX_DEADBAND_DEG) errorDeg = 0;
            aimDeg = turretNowDeg + Constants.TURRET_VISION_GAIN * errorDeg;
            hasAim = true;
            state = State.TRACKING;
        } else if (!hasAim) {
            state = State.WAITING_FOR_TARGET;
            return angleToPosition(commandedDeg);
        } else {
            state = State.HOLDING;
        }

        double targetDeg = reachableAngle(aimDeg, rotationDeg, dtSec);
        double moveDeg = Math.abs(targetDeg - commandedDeg);
        if (moveDeg > 90) {
            // Big swing (wrap to the other end): ignore the Limelight until it should be done.
            swingTimeLeftSec = moveDeg / Constants.TURRET_SWING_SPEED_DEG_PER_SEC;
            state = State.SWINGING;
        } else if (swingTimeLeftSec > 0 && state != State.MANUAL) {
            state = State.SWINGING;
        }
        commandedDeg = targetDeg;
        // Keep the aim close to the commanded angle so it doesn't wind up past +/-360.
        aimDeg = commandedDeg + wrap180(aimDeg - commandedDeg);
        return angleToPosition(commandedDeg);
    }

    /**
     * The turret angle to command for an aim: the equivalent (+/-360) reachable angle closest to
     * the current one, or, if none is reachable, the end the target will come around to.
     */
    private double reachableAngle(double aim, double rotationDeg, double dtSec) {
        double best = Double.NaN;
        for (int k = -2; k <= 2; k++) {
            double candidate = aim + 360.0 * k;
            if (candidate >= minAngleDeg && candidate <= maxAngleDeg
                    && (Double.isNaN(best)
                    || Math.abs(candidate - commandedDeg) < Math.abs(best - commandedDeg))) {
                best = candidate;
            }
        }
        if (!Double.isNaN(best)) return best;

        // Unreachable gap. The aim moves opposite to the robot's rotation.
        double turnRate = dtSec > 0 ? rotationDeg / dtSec : 0;
        if (state != State.MANUAL) state = State.WAITING_AT_LIMIT;
        if (turnRate < -Constants.TURRET_WRAP_MIN_TURN_RATE_DEG_PER_SEC) {
            // Robot turning clockwise: the aim passes the max end and reappears at the min end.
            return minAngleDeg;
        }
        if (turnRate > Constants.TURRET_WRAP_MIN_TURN_RATE_DEG_PER_SEC) {
            return maxAngleDeg;
        }
        // Not turning: wait at whichever end is closer to the target.
        double pastMax = wrap360(aim - maxAngleDeg);
        double beforeMin = wrap360(minAngleDeg - aim);
        return pastMax <= beforeMin ? maxAngleDeg : minAngleDeg;
    }

    public State getState() {
        return state;
    }

    /** Last commanded turret angle (deg, CCW+, 0 = forward). */
    public double getCommandedAngleDeg() {
        return commandedDeg;
    }

    /** Where the tracker wants the turret to point (deg, CCW+), before limits. */
    public double getAimAngleDeg() {
        return aimDeg;
    }

    public double getMinAngleDeg() {
        return minAngleDeg;
    }

    public double getMaxAngleDeg() {
        return maxAngleDeg;
    }

    private static double wrap180(double deg) {
        double w = wrap360(deg);
        return w >= 180 ? w - 360 : w;
    }

    private static double wrap360(double deg) {
        double w = deg % 360.0;
        return w < 0 ? w + 360 : w;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
