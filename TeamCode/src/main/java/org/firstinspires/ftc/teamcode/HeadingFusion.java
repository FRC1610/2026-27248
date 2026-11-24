package org.firstinspires.ftc.teamcode;

/**
 * Picks the robot heading for driving from the Pinpoint, with the Control Hub IMU as a backup.
 * Pure logic, no hardware: call {@link #update} once per loop.
 *
 * <p>All headings are radians, counter-clockwise positive, in the Pinpoint's frame (0 = the
 * robot's facing when the Pinpoint was last zeroed). The start offset is added by the caller.
 *
 * <ul>
 *   <li><b>PINPOINT</b> (normal): the Pinpoint heading is used. The difference to the IMU
 *       heading is followed slowly (OFFSET_FOLLOW_RATE per loop), which absorbs slow IMU drift
 *       but still shows a frozen or jumping Pinpoint as a disagreement.
 *   <li><b>IMU</b> (backup): if the Pinpoint is not ready, or its heading jumps more than
 *       HEADING_JUMP_DEG away from the IMU (e.g. it rebooted and reset to 0), the IMU heading
 *       plus the remembered difference is used, so driving continues without a jump. The Pinpoint
 *       is trusted again after agreeing with the IMU for PINPOINT_RETRUST_LOOPS loops in a row;
 *       {@link #takeRealignRequest} asks the caller to write the correct heading back to it.
 *   <li><b>NONE</b>: neither is usable; the caller should drive robot-centric.
 * </ul>
 */
public class HeadingFusion {
    public enum Source { PINPOINT, IMU, NONE }

    /** Loops to wait between two realign requests (lets the Pinpoint apply the last one). */
    private static final int REALIGN_COOLDOWN_LOOPS = 10;
    /** Fraction of the Pinpoint-IMU disagreement absorbed into the offset each loop. */
    private static final double OFFSET_FOLLOW_RATE = 0.02;

    /** Pinpoint heading minus IMU heading (rad), learned while the Pinpoint is trusted. */
    private double imuOffset = Double.NaN;
    private boolean pinpointTrusted = true;
    private int agreeLoops = 0;
    private int realignCooldown = 0;
    private double realignHeading = Double.NaN;
    private Source source = Source.NONE;

    /**
     * Sets the current heading (e.g. 0 after the Pinpoint is zeroed at START or by the driver).
     * Works even if the Pinpoint is down, as long as the IMU reads.
     */
    public void reset(double headingRad, double imuRad) {
        imuOffset = Double.isFinite(imuRad) ? wrap(headingRad - imuRad) : Double.NaN;
        agreeLoops = 0;
        realignCooldown = 0;
        realignHeading = Double.NaN;
    }

    /**
     * @param pinpointReady Pinpoint status is READY (and its data was read without an error)
     * @param pinpointRad   Pinpoint heading (rad)
     * @param imuRad        IMU yaw (rad), NaN if it could not be read
     * @return heading to drive with (rad), or NaN if there is none
     */
    public double update(boolean pinpointReady, double pinpointRad, double imuRad) {
        boolean pinpointOk = pinpointReady && Double.isFinite(pinpointRad);
        boolean imuOk = Double.isFinite(imuRad);
        boolean canCompare = imuOk && !Double.isNaN(imuOffset);
        if (realignCooldown > 0) realignCooldown--;

        if (!pinpointOk) {
            // Lost: when it comes back it must agree with the IMU before it is used again.
            pinpointTrusted = false;
            agreeLoops = 0;
        } else if (canCompare) {
            double expected = wrap(imuRad + imuOffset);
            double errorDeg = Math.abs(Math.toDegrees(wrap(pinpointRad - expected)));
            if (pinpointTrusted) {
                if (errorDeg > Constants.HEADING_JUMP_DEG) {
                    pinpointTrusted = false;
                    agreeLoops = 0;
                    requestRealign(expected);
                }
            } else if (errorDeg <= Constants.HEADING_AGREE_DEG) {
                if (++agreeLoops >= Constants.PINPOINT_RETRUST_LOOPS) pinpointTrusted = true;
            } else {
                agreeLoops = 0;
                requestRealign(expected);
            }
        } else if (!pinpointTrusted) {
            // No IMU to check against: take the Pinpoint back as it is.
            pinpointTrusted = true;
        }

        if (pinpointOk && pinpointTrusted) {
            if (canCompare) {
                double error = wrap(pinpointRad - wrap(imuRad + imuOffset));
                imuOffset = wrap(imuOffset + OFFSET_FOLLOW_RATE * error);
            } else if (imuOk) {
                imuOffset = wrap(pinpointRad - imuRad);
            }
            source = Source.PINPOINT;
            return pinpointRad;
        }
        if (canCompare) {
            source = Source.IMU;
            return wrap(imuRad + imuOffset);
        }
        source = Source.NONE;
        return Double.NaN;
    }

    private void requestRealign(double headingRad) {
        if (realignCooldown > 0) return;
        realignHeading = headingRad;
        realignCooldown = REALIGN_COOLDOWN_LOOPS;
    }

    /**
     * Heading (rad) the caller should write to the Pinpoint so it matches the IMU again, or NaN
     * if none is needed. Each request is returned once.
     */
    public double takeRealignRequest() {
        double h = realignHeading;
        realignHeading = Double.NaN;
        return h;
    }

    public Source getSource() {
        return source;
    }

    private static double wrap(double rad) {
        double w = rad % (2 * Math.PI);
        if (w >= Math.PI) w -= 2 * Math.PI;
        if (w < -Math.PI) w += 2 * Math.PI;
        return w;
    }
}
