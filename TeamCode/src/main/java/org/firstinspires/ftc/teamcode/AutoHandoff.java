package org.firstinspires.ftc.teamcode;

/**
 * Carries the robot's state from the end of autonomous into TeleOp. Static fields live as long as
 * the Robot Controller app, so they survive the switch from an autonomous OpMode to Main 2026
 * (but not an app restart or robot power cycle).
 *
 * <p>Autonomous calls {@link #save} when it ends; Main 2026 calls {@link #take} once at INIT.
 * A handoff is used only once and only if it is at most AUTO_HANDOFF_MAX_AGE_SEC old, so a later
 * TeleOp run (e.g. practice after moving the robot) starts fresh.
 */
public final class AutoHandoff {
    private AutoHandoff() {
    }

    /** What autonomous left behind. */
    public static final class State {
        /** Pinpoint heading (rad) at the end of autonomous; NaN if it was not available. */
        public final double pinpointHeadingRad;
        /** Last commanded turret servo position. */
        public final double turretPosition;
        /** Flower wedge preset at the end of autonomous: true = UP, false = DOWN. */
        public final boolean flowerWedgeUp;
        /** Seconds since autonomous ended. */
        public final double ageSec;

        State(double pinpointHeadingRad, double turretPosition, boolean flowerWedgeUp,
              double ageSec) {
            this.pinpointHeadingRad = pinpointHeadingRad;
            this.turretPosition = turretPosition;
            this.flowerWedgeUp = flowerWedgeUp;
            this.ageSec = ageSec;
        }
    }

    private static boolean saved = false;
    private static double pinpointHeadingRad = Double.NaN;
    private static double turretPosition = Double.NaN;
    private static boolean flowerWedgeUp = true;
    private static long savedNanos = 0;

    /** Call when autonomous ends (also when stopped early). */
    public static synchronized void save(double headingRad, double turret, boolean wedgeUp) {
        pinpointHeadingRad = headingRad;
        turretPosition = turret;
        flowerWedgeUp = wedgeUp;
        savedNanos = System.nanoTime();
        saved = true;
    }

    /**
     * Returns the handoff and clears it, or null if there is none or it is older than
     * Constants.AUTO_HANDOFF_MAX_AGE_SEC.
     */
    public static synchronized State take() {
        if (!saved) return null;
        saved = false;
        double ageSec = (System.nanoTime() - savedNanos) / 1e9;
        if (ageSec > Constants.AUTO_HANDOFF_MAX_AGE_SEC) return null;
        return new State(pinpointHeadingRad, turretPosition, flowerWedgeUp, ageSec);
    }
}
