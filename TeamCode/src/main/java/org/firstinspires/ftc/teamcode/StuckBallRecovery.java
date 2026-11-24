package org.firstinspires.ftc.teamcode;

/**
 * Stuck-ball recovery for the indexer, using the same logic as Shot Counter Test plus a retry
 * limit. Pure logic, no hardware: call {@link #update} once per loop and drive the indexer from
 * the returned {@link Action}.
 *
 * <ul>
 *   <li><b>Ball pending:</b> set when a ball moves past indexBeam1 (blocked -> clear), cleared
 *       when a shot is detected.
 *   <li><b>Shot:</b> the flywheel falling more than SHOT_DROP_RPM below its setpoint. Detection
 *       (re-)arms once the flywheel is back within SHOT_REARM_RPM, so spin-up does not count.
 *   <li><b>Stuck:</b> the indexer has fed forward for INDEXER_STUCK_TIMEOUT_SEC with a ball
 *       pending and no shot. The indexer then reverses at INDEXER_UNJAM_POWER for
 *       INDEXER_UNJAM_SEC and feeds again.
 *   <li><b>Give up:</b> after INDEXER_MAX_UNJAMS_PER_BALL unjams on the same ball, the indexer is
 *       stopped ({@link Action#STOPPED_STUCK}) instead of looping forward/reverse. It stays
 *       stopped until a shot is detected or the driver stops feeding (indexer off or an
 *       emergency button), which also gives the next attempt a fresh set of retries.
 * </ul>
 */
public class StuckBallRecovery {
    /** What the indexer should do this loop. */
    public enum Action {
        /** Normal control (feed forward if the driver has the indexer on). */
        NORMAL,
        /** Reverse at INDEXER_UNJAM_POWER. */
        UNJAM,
        /** Retries used up on this ball: stop the indexer until the driver clears it. */
        STOPPED_STUCK
    }

    private final boolean enabled;
    private boolean beamWasBlocked;
    private boolean shotArmed = false;
    private boolean ballPending = false;
    private double stuckSec = 0;
    private double unjamLeftSec = 0;
    private int unjamsThisBall = 0;
    private boolean gaveUp = false;
    private int shots = 0;
    private int unjams = 0;

    /**
     * @param enabled            false = track balls/shots only, never unjam or stop
     * @param beamBlockedAtStart a ball already in the beam at START is not counted until it passes
     */
    public StuckBallRecovery(boolean enabled, boolean beamBlockedAtStart) {
        this.enabled = enabled;
        this.beamWasBlocked = beamBlockedAtStart;
    }

    /**
     * Runs one step.
     *
     * @param beamBlocked     indexBeam1 sees a ball
     * @param setpointRpm     commanded flywheel RPM (0 when off)
     * @param actualRpm       measured flywheel RPM
     * @param feedRequested   the driver wants the indexer feeding forward (not an emergency)
     * @param forwardAllowed  the flywheel interlock allows the indexer forward
     * @param dtSec           seconds since the previous call
     */
    public Action update(boolean beamBlocked, double setpointRpm, double actualRpm,
                         boolean feedRequested, boolean forwardAllowed, double dtSec) {
        // A ball moved past the beam into the indexer.
        if (beamWasBlocked && !beamBlocked && !ballPending) {
            ballPending = true;
            resetRetries();
        }
        beamWasBlocked = beamBlocked;

        // Shot detection, only once the flywheel has reached speed.
        if (setpointRpm > 0) {
            double underspeed = setpointRpm - actualRpm;
            if (shotArmed && underspeed > Constants.SHOT_DROP_RPM) {
                shots++;
                shotArmed = false;
                ballPending = false;
                resetRetries();
            } else if (!shotArmed && underspeed < Constants.SHOT_REARM_RPM) {
                shotArmed = true;
            }
        } else {
            shotArmed = false;
        }

        if (!enabled) return Action.NORMAL;

        // The driver stopped feeding: cancel any unjam and start fresh next time.
        if (!feedRequested) {
            unjamLeftSec = 0;
            resetRetries();
            return Action.NORMAL;
        }
        if (gaveUp) return Action.STOPPED_STUCK;

        if (unjamLeftSec > 0) {
            unjamLeftSec = Math.max(0, unjamLeftSec - dtSec);
            if (unjamLeftSec > 0) return Action.UNJAM;
        }

        if (forwardAllowed && ballPending) {
            stuckSec += dtSec;
            if (stuckSec > Constants.INDEXER_STUCK_TIMEOUT_SEC) {
                stuckSec = 0;
                if (unjamsThisBall >= Constants.INDEXER_MAX_UNJAMS_PER_BALL) {
                    gaveUp = true;
                    return Action.STOPPED_STUCK;
                }
                unjamLeftSec = Constants.INDEXER_UNJAM_SEC;
                unjamsThisBall++;
                unjams++;
                return Action.UNJAM;
            }
        }
        return Action.NORMAL;
    }

    private void resetRetries() {
        stuckSec = 0;
        unjamsThisBall = 0;
        gaveUp = false;
    }

    /** Forget the pending ball (e.g. everything was reversed back out). */
    public void clearPendingBall() {
        ballPending = false;
        resetRetries();
    }

    public boolean isBallPending() {
        return ballPending;
    }

    public int getShots() {
        return shots;
    }

    /** Total unjams this OpMode. */
    public int getUnjams() {
        return unjams;
    }

    /** Unjams on the current pending ball. */
    public int getUnjamsThisBall() {
        return unjamsThisBall;
    }
}
