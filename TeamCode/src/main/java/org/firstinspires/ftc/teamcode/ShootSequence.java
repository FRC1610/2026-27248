package org.firstinspires.ftc.teamcode;

/**
 * Ball loading and the hold-to-shoot sequence for Main 2026. Pure logic, no hardware: call
 * {@link #update} once per loop and drive the intake/indexer from the returned {@link State}.
 *
 * <p>indexBeam1 is at the indexer entrance. A ball "staged" at the beam is blocking it; a ball
 * "pending" has moved past the beam into the indexer (blocked -> clear) and has not been shot.
 *
 * <ul>
 *   <li><b>Loading (shoot not held):</b> while the intake runs forward, the indexer also runs
 *       until a ball is staged at the beam (or one is already pending). The indexer then stops;
 *       the intake keeps following the driver.
 *   <li><b>Shooting (shoot held, flywheel on):</b> one ball at a time. With a ball staged or
 *       pending, wait until the flywheel is ready (back within SHOOT_READY_TOLERANCE_RPM below
 *       its setpoint after the previous shot), then run the indexer with the intake stopped
 *       until the shot is detected. With no ball, run the intake and indexer until the next ball
 *       is staged. Repeat.
 *   <li><b>Shot:</b> the flywheel falling more than SHOT_DROP_RPM below its setpoint. Detection
 *       (re-)arms once the flywheel is back within SHOT_REARM_RPM, so spin-up does not count.
 *   <li><b>Stuck ball:</b> feeding for INDEXER_STUCK_TIMEOUT_SEC (from the start of feeding, or
 *       from the ball passing the beam) with no shot. The indexer reverses at
 *       INDEXER_UNJAM_POWER until the ball is seen at the beam again (at most
 *       INDEXER_STUCK_REVERSE_MAX_SEC), then pushes at INDEXER_STUCK_PUSH_POWER (100%) until the
 *       shot. If no shot within INDEXER_STUCK_PUSH_MAX_SEC it reverses and pushes again, up to
 *       INDEXER_MAX_UNJAMS_PER_BALL times, then stops ({@link State#STUCK}) until shoot is
 *       released.
 * </ul>
 */
public class ShootSequence {
    /** What the intake and indexer should do this loop. */
    public enum State {
        /** No ball staged or pending and not loading: indexer off, intake follows the driver. */
        IDLE,
        /** Indexer forward (loading speed) until a ball is staged. Intake forward while shooting. */
        LOADING,
        /** Shoot not held, ball staged/pending: indexer off, intake follows the driver. */
        HOLDING,
        /** Shoot held, ball staged/pending, flywheel not ready yet: indexer and intake off. */
        WAITING,
        /** Indexer forward (feed speed) until the shot; intake off. */
        FEEDING,
        /** Stuck ball: indexer reverse until the ball is back at the beam; intake off. */
        REVERSING,
        /** Stuck ball: indexer forward at 100% until the shot; intake off. */
        PUSHING,
        /** Retries used up: indexer and intake off until shoot is released. */
        STUCK
    }

    private final boolean recoveryEnabled;
    private State state = State.IDLE;
    private boolean beamWasBlocked;
    private boolean ballPending = false;
    private boolean shotArmed = false;
    private boolean flywheelReady = false;
    private double feedSec = 0;
    private double reverseSec = 0;
    private double pushSec = 0;
    private int unjamsThisBall = 0;
    private int shots = 0;
    private int unjams = 0;

    /**
     * @param recoveryEnabled    false = never reverse/push a stuck ball, just keep feeding
     * @param beamBlockedAtStart a ball in the beam at START is staged, not yet pending
     */
    public ShootSequence(boolean recoveryEnabled, boolean beamBlockedAtStart) {
        this.recoveryEnabled = recoveryEnabled;
        this.beamWasBlocked = beamBlockedAtStart;
    }

    /**
     * Runs one step.
     *
     * @param beamBlocked   indexBeam1 sees a ball
     * @param shootActive   the shoot command is held and the flywheel is on (and no emergency)
     * @param intakeForward the driver is running the intake forward
     * @param setpointRpm   commanded flywheel RPM (0 when off)
     * @param actualRpm     measured flywheel RPM
     * @param dtSec         seconds since the previous call
     */
    public State update(boolean beamBlocked, boolean shootActive, boolean intakeForward,
                        double setpointRpm, double actualRpm, double dtSec) {
        // A ball moved past the beam into the indexer.
        if (beamWasBlocked && !beamBlocked && !ballPending) {
            ballPending = true;
            feedSec = 0; // stuck timer now measures the trip from the beam to the flywheel
        }
        beamWasBlocked = beamBlocked;

        // Shot detection and the "ready to feed" latch, only once the flywheel has reached speed.
        boolean shot = false;
        if (setpointRpm > 0) {
            double underspeed = setpointRpm - actualRpm;
            if (shotArmed && underspeed > Constants.SHOT_DROP_RPM) {
                shot = true;
                shots++;
                shotArmed = false;
                flywheelReady = false;
                ballPending = false;
            } else if (!shotArmed && underspeed < Constants.SHOT_REARM_RPM) {
                shotArmed = true;
            }
            if (shotArmed && underspeed <= Constants.SHOOT_READY_TOLERANCE_RPM) {
                flywheelReady = true; // latched until the next shot (ignores speed-reading noise)
            }
        } else {
            shotArmed = false;
            flywheelReady = false;
        }
        if (shot) {
            unjamsThisBall = 0;
            state = State.IDLE; // pick the next step below
        }

        if (!shootActive) {
            // Releasing shoot after giving up: start fresh next time.
            if (state == State.STUCK) ballPending = false;
            unjamsThisBall = 0;
            if (intakeForward && !beamBlocked && !ballPending) {
                state = State.LOADING;
            } else {
                state = beamBlocked || ballPending ? State.HOLDING : State.IDLE;
            }
            return state;
        }

        // ---- Shoot held ----
        if (state == State.STUCK) return state;
        if (state == State.REVERSING) {
            reverseSec += dtSec;
            if (!beamBlocked && reverseSec < Constants.INDEXER_STUCK_REVERSE_MAX_SEC) return state;
            state = State.PUSHING; // ball back at the beam (or reverse timed out)
            pushSec = 0;
            return state;
        }
        if (state == State.PUSHING) {
            pushSec += dtSec;
            if (pushSec < Constants.INDEXER_STUCK_PUSH_MAX_SEC) return state;
            return startUnjam(beamBlocked);
        }

        if (!ballPending && !beamBlocked) {
            state = State.LOADING;
            return state;
        }
        if (!flywheelReady) {
            state = State.WAITING;
            return state;
        }
        if (state != State.FEEDING) {
            state = State.FEEDING;
            feedSec = 0;
        }
        feedSec += dtSec;
        if (recoveryEnabled && feedSec > Constants.INDEXER_STUCK_TIMEOUT_SEC) {
            return startUnjam(beamBlocked);
        }
        return state;
    }

    private State startUnjam(boolean beamBlocked) {
        if (!recoveryEnabled) {
            state = State.FEEDING;
            return state;
        }
        if (unjamsThisBall >= Constants.INDEXER_MAX_UNJAMS_PER_BALL) {
            state = State.STUCK;
            return state;
        }
        unjamsThisBall++;
        unjams++;
        reverseSec = 0;
        pushSec = 0;
        // Ball already at the beam (it never left it): push straight away.
        state = beamBlocked ? State.PUSHING : State.REVERSING;
        return state;
    }

    /** Forget the pending ball (e.g. everything was reversed back out). */
    public void clearPendingBall() {
        ballPending = false;
        unjamsThisBall = 0;
        if (state != State.IDLE && state != State.HOLDING) state = State.IDLE;
    }

    public State getState() {
        return state;
    }

    public boolean isBallPending() {
        return ballPending;
    }

    /** The flywheel has recovered since the last shot (feeding is allowed). */
    public boolean isFlywheelReady() {
        return flywheelReady;
    }

    public int getShots() {
        return shots;
    }

    /** Total reverse/push attempts this OpMode. */
    public int getUnjams() {
        return unjams;
    }

    /** Reverse/push attempts on the current ball. */
    public int getUnjamsThisBall() {
        return unjamsThisBall;
    }
}
