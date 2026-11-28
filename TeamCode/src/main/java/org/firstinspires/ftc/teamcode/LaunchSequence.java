package org.firstinspires.ftc.teamcode;

/**
 * Launch sequence state machine: ball loading and the hold-to-shoot sequence for Main 2026. Pure logic, no hardware: call
 * {@link #update} once per loop and drive the intake/uptake from the returned {@link State}.
 * All timings and thresholds are read live from {@link ShootTuning} (editable in Panels).
 *
 * <p>indexBeam1 is at the uptake entrance. A ball "staged" at the beam is blocking it; a ball
 * "pending" has moved past the beam into the uptake (blocked -> clear) and has not been shot.
 *
 * <ul>
 *   <li><b>Loading (shoot not held):</b> while the intake runs forward, the uptake also runs
 *       until a ball is staged at the beam (or one is already pending). The uptake then stops;
 *       the intake keeps following the driver.
 *   <li><b>Shooting (shoot held, flywheel on):</b> one ball at a time. With a ball staged or
 *       pending, wait until the flywheel is ready (back within SHOOT_READY_TOLERANCE_RPM below
 *       its setpoint after the previous shot) plus FEED_DELAY_AFTER_READY_SEC, then run the
 *       uptake with the intake stopped until the shot is detected. After a shot, pause
 *       LOAD_DELAY_AFTER_SHOT_SEC ({@link State#SETTLING}); then, with no ball, run the intake and
 *       uptake until the next ball is staged. Repeat.
 *   <li><b>Intake nudge:</b> the beam can see a ball that isn't touching the uptake rollers
 *       yet. While FEEDING or PUSHING with the ball still in the beam (not yet passed), the
 *       intake is pulsed: NUDGE_DELAY_SEC off, NUDGE_PULSE_SEC on, repeating until the ball
 *       leaves the beam ({@link #isIntakeNudging()}).
 *   <li><b>Shot:</b> the flywheel falling more than SHOT_DROP_RPM below its setpoint. Detection
 *       (re-)arms once the flywheel is back within SHOT_REARM_RPM, so spin-up does not count.
 *   <li><b>Pre-stage:</b> if shoot starts with a ball already past the beam (it coasted into
 *       the uptake while loading), the uptake first reverses it back to the beam
 *       (REVERSING, {@link #isPrestaging()}) so it gets a running start, then it is fed normally.
 *       PRESTAGE_PENDING_BALL turns this off.
 *   <li><b>Stuck ball:</b> feeding for STUCK_TIMEOUT_SEC (from the start of feeding, or from
 *       the ball passing the beam) with no shot. The uptake reverses at UNJAM_POWER until the
 *       ball is seen at the beam again (at most STUCK_REVERSE_MAX_SEC), then pushes at
 *       STUCK_PUSH_POWER until the shot. If no shot within STUCK_PUSH_MAX_SEC it reverses and
 *       pushes again, up to MAX_UNJAMS_PER_BALL times, then stops ({@link State#STUCK}) until
 *       shoot is released.
 * </ul>
 */
public class LaunchSequence {
    /** What the intake and uptake should do this loop. */
    public enum State {
        /** No ball staged or pending and not loading: uptake off, intake follows the driver. */
        IDLE,
        /** Uptake forward (loading speed) until a ball is staged. Intake forward while shooting. */
        LOADING,
        /** Shoot not held, ball staged/pending: uptake off, intake follows the driver. */
        HOLDING,
        /** Shoot held, ball staged/pending, flywheel not ready yet: uptake and intake off. */
        WAITING,
        /** Uptake forward (feed speed) until the shot; intake off (except nudges). */
        FEEDING,
        /** Shoot held, just shot: pause LOAD_DELAY_AFTER_SHOT_SEC before loading. All off. */
        SETTLING,
        /**
         * Stuck ball or pre-stage: uptake reverse until the ball is back at the beam; intake off.
         */
        REVERSING,
        /** Stuck ball: uptake forward at STUCK_PUSH_POWER until the shot; intake off. */
        PUSHING,
        /** Retries used up: uptake and intake off until shoot is released. */
        STUCK
    }

    private State state = State.IDLE;
    private boolean beamWasBlocked;
    private boolean ballPending = false;
    private boolean shotArmed = false;
    private boolean flywheelReady = false;
    private double readyHeldSec = 0;
    private double sinceShotSec = Double.POSITIVE_INFINITY;
    private double feedSec = 0;
    private double reverseSec = 0;
    private double pushSec = 0;
    private int unjamsThisBall = 0;
    private int shots = 0;
    private int unjams = 0;
    private double nudgeSec = 0;
    private boolean intakeNudging = false;
    private int nudges = 0;
    private boolean wasShootActive = false;
    /** REVERSING is a pre-stage (running start for a ball already past the beam), not an unjam. */
    private boolean prestaging = false;
    private int prestages = 0;
    // One-loop events, for logging.
    private boolean prestageThisLoop = false;
    private boolean shotThisLoop = false;
    private boolean ballPassedThisLoop = false;
    private boolean unjamThisLoop = false;

    /** @param beamBlockedAtStart a ball in the beam at START is staged, not yet pending */
    public LaunchSequence(boolean beamBlockedAtStart) {
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
        shotThisLoop = false;
        prestageThisLoop = false;
        ballPassedThisLoop = false;
        unjamThisLoop = false;
        State next = step(beamBlocked, shootActive, intakeForward, setpointRpm, actualRpm, dtSec);
        updateNudge(beamBlocked, dtSec);
        return next;
    }

    /**
     * Intake nudge while the uptake feeds a ball that is still in the beam: off for
     * NUDGE_DELAY_SEC (give the rollers a chance), then on for NUDGE_PULSE_SEC, repeat.
     */
    private void updateNudge(boolean beamBlocked, double dtSec) {
        boolean ballNotOnRollers = (state == State.FEEDING || state == State.PUSHING)
                && beamBlocked && !ballPending;
        double cycle = ShootTuning.NUDGE_DELAY_SEC + ShootTuning.NUDGE_PULSE_SEC;
        if (!ballNotOnRollers || cycle <= 0) {
            nudgeSec = 0;
            intakeNudging = false;
            return;
        }
        nudgeSec += dtSec;
        boolean nudgeNow = nudgeSec % cycle >= ShootTuning.NUDGE_DELAY_SEC;
        if (nudgeNow && !intakeNudging) nudges++;
        intakeNudging = nudgeNow;
    }

    private State step(boolean beamBlocked, boolean shootActive, boolean intakeForward,
                       double setpointRpm, double actualRpm, double dtSec) {
        // A ball moved past the beam into the uptake.
        if (beamWasBlocked && !beamBlocked && !ballPending) {
            ballPending = true;
            ballPassedThisLoop = true;
            feedSec = 0; // stuck timer now measures the trip from the beam to the flywheel
        }
        beamWasBlocked = beamBlocked;

        // Shot detection and the "ready to feed" latch, only once the flywheel has reached speed.
        sinceShotSec += dtSec;
        if (setpointRpm > 0) {
            double underspeed = setpointRpm - actualRpm;
            if (shotArmed && underspeed > ShootTuning.SHOT_DROP_RPM) {
                shotThisLoop = true;
                shots++;
                shotArmed = false;
                flywheelReady = false;
                ballPending = false;
                sinceShotSec = 0;
            } else if (!shotArmed && underspeed < ShootTuning.SHOT_REARM_RPM) {
                shotArmed = true;
            }
            if (shotArmed && underspeed <= ShootTuning.SHOOT_READY_TOLERANCE_RPM) {
                flywheelReady = true; // latched until the next shot (ignores speed-reading noise)
            }
        } else {
            shotArmed = false;
            flywheelReady = false;
        }
        readyHeldSec = flywheelReady ? readyHeldSec + dtSec : 0;
        if (shotThisLoop) {
            unjamsThisBall = 0;
            prestaging = false;
            state = State.IDLE; // pick the next step below
        }
        boolean shootStarted = shootActive && !wasShootActive;
        wasShootActive = shootActive;

        if (!shootActive) {
            // Releasing shoot after giving up: start fresh next time.
            if (state == State.STUCK) ballPending = false;
            unjamsThisBall = 0;
            prestaging = false;
            if (intakeForward && !beamBlocked && !ballPending) {
                state = State.LOADING;
            } else {
                state = beamBlocked || ballPending ? State.HOLDING : State.IDLE;
            }
            return state;
        }

        // ---- Shoot held ----
        // Pre-stage: shoot started with a ball already inside the uptake (past the beam, from
        // rest). Reverse it back to the beam first so it gets a running start.
        if (shootStarted && ShootTuning.PRESTAGE_PENDING_BALL && ballPending && !beamBlocked) {
            prestaging = true;
            prestages++;
            prestageThisLoop = true;
            reverseSec = 0;
            state = State.REVERSING;
            return state;
        }
        if (state == State.STUCK) return state;
        if (state == State.REVERSING) {
            reverseSec += dtSec;
            if (!beamBlocked && reverseSec < ShootTuning.STUCK_REVERSE_MAX_SEC) return state;
            if (prestaging) {
                // Back at the beam: it is now a staged ball and is fed normally below (if the
                // reverse timed out without seeing it, it stays pending and is fed from there).
                prestaging = false;
                if (beamBlocked) ballPending = false;
                state = State.WAITING;
            } else {
                state = State.PUSHING; // stuck ball back at the beam (or reverse timed out)
                pushSec = 0;
                return state;
            }
        }
        if (state == State.PUSHING) {
            pushSec += dtSec;
            if (pushSec < ShootTuning.STUCK_PUSH_MAX_SEC) return state;
            return startUnjam(beamBlocked);
        }

        if (!ballPending && !beamBlocked) {
            // Just shot: optional pause before pulling the next ball in.
            state = sinceShotSec < ShootTuning.LOAD_DELAY_AFTER_SHOT_SEC
                    ? State.SETTLING : State.LOADING;
            return state;
        }
        if (state != State.FEEDING
                && (!flywheelReady || readyHeldSec < ShootTuning.FEED_DELAY_AFTER_READY_SEC)) {
            state = State.WAITING;
            return state;
        }
        if (state != State.FEEDING) {
            state = State.FEEDING;
            feedSec = 0;
        }
        feedSec += dtSec;
        if (ShootTuning.STUCK_RECOVERY_ENABLED && feedSec > ShootTuning.STUCK_TIMEOUT_SEC) {
            return startUnjam(beamBlocked);
        }
        return state;
    }

    private State startUnjam(boolean beamBlocked) {
        if (!ShootTuning.STUCK_RECOVERY_ENABLED) {
            state = State.FEEDING;
            return state;
        }
        if (unjamsThisBall >= ShootTuning.MAX_UNJAMS_PER_BALL) {
            state = State.STUCK;
            return state;
        }
        unjamsThisBall++;
        unjams++;
        unjamThisLoop = true;
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
        prestaging = false;
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

    /** Shot detection is armed (the flywheel has been back near its setpoint since the last shot). */
    public boolean isShotArmed() {
        return shotArmed;
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

    /** True while the intake should pulse forward to push a staged ball onto the rollers. */
    public boolean isIntakeNudging() {
        return intakeNudging;
    }

    /** Total intake nudge pulses this OpMode. */
    public int getNudges() {
        return nudges;
    }

    /** REVERSING is currently a pre-stage (running start), not a stuck-ball unjam. */
    public boolean isPrestaging() {
        return prestaging;
    }

    /** Total pre-stages this OpMode. */
    public int getPrestages() {
        return prestages;
    }

    /** A pre-stage started in the last {@link #update}. */
    public boolean didPrestageStartThisLoop() {
        return prestageThisLoop;
    }

    /** A shot was detected in the last {@link #update}. */
    public boolean wasShotThisLoop() {
        return shotThisLoop;
    }

    /** A ball passed the beam into the uptake in the last {@link #update}. */
    public boolean didBallPassThisLoop() {
        return ballPassedThisLoop;
    }

    /** A stuck-ball recovery (reverse/push) started in the last {@link #update}. */
    public boolean didUnjamStartThisLoop() {
        return unjamThisLoop;
    }
}
