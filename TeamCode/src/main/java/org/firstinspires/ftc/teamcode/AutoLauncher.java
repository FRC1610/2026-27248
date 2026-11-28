package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;

/**
 * Reusable launch sequence for autonomous OpModes: aim the turret, then the same
 * {@link LaunchSequence} as Main 2026 (one ball at a time, wait for flywheel recovery, intake
 * nudge, stuck-ball recovery), driving the turret, flywheel, uptake and intake through
 * {@link RobotHardware}.
 *
 * <p>Non-blocking, so the OpMode keeps running its own loop (driving, telemetry, ...):
 * <pre>
 *   AutoLauncher launcher = new AutoLauncher(robot, shootLog);   // shootLog may be null
 *   launcher.start(4, ShootTuning.FLYWHEEL_RPM, 15.0);           // turret starts FORWARD
 *   launcher.start(4, ShootTuning.FLYWHEEL_RPM, 15.0, TurretDirection.REVERSE); // or REVERSE
 *   while (opModeIsActive() &amp;&amp; !launcher.isFinished()) {
 *       launcher.update(nowSec, dtSec, pinpointHeadingRad);   // NaN heading is fine
 *       ...
 *   }
 *   launcher.stop(false); // uptake and intake off; flywheel off (true = keep it spinning)
 * </pre>
 *
 * <p>Steps after {@link #start}:
 * <ol>
 *   <li><b>Swing:</b> the turret moves to the start direction (FORWARD unless given); vision is
 *       ignored for the estimated swing time (TURRET_SWING_SPEED_DEG_PER_SEC). The flywheel
 *       spins up meanwhile.
 *   <li><b>Aim:</b> turret tracking (Limelight tx + Pinpoint heading assist) runs until it locks
 *       on the target, or AUTO_AIM_TIMEOUT_SEC passes (then it launches from where it points).
 *       If the Limelight stops sending data, the turret returns to the start direction.
 *   <li><b>Launch:</b> tracking continues while the balls are fed.
 * </ol>
 *
 * <p>The launch finishes ({@link #isFinished()}) when:
 * <ul>
 *   <li>{@link Result#DONE}: the requested number of shots was detected;
 *   <li>{@link Result#OUT_OF_BALLS}: the sequence loaded (intake + uptake) for
 *       AUTO_NO_BALL_TIMEOUT_SEC without the beam seeing another ball, so the robot is empty
 *       (also right at the start if no ball is loaded);
 *   <li>{@link Result#STUCK}: a ball stayed stuck after all stuck-ball retries;
 *   <li>{@link Result#TIMEOUT}: the time limit given to {@link #start} passed.
 * </ul>
 * When it finishes, the uptake and intake stop; the turret stays where it is and the flywheel
 * keeps its setpoint until {@link #stop}. Spinning the flywheel up earlier (e.g. while driving)
 * with robot.setFlywheelRpm saves time.
 */
public class AutoLauncher {
    public enum Result {
        /** Not started, or {@link #stop} was called before it finished. */
        NOT_STARTED,
        RUNNING,
        DONE,
        OUT_OF_BALLS,
        STUCK,
        TIMEOUT
    }

    private final RobotHardware robot;
    /** Optional CSV logging of every loop (null = no log). */
    private final ShootLog shootLog;

    private LaunchSequence sequence;
    private TurretTracker tracker;
    private TurretDirection direction = TurretDirection.FORWARD;
    private Result result = Result.NOT_STARTED;
    private int maxShots;
    private double flywheelRpm;
    private double timeoutSec;
    private double elapsedSec;
    private double swingSec;
    private double aimSec;
    private boolean aimed;
    private double loadingSec;
    private LaunchSequence.State state = LaunchSequence.State.IDLE;
    private double intakePower = 0;
    private double actualRpm = 0;

    public AutoLauncher(RobotHardware robot, ShootLog shootLog) {
        this.robot = robot;
        this.shootLog = shootLog;
    }

    /** Starts a launch with the turret starting FORWARD. */
    public void start(int maxShots, double flywheelRpm, double timeoutSec) {
        start(maxShots, flywheelRpm, timeoutSec, TurretDirection.FORWARD);
    }

    /**
     * Starts a launch. Can be called again later for another volley.
     *
     * @param maxShots    shots to fire before finishing (finishes earlier if the robot is empty)
     * @param flywheelRpm flywheel setpoint while launching (e.g. ShootTuning.FLYWHEEL_RPM)
     * @param timeoutSec  longest time for the whole launch, swing and spin-up included
     * @param direction   turret starting direction (null = FORWARD)
     */
    public void start(int maxShots, double flywheelRpm, double timeoutSec,
                      TurretDirection direction) {
        this.maxShots = maxShots;
        this.flywheelRpm = flywheelRpm;
        this.timeoutSec = timeoutSec;
        this.direction = direction == null ? TurretDirection.FORWARD : direction;
        double startPosition = this.direction.servoPosition();
        // Time for the servo to get there from its last commanded position, before vision is used.
        double swingDeg = Math.abs(TurretTracker.positionToAngle(startPosition)
                - TurretTracker.positionToAngle(robot.getTurretPosition()));
        swingSec = Double.isFinite(swingDeg)
                ? swingDeg / Constants.TURRET_SWING_SPEED_DEG_PER_SEC : 0;
        tracker = new TurretTracker(startPosition, startPosition);
        robot.setTurretPosition(startPosition);
        sequence = new LaunchSequence(robot.isBallAtUptake());
        elapsedSec = 0;
        aimSec = 0;
        aimed = false;
        loadingSec = 0;
        intakePower = 0;
        state = LaunchSequence.State.IDLE;
        result = maxShots > 0 ? Result.RUNNING : Result.DONE;
    }

    /** {@link #update(double, double, double)} without a heading (no Pinpoint assist). */
    public Result update(double nowSec, double dtSec) {
        return update(nowSec, dtSec, Double.NaN);
    }

    /**
     * Runs one step; call every loop until {@link #isFinished()}.
     *
     * @param nowSec     OpMode time (s), for the log
     * @param dtSec      seconds since the previous loop
     * @param headingRad Pinpoint heading (rad, CCW+) for the turret's heading assist; NaN if
     *                   unavailable
     * @return the current result ({@link Result#RUNNING} until finished)
     */
    public Result update(double nowSec, double dtSec, double headingRad) {
        if (result != Result.RUNNING) return result;
        elapsedSec += dtSec;

        // ---- Turret: swing to the start direction, then track ----
        boolean swingDone = elapsedSec >= swingSec;
        if (!swingDone) {
            robot.setTurretPosition(direction.servoPosition());
        } else {
            robot.selectAllianceLimelightPipeline();
            robot.refreshLimelightResult();
            LLResult vision = robot.getLatestLimelightResult();
            if (!TurretTracker.isLimelightReceiving(vision)) {
                robot.setTurretPosition(tracker.limelightLost());
            } else {
                boolean targetVisible = vision.isValid()
                        && vision.getPipelineIndex() == robot.getSelectedVisionPipeline()
                        && vision.getStaleness() <= Constants.TURRET_MAX_VISION_AGE_MS;
                robot.setTurretPosition(tracker.update(headingRad, targetVisible,
                        targetVisible ? vision.getTx() : Double.NaN,
                        vision.getControlHubTimeStampNanos(), 0, dtSec));
            }
            if (!aimed) {
                aimSec += dtSec;
                aimed = tracker.getState() == TurretTracker.State.TRACKING
                        || aimSec > Constants.AUTO_AIM_TIMEOUT_SEC;
            }
        }

        // ---- Flywheel and launch sequence (feeding only once aimed) ----
        robot.setFlywheelRpm(flywheelRpm);
        actualRpm = robot.getFlywheelRpm();
        boolean beamBlocked = robot.isBallAtUptake();
        state = sequence.update(beamBlocked, aimed, false, flywheelRpm, actualRpm, dtSec);
        if (aimed) {
            intakePower = applyState(state);
        } else {
            stopFeed();
        }

        // Out of balls: loading with nothing reaching the beam (and nothing already inside).
        boolean waitingForBall = aimed && state == LaunchSequence.State.LOADING
                && !beamBlocked && !sequence.isBallPending();
        loadingSec = waitingForBall ? loadingSec + dtSec : 0;

        if (sequence.getShots() >= maxShots) {
            result = Result.DONE;
        } else if (state == LaunchSequence.State.STUCK) {
            result = Result.STUCK;
        } else if (loadingSec > Constants.AUTO_NO_BALL_TIMEOUT_SEC) {
            result = Result.OUT_OF_BALLS;
        } else if (elapsedSec > timeoutSec) {
            result = Result.TIMEOUT;
        }

        if (shootLog != null) {
            shootLog.update(nowSec, sequence, state, aimed, beamBlocked, flywheelRpm, actualRpm,
                    robot.getUptakeRpm(), intakePower, Double.NaN);
        }
        if (result != Result.RUNNING) stopFeed();
        return result;
    }

    /**
     * Stops the launch (if still running) and the uptake and intake. The turret stays where it
     * is.
     *
     * @param keepFlywheel true to leave the flywheel at the launch RPM (e.g. another volley
     *                     soon), false to turn it off
     */
    public void stop(boolean keepFlywheel) {
        if (result == Result.RUNNING) result = Result.NOT_STARTED;
        stopFeed();
        robot.setFlywheelRpm(keepFlywheel ? flywheelRpm : 0);
    }

    private void stopFeed() {
        robot.setUptakePower(0);
        robot.setIntakePower(0);
        intakePower = 0;
    }

    /**
     * Drives the uptake and intake for a LaunchSequence state, as Main 2026 does while shoot is
     * held with no driver intake input. Returns the intake power commanded.
     */
    private double applyState(LaunchSequence.State state) {
        double feedRpm = Constants.UPTAKE_MAX_RPM
                * Math.max(0, Math.min(100, ShootTuning.UPTAKE_FEED_PERCENT)) / 100.0;
        double loadRpm = Constants.UPTAKE_MAX_RPM
                * Math.max(0, Math.min(100, ShootTuning.UPTAKE_LOAD_PERCENT)) / 100.0;
        double nudge = sequence.isIntakeNudging() ? ShootTuning.NUDGE_INTAKE_POWER : 0;
        double intake = 0;
        switch (state) {
            case LOADING:
                robot.setUptakeLoadRpm(loadRpm);
                intake = ShootTuning.SHOOT_INTAKE_POWER;
                break;
            case FEEDING:
                robot.setUptakeRpm(feedRpm);
                intake = nudge;
                break;
            case REVERSING:
                robot.setUptakePower(ShootTuning.UNJAM_POWER);
                break;
            case PUSHING:
                robot.setUptakePower(ShootTuning.STUCK_PUSH_POWER);
                intake = nudge;
                break;
            default: // IDLE, HOLDING, WAITING, SETTLING, STUCK
                robot.setUptakePower(0);
                break;
        }
        robot.setIntakePower(intake);
        return intake;
    }

    /** True once the launch has finished (any result other than RUNNING after start). */
    public boolean isFinished() {
        return result != Result.RUNNING;
    }

    public Result getResult() {
        return result;
    }

    /** Shots detected in the current/last launch. */
    public int getShots() {
        return sequence == null ? 0 : sequence.getShots();
    }

    public int getMaxShots() {
        return maxShots;
    }

    /** LaunchSequence state from the last update (for telemetry). */
    public LaunchSequence.State getState() {
        return state;
    }

    /** Turret starting direction of the current/last launch. */
    public TurretDirection getDirection() {
        return direction;
    }

    /** "SWINGING", "AIMING" or the TurretTracker state (for telemetry). */
    public String getTurretStatus() {
        if (tracker == null) return "-";
        if (result == Result.RUNNING && elapsedSec < swingSec) return "SWINGING";
        if (result == Result.RUNNING && !aimed) return "AIMING (" + tracker.getState() + ")";
        return String.valueOf(tracker.getState());
    }

    /** Feeding has started (turret aimed or aim timed out). */
    public boolean isAimed() {
        return aimed;
    }

    /** The shoot command is active this loop (for the lights: police lights while launching). */
    public boolean isLaunching() {
        return result == Result.RUNNING && aimed;
    }

    public double getFlywheelSetpointRpm() {
        return flywheelRpm;
    }

    /** Flywheel RPM read in the last update. */
    public double getFlywheelActualRpm() {
        return actualRpm;
    }

    /** Short summary for telemetry, e.g. "DONE 4/4" or "OUT_OF_BALLS 2/4". */
    public String getSummary() {
        return result + " " + getShots() + "/" + maxShots;
    }
}
