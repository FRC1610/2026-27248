package org.firstinspires.ftc.teamcode;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;
import java.util.Map;

/**
 * Shoot-sequence logging for tuning. Call {@link #update} once per loop after
 * {@link LaunchSequence#update}. Writes two CSV files to /sdcard/FIRST/data/:
 *
 * <ul>
 *   <li><b>shoot_events_&lt;time&gt;.csv</b>: one row per step - BURST_START/BURST_END (shoot
 *       pressed/released), STATE changes, BEAM_BLOCKED/BEAM_CLEAR, SHOT, READY (flywheel
 *       recovered), NUDGE, UNJAM, PARAM (a Panels value changed). Starts with a PARAMS row
 *       holding every ShootTuning value.
 *   <li><b>shots_&lt;time&gt;.csv</b>: one row per detected shot with the timing of each phase:
 *       wait (ball ready, flywheel not), feed start -&gt; ball leaves beam, beam -&gt; shot,
 *       interval since the previous shot, flywheel RPM at feed, lowest RPM and recovery time
 *       after the shot, and time to load the next ball to the beam. Plus the tuning values in
 *       use, so runs with different settings can be compared.
 * </ul>
 *
 * <p>A "burst" is one press-and-hold of the shoot command. Burst time = shoot pressed until
 * the last shot of the burst.
 */
public class ShootLog {
    private BufferedWriter events;
    private BufferedWriter shotsOut;
    private String error = null;
    private double lastFlushSec = 0;

    private boolean wasShootActive = false;
    private LaunchSequence.State prevState = null;
    private boolean prevBeam = false;
    private boolean prevReady = false;
    private boolean prevNudging = false;
    private Map<String, Double> lastParams = ShootTuning.values();

    // Burst.
    private int burst = 0;
    private double burstStartSec = Double.NaN;
    private int burstShots = 0;
    private double burstLastShotSec = Double.NaN;
    private double burstMinRpm = Double.NaN;

    // Ball being fed (before its shot).
    private double waitStartSec = Double.NaN;
    private double feedStartSec = Double.NaN;
    private double clearSec = Double.NaN;
    private double rpmAtFeed = Double.NaN;
    private int nudgesAtStart = 0;
    private int unjamsAtStart = 0;

    // Shot already detected, still measuring its recovery / the next load.
    private boolean shotOpen = false;
    private int shotBall;
    private double shotSec;
    private double shotSinceBurstSec;
    private double shotIntervalSec;
    private double shotWaitSec;
    private double shotFeedToClearSec;
    private double shotClearToShotSec;
    private double shotFeedToShotSec;
    private double shotSetpoint;
    private double shotRpmAtFeed;
    private double shotRpmAtContact;
    private double shotMinRpm;
    private double shotReadySec;
    private double shotNextStagedSec;
    private int shotNudges;
    private int shotUnjams;
    private String shotParams;

    /**
     * RPM at contact = highest flywheel reading in this window (s) before the shot is detected,
     * i.e. just before the dip. Detection lags the contact by only a few loops.
     */
    private static final double CONTACT_WINDOW_SEC = 0.15;
    // Recent flywheel readings (ring buffer), for the RPM at contact.
    private final double[] histSec = new double[64];
    private final double[] histRpm = new double[64];
    private int histNext = 0;
    private int histCount = 0;

    // For Panels.
    private double lastContactRpm = Double.NaN;
    private double lastIntervalSec = Double.NaN;
    private double lastRecoverySec = Double.NaN;
    private double lastMinRpm = Double.NaN;
    private double lastFeedToShotSec = Double.NaN;
    private double lastLoadSec = Double.NaN;
    private String lastBurstSummary = "none yet";

    /** @param dir log directory; @param stamp time stamp for the file names */
    public ShootLog(File dir, String stamp) {
        try {
            //noinspection ResultOfMethodCallIgnored
            dir.mkdirs();
            events = new BufferedWriter(new FileWriter(
                    new File(dir, "shoot_events_" + stamp + ".csv")));
            events.write("t_s,event,burst,ball,state,detail,setpoint_rpm,actual_rpm,"
                    + "underspeed_rpm,uptake_rpm,intake_power,battery_v\n");
            shotsOut = new BufferedWriter(new FileWriter(new File(dir, "shots_" + stamp + ".csv")));
            shotsOut.write("burst,ball,shot_t_s,since_burst_start_s,interval_s,wait_s,"
                    + "feed_to_clear_s,clear_to_shot_s,feed_to_shot_s,setpoint_rpm,"
                    + "rpm_at_feed,rpm_at_contact,contact_underspeed_rpm,min_rpm,max_drop_rpm,"
                    + "recovery_s,load_next_s,nudges,unjams,"
                    + "feed_pct,ready_tol_rpm,feed_delay_s,load_delay_s,load_pct,"
                    + "pidf\n");
            event(0, "PARAMS", "", ShootTuning.snapshot(), 0, 0, 0, 0, 0, Double.NaN);
        } catch (IOException e) {
            error = e.getMessage();
            events = null;
            shotsOut = null;
        }
    }

    /**
     * Records one loop.
     *
     * @param setpointRpm commanded flywheel RPM (0 when off)
     */
    public void update(double nowSec, LaunchSequence seq, LaunchSequence.State state,
                       boolean shootActive, boolean beamBlocked, double setpointRpm,
                       double actualRpm, double uptakeRpm, double intakePower, double volts) {
        double underspeed = setpointRpm > 0 ? setpointRpm - actualRpm : 0;
        String st = String.valueOf(state);

        // Panels value changes.
        Map<String, Double> params = ShootTuning.values();
        StringBuilder changed = new StringBuilder();
        for (Map.Entry<String, Double> e : params.entrySet()) {
            Double old = lastParams.get(e.getKey());
            if (old == null || Math.abs(old - e.getValue()) > 1e-9) {
                if (changed.length() > 0) changed.append(';');
                changed.append(e.getKey()).append('=').append(ShootTuning.format(e.getValue()));
            }
        }
        lastParams = params;
        if (changed.length() > 0) {
            event(nowSec, "PARAM", st, changed.toString(), setpointRpm, actualRpm, underspeed,
                    uptakeRpm, intakePower, volts);
        }

        // Burst start / end.
        if (shootActive && !wasShootActive) {
            burst++;
            burstStartSec = nowSec;
            burstShots = 0;
            burstLastShotSec = Double.NaN;
            burstMinRpm = actualRpm;
            // Forget feed timing from before the burst (e.g. a ball that passed the beam while
            // loading), so the first ball's phases are measured from the burst only.
            resetBall();
            event(nowSec, "BURST_START", st, "", setpointRpm, actualRpm, underspeed,
                    uptakeRpm, intakePower, volts);
        }
        if (!shootActive && wasShootActive) {
            finishShotRow();
            double burstSec = Double.isNaN(burstLastShotSec) ? Double.NaN
                    : burstLastShotSec - burstStartSec;
            lastBurstSummary = String.format(Locale.US,
                    "#%d: %d shots in %.2f s (avg %.2f s/shot), min %.0f RPM",
                    burst, burstShots, burstSec,
                    burstShots > 0 ? burstSec / burstShots : Double.NaN, burstMinRpm);
            event(nowSec, "BURST_END", st, String.format(Locale.US,
                            "shots=%d;burst_s=%.3f;held_s=%.3f;min_rpm=%.0f",
                            burstShots, burstSec, nowSec - burstStartSec, burstMinRpm),
                    setpointRpm, actualRpm, underspeed, uptakeRpm, intakePower, volts);
            resetBall();
        }
        wasShootActive = shootActive;
        if (shootActive && !Double.isNaN(burstMinRpm)) {
            burstMinRpm = Math.min(burstMinRpm, actualRpm);
        }

        // State changes, and the feed timing of the current ball.
        if (state != prevState) {
            event(nowSec, "STATE", st, prevState + "->" + state, setpointRpm, actualRpm,
                    underspeed, uptakeRpm, intakePower, volts);
            if (state == LaunchSequence.State.WAITING && Double.isNaN(waitStartSec)
                    && Double.isNaN(feedStartSec)) {
                waitStartSec = nowSec;
            }
            if (state == LaunchSequence.State.FEEDING && Double.isNaN(feedStartSec)) {
                feedStartSec = nowSec;
                rpmAtFeed = actualRpm;
                nudgesAtStart = seq.getNudges();
                unjamsAtStart = seq.getUnjams();
            }
        }
        prevState = state;

        if (beamBlocked != prevBeam) {
            event(nowSec, beamBlocked ? "BEAM_BLOCKED" : "BEAM_CLEAR", st,
                    seq.isBallPending() ? "pending" : "", setpointRpm, actualRpm, underspeed,
                    uptakeRpm, intakePower, volts);
            // Next ball staged after a shot.
            if (beamBlocked && shotOpen && Double.isNaN(shotNextStagedSec)) {
                shotNextStagedSec = nowSec;
            }
        }
        prevBeam = beamBlocked;
        if (seq.didBallPassThisLoop() && Double.isNaN(clearSec)) clearSec = nowSec;

        boolean ready = seq.isFlywheelReady();
        if (ready && !prevReady) {
            event(nowSec, "READY", st, "", setpointRpm, actualRpm, underspeed, uptakeRpm,
                    intakePower, volts);
            if (shotOpen && Double.isNaN(shotReadySec)) shotReadySec = nowSec;
        }
        prevReady = ready;

        boolean nudging = seq.isIntakeNudging();
        if (nudging && !prevNudging) {
            event(nowSec, "NUDGE", st, "", setpointRpm, actualRpm, underspeed, uptakeRpm,
                    intakePower, volts);
        }
        prevNudging = nudging;

        if (seq.didPrestageStartThisLoop()) {
            event(nowSec, "PRESTAGE", st, "ball past beam at shoot start", setpointRpm,
                    actualRpm, underspeed, uptakeRpm, intakePower, volts);
        }
        if (seq.didUnjamStartThisLoop()) {
            event(nowSec, "UNJAM", st, "try " + seq.getUnjamsThisBall(), setpointRpm,
                    actualRpm, underspeed, uptakeRpm, intakePower, volts);
        }

        // Lowest RPM after the shot, until the flywheel is ready again.
        if (shotOpen && Double.isNaN(shotReadySec)) shotMinRpm = Math.min(shotMinRpm, actualRpm);

        if (seq.wasShotThisLoop()) {
            finishShotRow(); // previous shot's row, if still open
            shotRpmAtContact = contactRpm(nowSec);
            lastContactRpm = shotRpmAtContact;
            event(nowSec, "SHOT", st, String.format(Locale.US, "#%d;contact_rpm=%.0f",
                            seq.getShots(), shotRpmAtContact), setpointRpm, actualRpm, underspeed,
                    uptakeRpm, intakePower, volts);
            if (shootActive) burstShots++;
            shotOpen = true;
            shotBall = shootActive ? burstShots : 0; // 0 = shot outside a burst
            shotSec = nowSec;
            shotSinceBurstSec = shootActive ? nowSec - burstStartSec : Double.NaN;
            shotIntervalSec = !shootActive ? Double.NaN
                    : Double.isNaN(burstLastShotSec) ? nowSec - burstStartSec
                    : nowSec - burstLastShotSec;
            if (shootActive) burstLastShotSec = nowSec;
            shotWaitSec = Double.isNaN(waitStartSec) ? 0
                    : (Double.isNaN(feedStartSec) ? nowSec : feedStartSec) - waitStartSec;
            shotFeedToClearSec = clearSec - feedStartSec;
            shotClearToShotSec = nowSec - clearSec;
            shotFeedToShotSec = nowSec - feedStartSec;
            shotSetpoint = setpointRpm;
            shotRpmAtFeed = rpmAtFeed;
            shotMinRpm = actualRpm;
            shotReadySec = Double.NaN;
            shotNextStagedSec = beamBlocked ? nowSec : Double.NaN;
            shotNudges = Double.isNaN(feedStartSec) ? 0 : seq.getNudges() - nudgesAtStart;
            shotUnjams = Double.isNaN(feedStartSec) ? 0 : seq.getUnjams() - unjamsAtStart;
            shotParams = String.format(Locale.US, "%s,%s,%s,%s,%s,P%s I%s D%s F%s",
                    ShootTuning.format(ShootTuning.UPTAKE_FEED_PERCENT),
                    ShootTuning.format(ShootTuning.SHOOT_READY_TOLERANCE_RPM),
                    ShootTuning.format(ShootTuning.FEED_DELAY_AFTER_READY_SEC),
                    ShootTuning.format(ShootTuning.LOAD_DELAY_AFTER_SHOT_SEC),
                    ShootTuning.format(ShootTuning.UPTAKE_LOAD_PERCENT),
                    ShootTuning.format(ShootTuning.FLYWHEEL_P),
                    ShootTuning.format(ShootTuning.FLYWHEEL_I),
                    ShootTuning.format(ShootTuning.FLYWHEEL_D),
                    ShootTuning.format(ShootTuning.FLYWHEEL_F));
            lastIntervalSec = shotIntervalSec;
            lastFeedToShotSec = shotFeedToShotSec;
            resetBall();
        }
        // Close the row once recovery and the next load are both known.
        if (shotOpen && !Double.isNaN(shotReadySec) && !Double.isNaN(shotNextStagedSec)) {
            finishShotRow();
        }

        // Remember this reading (after the shot check, so the RPM at contact only uses earlier
        // loops).
        histSec[histNext] = nowSec;
        histRpm[histNext] = actualRpm;
        histNext = (histNext + 1) % histSec.length;
        histCount = Math.min(histCount + 1, histSec.length);

        if (nowSec - lastFlushSec > 1.0) {
            flush();
            lastFlushSec = nowSec;
        }
    }

    /** Highest flywheel RPM in the CONTACT_WINDOW_SEC before nowSec (NaN if no readings). */
    private double contactRpm(double nowSec) {
        double best = Double.NaN;
        for (int k = 1; k <= histCount; k++) {
            int i = (histNext - k + histSec.length) % histSec.length;
            if (nowSec - histSec[i] > CONTACT_WINDOW_SEC) break;
            if (Double.isNaN(best) || histRpm[i] > best) best = histRpm[i];
        }
        return best;
    }

    private void resetBall() {
        waitStartSec = Double.NaN;
        feedStartSec = Double.NaN;
        clearSec = Double.NaN;
        rpmAtFeed = Double.NaN;
    }

    private void finishShotRow() {
        if (!shotOpen) return;
        shotOpen = false;
        double recovery = shotReadySec - shotSec;
        double load = shotNextStagedSec - shotSec;
        lastRecoverySec = recovery;
        lastMinRpm = shotMinRpm;
        lastLoadSec = load;
        if (shotsOut == null) return;
        try {
            shotsOut.write(String.format(Locale.US,
                    "%d,%d,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.0f,%.1f,%.1f,%.1f,%.1f,%.1f,%.3f,"
                            + "%.3f,%d,%d,%s\n",
                    burst, shotBall, shotSec, shotSinceBurstSec, shotIntervalSec, shotWaitSec,
                    shotFeedToClearSec, shotClearToShotSec, shotFeedToShotSec, shotSetpoint,
                    shotRpmAtFeed, shotRpmAtContact, shotSetpoint - shotRpmAtContact, shotMinRpm, shotSetpoint - shotMinRpm, recovery, load,
                    shotNudges, shotUnjams, shotParams));
        } catch (IOException e) {
            error = e.getMessage();
            shotsOut = null;
        }
    }

    private void event(double t, String name, String state, String detail, double setpoint,
                       double actual, double underspeed, double uptakeRpm, double intakePower,
                       double volts) {
        if (events == null) return;
        try {
            events.write(String.format(Locale.US,
                    "%.4f,%s,%d,%d,%s,\"%s\",%.0f,%.1f,%.1f,%.1f,%.2f,%.2f\n",
                    t, name, burst, burstShots, state, detail, setpoint, actual, underspeed,
                    uptakeRpm, intakePower, volts));
        } catch (IOException e) {
            error = e.getMessage();
            events = null;
        }
    }

    public void flush() {
        try {
            if (events != null) events.flush();
            if (shotsOut != null) shotsOut.flush();
        } catch (IOException e) {
            error = e.getMessage();
        }
    }

    /** Writes any open shot row and closes both files. */
    public void close() {
        finishShotRow();
        try {
            if (events != null) events.close();
            if (shotsOut != null) shotsOut.close();
        } catch (IOException ignored) {
            // Nothing else to do at stop.
        }
        events = null;
        shotsOut = null;
    }

    /** Null if logging works, otherwise the error. */
    public String getError() {
        return error;
    }

    public int getBurstShots() {
        return burstShots;
    }

    /** Seconds since shoot was pressed (NaN when not shooting). */
    public double getBurstElapsedSec(double nowSec) {
        return wasShootActive ? nowSec - burstStartSec : Double.NaN;
    }

    /** Flywheel RPM just before the last shot's dip (the speed the ball met). */
    public double getLastContactRpm() {
        return lastContactRpm;
    }

    public double getLastIntervalSec() {
        return lastIntervalSec;
    }

    public double getLastRecoverySec() {
        return lastRecoverySec;
    }

    public double getLastMinRpm() {
        return lastMinRpm;
    }

    public double getLastFeedToShotSec() {
        return lastFeedToShotSec;
    }

    public double getLastLoadSec() {
        return lastLoadSec;
    }

    public String getLastBurstSummary() {
        return lastBurstSummary;
    }
}
