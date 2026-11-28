package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.AutoHandoff;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.LaunchSequence;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.ShootLog;
import org.firstinspires.ftc.teamcode.ShootTuning;
import org.firstinspires.ftc.teamcode.TurretTracker;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Sample autonomous, no driving: turret tracking on, launch the 4 preloaded balls with the same
 * LaunchSequence as Main 2026 (one ball at a time, waiting for flywheel recovery, stuck-ball
 * recovery), then lower the flower wedge.
 *
 * <ol>
 *   <li>INIT: wedge up, turret at TURRET_HOLD_POSITION, lights and Limelight pipeline follow
 *       allianceToggle. Keep the robot still (Pinpoint calibrates).
 *   <li>START: flywheel on at ShootTuning.FLYWHEEL_RPM, turret tracking on.
 *   <li>Wait for the turret to lock on the target (up to AIM_TIMEOUT_SEC; if no target is seen
 *       it shoots from the hold position anyway).
 *   <li>Launch until LAUNCH_COUNT shots are detected, the sequence gives up on a stuck ball,
 *       or LAUNCH_TIMEOUT_SEC passes.
 *   <li>Flywheel, uptake and intake off, turret back to hold, flower wedge DOWN. After
 *       END_DELAY_SEC the OpMode ends by itself, handing the heading, turret position and
 *       wedge up/down state to Main 2026 (AutoHandoff).
 * </ol>
 *
 * Writes shoot_events_auto_&lt;time&gt;.csv and shots_auto_&lt;time&gt;.csv (ShootLog).
 */
@Autonomous(name = "Launch 4 (no drive)", group = "Auto", preselectTeleOp = "Main 2026")
public class LaunchFourAuto extends LinearOpMode {
    private static final int LAUNCH_COUNT = 4;
    /** Longest wait (s) after START for the turret to lock on before launching anyway. */
    private static final double AIM_TIMEOUT_SEC = 1.5;
    /** Longest time (s) for the launch phase (spin-up included). */
    private static final double LAUNCH_TIMEOUT_SEC = 15.0;
    /** Time (s) after lowering the wedge for it and the turret to finish moving before ending. */
    private static final double END_DELAY_SEC = 1.0;

    private enum Phase { AIM, LAUNCH, DONE }

    private final RobotHardware robot = new RobotHardware();
    /** Last good Pinpoint heading (rad), handed to TeleOp at the end (AutoHandoff). */
    private double lastHeadingRad = Double.NaN;
    private boolean started = false;

    @Override
    public void runOpMode() {
        ShootLog shootLog = null;
        try {
            robot.init(hardwareMap);
            robot.setFlowerWedgePosition(Constants.FLOWER_WEDGE_UP);
            robot.setTurretPosition(Constants.TURRET_HOLD_POSITION);
            robot.setFlywheelPidf(ShootTuning.FLYWHEEL_P, ShootTuning.FLYWHEEL_I,
                    ShootTuning.FLYWHEEL_D, ShootTuning.FLYWHEEL_F);

            while (opModeInInit()) {
                robot.selectAllianceLimelightPipeline();
                robot.lights.showAlliance(robot.isRedAlliance());
                telemetry.addData("Alliance", robot.isRedAlliance() ? "RED" : "BLUE");
                telemetry.addLine("Keep still. START: track, launch " + LAUNCH_COUNT
                        + ", lower wedge.");
                telemetry.update();
                idle();
            }
            if (isStopRequested()) return;
            started = true;

            String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            shootLog = new ShootLog(AppUtil.ROBOT_DATA_DIR, "auto_" + stamp);
            LaunchSequence launch = new LaunchSequence(robot.isBallAtUptake());
            TurretTracker tracker = new TurretTracker(Constants.TURRET_HOLD_POSITION);
            double flywheelRpm = ShootTuning.FLYWHEEL_RPM;
            Phase phase = Phase.AIM;
            String result = "";

            ElapsedTime timer = new ElapsedTime();
            double lastSec = 0;
            double launchStartSec = 0;
            double doneSec = 0;
            while (opModeIsActive()) {
                double nowSec = timer.seconds();
                double dtSec = nowSec - lastSec;
                lastSec = nowSec;

                // ---- Turret tracking (Limelight tx + Pinpoint heading assist) ----
                robot.selectAllianceLimelightPipeline();
                robot.refreshLimelightResult();
                boolean pinpointOk = robot.updatePinpoint() && robot.isPinpointReady();
                double heading = pinpointOk
                        ? robot.pinpoint.getPosition().getHeading(AngleUnit.RADIANS)
                        : Double.NaN;
                if (Double.isFinite(heading)) lastHeadingRad = heading;
                if (phase == Phase.DONE) {
                    robot.setTurretPosition(Constants.TURRET_HOLD_POSITION);
                } else {
                    LLResult vision = robot.getLatestLimelightResult();
                    if (!TurretTracker.isLimelightReceiving(vision)) {
                        robot.setTurretPosition(tracker.limelightLost());
                    } else {
                        boolean targetVisible = vision.isValid()
                                && vision.getPipelineIndex() == robot.getSelectedVisionPipeline()
                                && vision.getStaleness() <= Constants.TURRET_MAX_VISION_AGE_MS;
                        robot.setTurretPosition(tracker.update(heading, targetVisible,
                                targetVisible ? vision.getTx() : Double.NaN,
                                vision.getControlHubTimeStampNanos(), 0, dtSec));
                    }
                }

                // ---- Flywheel ----
                boolean flywheelOn = phase != Phase.DONE;
                robot.setFlywheelRpm(flywheelOn ? flywheelRpm : 0);
                double actualRpm = robot.getFlywheelRpm();

                // ---- Phase changes ----
                if (phase == Phase.AIM && (tracker.getState() == TurretTracker.State.TRACKING
                        || nowSec > AIM_TIMEOUT_SEC)) {
                    phase = Phase.LAUNCH;
                    launchStartSec = nowSec;
                }

                // ---- Launch sequence (same states and commands as Main 2026) ----
                boolean shootActive = phase == Phase.LAUNCH;
                boolean ballPresent = robot.isBallAtUptake();
                LaunchSequence.State state = launch.update(ballPresent, shootActive, false,
                        flywheelOn ? flywheelRpm : 0, actualRpm, dtSec);
                double intakePower = applyLaunchState(launch, state, shootActive);
                shootLog.update(nowSec, launch, state, shootActive, ballPresent,
                        flywheelOn ? flywheelRpm : 0, actualRpm, robot.getUptakeRpm(),
                        intakePower, Double.NaN);

                if (phase == Phase.LAUNCH) {
                    if (launch.getShots() >= LAUNCH_COUNT) {
                        result = "launched " + LAUNCH_COUNT;
                    } else if (state == LaunchSequence.State.STUCK) {
                        result = "STUCK after " + launch.getShots() + " shots";
                    } else if (nowSec - launchStartSec > LAUNCH_TIMEOUT_SEC) {
                        result = "TIMEOUT after " + launch.getShots() + " shots";
                    }
                    if (!result.isEmpty()) {
                        phase = Phase.DONE;
                        doneSec = nowSec;
                        robot.setUptakePower(0);
                        robot.setIntakePower(0);
                        robot.setFlowerWedgeUp(false);
                    }
                } else if (phase == Phase.DONE && nowSec - doneSec > END_DELAY_SEC) {
                    break; // wedge and turret have moved: end the OpMode (finally hands off)
                }

                robot.lights.update(robot.isRedAlliance(), shootActive, ballPresent, flywheelOn,
                        flywheelRpm, actualRpm);
                telemetry.addData("Phase", phase);
                telemetry.addData("Shots", "%d / %d", launch.getShots(), LAUNCH_COUNT);
                telemetry.addData("Launch state", state);
                telemetry.addData("Flywheel (RPM)", "%.0f / %.0f", actualRpm,
                        flywheelOn ? flywheelRpm : 0);
                telemetry.addData("Turret", tracker.getState());
                if (!result.isEmpty()) telemetry.addData("Result", result);
                telemetry.update();
                idle();
            }
        } finally {
            // Hand the heading and servo positions to TeleOp (Main 2026 INIT), before
            // stopMechanisms() relaxes the servos.
            if (started) {
                AutoHandoff.save(lastHeadingRad, robot.getTurretPosition(),
                        robot.isFlowerWedgeUp());
            }
            try {
                robot.stopMechanisms();
            } finally {
                try {
                    robot.stopVision();
                } finally {
                    if (shootLog != null) shootLog.close();
                }
            }
        }
    }

    /**
     * Drives the uptake and intake for a LaunchSequence state (as Main 2026 does with no driver
     * intake input). Returns the intake power commanded.
     */
    private double applyLaunchState(LaunchSequence launch, LaunchSequence.State state,
                                    boolean shootActive) {
        double feedRpm = Constants.UPTAKE_MAX_RPM
                * Math.max(0, Math.min(100, ShootTuning.UPTAKE_FEED_PERCENT)) / 100.0;
        double loadRpm = Constants.UPTAKE_MAX_RPM
                * Math.max(0, Math.min(100, ShootTuning.UPTAKE_LOAD_PERCENT)) / 100.0;
        double nudge = launch.isIntakeNudging() ? ShootTuning.NUDGE_INTAKE_POWER : 0;
        double intakePower = 0;
        switch (state) {
            case LOADING:
                robot.setUptakeLoadRpm(loadRpm);
                intakePower = shootActive ? ShootTuning.SHOOT_INTAKE_POWER : 0;
                break;
            case FEEDING:
                robot.setUptakeRpm(feedRpm);
                intakePower = nudge;
                break;
            case REVERSING:
                robot.setUptakePower(ShootTuning.UNJAM_POWER);
                break;
            case PUSHING:
                robot.setUptakePower(ShootTuning.STUCK_PUSH_POWER);
                intakePower = nudge;
                break;
            default: // IDLE, HOLDING, WAITING, SETTLING, STUCK
                robot.setUptakePower(0);
                break;
        }
        robot.setIntakePower(intakePower);
        return intakePower;
    }
}
