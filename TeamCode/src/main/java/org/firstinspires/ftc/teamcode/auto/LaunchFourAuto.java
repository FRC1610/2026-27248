package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.AutoHandoff;
import org.firstinspires.ftc.teamcode.AutoLauncher;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.ShootLog;
import org.firstinspires.ftc.teamcode.ShootTuning;
import org.firstinspires.ftc.teamcode.TurretDirection;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Sample autonomous, no driving: turret tracking on, launch the 4 preloaded balls with
 * {@link AutoLauncher} (the same LaunchSequence as Main 2026: one ball at a time, waiting for
 * flywheel recovery, stuck-ball recovery), then lower the flower wedge.
 *
 * <ol>
 *   <li>INIT: wedge up, turret at TURRET_HOLD_POSITION, lights and Limelight pipeline follow
 *       allianceToggle. Keep the robot still (Pinpoint calibrates).
 *   <li>START: AutoLauncher starts with the turret FORWARD (its normal starting position):
 *       flywheel on at ShootTuning.FLYWHEEL_RPM, turret tracking on, and it waits for the turret
 *       to lock on the target (up to AUTO_AIM_TIMEOUT_SEC; if no target is seen it shoots from
 *       where the turret points).
 *   <li>Launch until LAUNCH_COUNT shots are detected, the robot is out of balls (no ball reaches
 *       the beam within AUTO_NO_BALL_TIMEOUT_SEC), the sequence gives up on a stuck ball, or
 *       LAUNCH_TIMEOUT_SEC passes.
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
    /** Longest time (s) for the launch phase (aim and spin-up included). */
    private static final double LAUNCH_TIMEOUT_SEC = 15.0;
    /** Time (s) after lowering the wedge for it and the turret to finish moving before ending. */
    private static final double END_DELAY_SEC = 1.0;

    private enum Phase { LAUNCH, DONE }

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
            AutoLauncher launcher = new AutoLauncher(robot, shootLog);
            double flywheelRpm = ShootTuning.FLYWHEEL_RPM;
            launcher.start(LAUNCH_COUNT, flywheelRpm, LAUNCH_TIMEOUT_SEC,
                    TurretDirection.FORWARD);
            Phase phase = Phase.LAUNCH;

            ElapsedTime timer = new ElapsedTime();
            double lastSec = 0;
            double doneSec = 0;
            while (opModeIsActive()) {
                double nowSec = timer.seconds();
                double dtSec = nowSec - lastSec;
                lastSec = nowSec;

                // Pinpoint heading: turret heading assist, and handed to TeleOp at the end.
                boolean pinpointOk = robot.updatePinpoint() && robot.isPinpointReady();
                double heading = pinpointOk
                        ? robot.pinpoint.getPosition().getHeading(AngleUnit.RADIANS)
                        : Double.NaN;
                if (Double.isFinite(heading)) lastHeadingRad = heading;

                // ---- Launch (AutoLauncher drives turret, flywheel, uptake and intake) ----
                if (phase == Phase.DONE) {
                    robot.setTurretPosition(Constants.TURRET_HOLD_POSITION);
                }
                if (phase == Phase.LAUNCH && launcher.update(nowSec, dtSec, heading)
                        != AutoLauncher.Result.RUNNING) {
                    launcher.stop(false); // flywheel off
                    robot.setFlowerWedgeUp(false);
                    phase = Phase.DONE;
                    doneSec = nowSec;
                } else if (phase == Phase.DONE && nowSec - doneSec > END_DELAY_SEC) {
                    break; // wedge and turret have moved: end the OpMode (finally hands off)
                }

                boolean flywheelOn = phase != Phase.DONE;
                double actualRpm = robot.getFlywheelRpm();
                robot.lights.update(robot.isRedAlliance(), launcher.isLaunching(),
                        robot.isBallAtUptake(), flywheelOn, flywheelRpm, actualRpm);
                telemetry.addData("Phase", phase);
                telemetry.addData("Launch", launcher.getSummary());
                telemetry.addData("Launch state", launcher.getState());
                telemetry.addData("Flywheel (RPM)", "%.0f / %.0f", actualRpm,
                        flywheelOn ? flywheelRpm : 0);
                telemetry.addData("Turret", launcher.getTurretStatus());
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
}
