package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.HeadingFusion;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.ShootLog;
import org.firstinspires.ftc.teamcode.LaunchSequence;
import org.firstinspires.ftc.teamcode.ShootTuning;
import org.firstinspires.ftc.teamcode.TurretTracker;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

@TeleOp(name = "Main 2026", group = "TeleOp")
public class MainTeleOp extends LinearOpMode {
    private final RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() {
        // Driver Station: short status only (telemetry). Panels (http://192.168.43.1:8001):
        // full detail and graphs (panels).
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
        BufferedWriter log = null;
        ShootLog shootLog = null;
        try {
            robot.init(hardwareMap);
            VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
            // Flower wedge starts UP, as in the test OpModes (moves when INIT is pressed).
            boolean wedgeUp = true;
            robot.setFlowerWedgePosition(Constants.FLOWER_WEDGE_UP);
            // Turret starts at the hold position (facing forward; moves when INIT is pressed).
            // While tracking is on (gamepad2 X), TurretTracker aims it; otherwise it holds the
            // hold position. Commands are clipped to TURRET_MIN/MAX_POSITION.
            robot.setTurretPosition(Constants.TURRET_HOLD_POSITION);
            TurretTracker turretTracker = new TurretTracker(Constants.TURRET_HOLD_POSITION);

            // INIT loop instead of waitForStart() so the lights, Limelight pipeline and the
            // telemetry follow allianceToggle and startPositionToggle until START.
            while (opModeInInit()) {
                robot.selectAllianceLimelightPipeline();
                robot.lights.showAlliance(robot.isRedAlliance());
                addInitTelemetry();
                telemetry.update();
                idle();
            }
            if (isStopRequested()) return;

            boolean leftStickPreviouslyPressed = false;
            boolean startPreviouslyPressed = false;
            double speed = Constants.DRIVE_FULL_SPEED;
            boolean xPreviouslyPressed = false;
            boolean bPreviouslyPressed = false;
            boolean aPreviouslyPressed = false;
            boolean turretTrackingOn = Constants.TURRET_TRACKING_ON_AT_START;
            boolean dpadUpPreviouslyPressed = false;
            boolean dpadDownPreviouslyPressed = false;
            boolean flywheelOn = false;
            // Flywheel RPM, flywheel PIDF, uptake speeds and all shoot-sequence timings live in
            // ShootTuning (editable in Panels > Configurables; d-pad changes them too).
            // Set when the flywheel is turned off; the shoot command (A) is ignored until A is
            // released.
            boolean shootCancelled = false;
            boolean yPreviouslyPressed = false;
            boolean dpadRightPreviouslyPressed = false;
            boolean dpadLeftPreviouslyPressed = false;
            LaunchSequence launchSequence = new LaunchSequence(robot.isBallAtUptake());
            // Flywheel PIDF currently on the motor (RobotHardware.init applied Constants), so a
            // Panels edit (or Panels values kept from an earlier run) is applied below.
            double appliedP = Constants.FLYWHEEL_PIDF_P;
            double appliedI = Constants.FLYWHEEL_PIDF_I;
            double appliedD = Constants.FLYWHEEL_PIDF_D;
            double appliedF = Constants.FLYWHEEL_PIDF_F;

            // CSV logs in /sdcard/FIRST/data/: main_<time>.csv (every loop), plus
            // shoot_events_<time>.csv and shots_<time>.csv (ShootLog: shoot-sequence steps and
            // per-shot timing).
            String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            shootLog = new ShootLog(AppUtil.ROBOT_DATA_DIR, stamp);
            String logPath;
            try {
                File dir = AppUtil.ROBOT_DATA_DIR;
                //noinspection ResultOfMethodCallIgnored
                dir.mkdirs();
                File file = new File(dir, "main_" + stamp + ".csv");
                log = new BufferedWriter(new FileWriter(file));
                log.write("t_s,loop_ms,beam_blocked,ball_pending,flywheel_on,setpoint_rpm,"
                        + "actual_rpm,flywheel_ready,shots,shoot_held,shoot_state,uptake_status,"
                        + "uptake_cmd,uptake_rpm,unjams,unjams_this_ball,intake_power,"
                        + "battery_v,heading_source,pinpoint_ok,pinpoint_deg,imu_deg,"
                        + "drive_heading_deg\n");
                logPath = file.getAbsolutePath();
            } catch (IOException e) {
                log = null;
                logPath = "LOG FAILED: " + e.getMessage();
            }
            double volts = battery.getVoltage();
            double lastFlushSec = 0;
            long loops = 0;
            // Heading: Pinpoint, with the Control Hub IMU as backup. Both were zeroed at INIT.
            HeadingFusion headingFusion = new HeadingFusion();
            headingFusion.reset(0, 0);

            ElapsedTime loopTimer = new ElapsedTime();
            double lastLoopSec = loopTimer.seconds();
            while (opModeIsActive()) {
                double nowSec = loopTimer.seconds();
                double dtSec = nowSec - lastLoopSec;
                lastLoopSec = nowSec;
                loops++;
                if (loops % 25 == 0) volts = battery.getVoltage();
                robot.selectAllianceLimelightPipeline();
                robot.refreshLimelightResult();
                boolean pinpointRead = robot.updatePinpoint();
                Pose2D pose = robot.pinpoint.getPosition();
                double imuYaw = robot.readImuYawRadians();

                if (gamepad1.left_stick_button && !leftStickPreviouslyPressed) {
                    speed = speed == Constants.DRIVE_FULL_SPEED
                            ? Constants.DRIVE_SLOW_SPEED : Constants.DRIVE_FULL_SPEED;
                }
                leftStickPreviouslyPressed = gamepad1.left_stick_button;

                // Heading reset (gamepad1 START), for when the heading is wrong: the driver lines
                // the robot up as at the match start (back against the wall, front facing the
                // drivers' left), then presses START. The heading becomes 0 (Pinpoint zeroed, IMU
                // backup re-aligned), so the start offset applies again exactly as at INIT. Works
                // on the IMU alone if the Pinpoint is down. X/Y are kept.
                if (gamepad1.start && !startPreviouslyPressed) {
                    robot.resetHeading();
                    headingFusion.reset(0, imuYaw);
                    turretTracker.resetHeadingReference(); // the jump is not a robot turn
                    pose = new Pose2D(DistanceUnit.INCH, pose.getX(DistanceUnit.INCH),
                            pose.getY(DistanceUnit.INCH), AngleUnit.RADIANS, 0);
                }
                startPreviouslyPressed = gamepad1.start;

                // Field-centric heading = (Pinpoint, or IMU backup) heading + start offset. If the
                // Pinpoint resets or glitches, HeadingFusion asks for its heading to be corrected.
                // With no heading at all, drive robot-centric instead of stopping.
                boolean pinpointOk = pinpointRead && robot.isPinpointReady();
                double pinpointHeading = pose.getHeading(AngleUnit.RADIANS);
                double fusedHeading = headingFusion.update(pinpointOk, pinpointHeading, imuYaw);
                double realignHeading = headingFusion.takeRealignRequest();
                if (!Double.isNaN(realignHeading)) robot.setPinpointHeading(realignHeading);
                HeadingFusion.Source headingSource = headingFusion.getSource();
                boolean headingValid = !Double.isNaN(fusedHeading);
                double heading = headingValid
                        ? fusedHeading + robot.getHeadingOffsetRadians() : Double.NaN;
                robot.fieldCentricDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y,
                        gamepad1.right_stick_x, headingValid ? heading : 0, speed);
                if (headingSource == HeadingFusion.Source.IMU) {
                    telemetry.addLine("!! Pinpoint lost - using IMU");
                } else if (headingSource == HeadingFusion.Source.NONE) {
                    telemetry.addLine("!! NO HEADING - robot-centric");
                }

                // Flywheel and turret tracking on/off (gamepad2): B toggles the flywheel, X toggles
                // tracking. A ball at the uptake beam does not turn either on. Pressing A (shoot)
                // turns the flywheel on if it is off; the sequence then waits for it to reach
                // speed. Turning the flywheel off (B) also stops the uptake: the shoot command is
                // cancelled until A is released (holding A does not turn it back on).
                boolean ballPresent = robot.isBallAtUptake();
                if (gamepad2.b && !bPreviouslyPressed) {
                    flywheelOn = !flywheelOn;
                    if (!flywheelOn) shootCancelled = true;
                }
                bPreviouslyPressed = gamepad2.b;
                if (gamepad2.a && !aPreviouslyPressed) flywheelOn = true;
                aPreviouslyPressed = gamepad2.a;
                if (gamepad2.x && !xPreviouslyPressed) turretTrackingOn = !turretTrackingOn;
                xPreviouslyPressed = gamepad2.x;
                boolean turretAiming = turretTrackingOn;

                // Flywheel setpoint (gamepad2): d-pad up/down adjusts ShootTuning.FLYWHEEL_RPM by
                // FLYWHEEL_RPM_STEP (also editable in Panels). Commanded before the uptake so the
                // uptake interlock sees this loop's flywheel command.
                if (gamepad2.dpad_up && !dpadUpPreviouslyPressed) {
                    ShootTuning.FLYWHEEL_RPM += Constants.FLYWHEEL_RPM_STEP;
                }
                dpadUpPreviouslyPressed = gamepad2.dpad_up;
                if (gamepad2.dpad_down && !dpadDownPreviouslyPressed) {
                    ShootTuning.FLYWHEEL_RPM -= Constants.FLYWHEEL_RPM_STEP;
                }
                dpadDownPreviouslyPressed = gamepad2.dpad_down;
                ShootTuning.FLYWHEEL_RPM = Math.max(0,
                        Math.min(Constants.FLYWHEEL_MAX_RPM, ShootTuning.FLYWHEEL_RPM));
                double flywheelSetpointRpm = ShootTuning.FLYWHEEL_RPM;
                // Flywheel PIDF: apply Panels edits as soon as they change.
                if (ShootTuning.FLYWHEEL_P != appliedP || ShootTuning.FLYWHEEL_I != appliedI
                        || ShootTuning.FLYWHEEL_D != appliedD
                        || ShootTuning.FLYWHEEL_F != appliedF) {
                    appliedP = ShootTuning.FLYWHEEL_P;
                    appliedI = ShootTuning.FLYWHEEL_I;
                    appliedD = ShootTuning.FLYWHEEL_D;
                    appliedF = ShootTuning.FLYWHEEL_F;
                    robot.setFlywheelPidf(appliedP, appliedI, appliedD, appliedF);
                }
                robot.setFlywheelRpm(flywheelOn ? flywheelSetpointRpm : 0);
                double flywheelRpm = robot.getFlywheelRpm();

                // Turret: while aiming, TurretTracker uses the turret-mounted Limelight's tx plus
                // Pinpoint heading changes (gamepad2 right stick X = manual). Only a valid, fresh
                // result from the selected alliance pipeline counts as a target. If the Limelight
                // stops sending data, the turret returns to the INIT (hold) position until it
                // comes back. When not aiming, the turret holds and the tracker starts fresh.
                LLResult vision = robot.getLatestLimelightResult();
                if (turretAiming && !TurretTracker.isLimelightReceiving(vision)) {
                    robot.setTurretPosition(turretTracker.limelightLost());
                } else if (turretAiming) {
                    boolean targetVisible = vision != null && vision.isValid()
                            && vision.getPipelineIndex() == robot.getSelectedVisionPipeline()
                            && vision.getStaleness() <= Constants.TURRET_MAX_VISION_AGE_MS;
                    double turretPosition = turretTracker.update(
                            headingValid ? heading : Double.NaN,
                            targetVisible, targetVisible ? vision.getTx() : Double.NaN,
                            vision.getControlHubTimeStampNanos(), // new frame detection
                            -gamepad2.right_stick_x, // stick right = turn clockwise (right)
                            dtSec);
                    robot.setTurretPosition(turretPosition);
                } else {
                    robot.setTurretPosition(Constants.TURRET_HOLD_POSITION);
                    turretTracker = new TurretTracker(Constants.TURRET_HOLD_POSITION);
                }

                // Emergency (gamepad2): hold RB = intake + uptake full forward, LB = full
                // reverse. Holding both cancels out and leaves the normal controls in charge.
                int emergencyDirection = (gamepad2.right_bumper ? 1 : 0)
                        - (gamepad2.left_bumper ? 1 : 0);
                boolean emergency = emergencyDirection != 0;

                // Shoot command (gamepad2): hold A. The press turns the flywheel on (above); after
                // the flywheel is turned off with B, A is ignored until it is released.
                if (!gamepad2.a) shootCancelled = false;
                boolean shootHeld = gamepad2.a && !shootCancelled;
                boolean shootActive = shootHeld && flywheelOn && !emergency;

                // Driver intake (gamepad2): right trigger forward, left trigger reverse.
                double driverIntakePower = gamepad2.right_trigger - gamepad2.left_trigger;
                if (Math.abs(driverIntakePower) < Constants.TRIGGER_DEADBAND) driverIntakePower = 0;

                // Uptake feed speed (gamepad2): d-pad right/left adjusts
                // ShootTuning.UPTAKE_FEED_PERCENT by 5% (also editable in Panels). Loading uses
                // the slower ShootTuning.UPTAKE_LOAD_PERCENT.
                if (gamepad2.dpad_right && !dpadRightPreviouslyPressed) {
                    ShootTuning.UPTAKE_FEED_PERCENT += Constants.UPTAKE_SPEED_STEP_PERCENT;
                }
                dpadRightPreviouslyPressed = gamepad2.dpad_right;
                if (gamepad2.dpad_left && !dpadLeftPreviouslyPressed) {
                    ShootTuning.UPTAKE_FEED_PERCENT -= Constants.UPTAKE_SPEED_STEP_PERCENT;
                }
                dpadLeftPreviouslyPressed = gamepad2.dpad_left;
                ShootTuning.UPTAKE_FEED_PERCENT =
                        Math.max(0, Math.min(100, ShootTuning.UPTAKE_FEED_PERCENT));
                double uptakeSpeedPercent = ShootTuning.UPTAKE_FEED_PERCENT;
                double uptakeTargetRpm = Constants.UPTAKE_MAX_RPM * uptakeSpeedPercent / 100.0;
                double uptakeLoadRpm = Constants.UPTAKE_MAX_RPM
                        * Math.max(0, Math.min(100, ShootTuning.UPTAKE_LOAD_PERCENT)) / 100.0;

                // Loading and shooting (LaunchSequence):
                // - RT intake runs the uptake too until a ball reaches the beam; the uptake then
                //   waits there while the intake keeps following RT.
                // - Holding A (flywheel on) feeds one ball at a time: wait for the flywheel to
                //   recover, feed with the intake stopped until the shot, then intake + uptake
                //   until the next ball reaches the beam, and repeat.
                // - A stuck ball (no shot) is reversed until it is back at the beam, then pushed
                //   at 100% until it shoots.
                LaunchSequence.State shootState = launchSequence.update(ballPresent, shootActive,
                        driverIntakePower > 0 && !emergency,
                        flywheelOn ? flywheelSetpointRpm : 0, flywheelRpm, dtSec);
                // Emergency reverse pushes balls back out; forget any ball in the uptake.
                if (emergency && emergencyDirection < 0) launchSequence.clearPendingBall();

                boolean uptakeForwardAllowed = robot.isUptakeForwardAllowed();
                double intakePower;
                double uptakeCmd; // logged: RPM when running closed loop, power otherwise
                String uptakeStatus;
                if (emergency) {
                    intakePower = emergencyDirection;
                    robot.setUptakePower(emergencyDirection);
                    uptakeCmd = emergencyDirection;
                    uptakeStatus = emergencyDirection > 0 ? "EMERGENCY FWD" : "EMERGENCY REV";
                } else {
                    switch (shootState) {
                        case LOADING:
                            // Staging up to the beam only, so the flywheel interlock is bypassed.
                            // Slow (ShootTuning.UPTAKE_LOAD_PERCENT) so the ball stops at the beam.
                            robot.setUptakeLoadRpm(uptakeLoadRpm);
                            uptakeCmd = uptakeLoadRpm;
                            intakePower = shootActive ? ShootTuning.SHOOT_INTAKE_POWER
                                    : driverIntakePower;
                            uptakeStatus = "LOADING";
                            break;
                        case FEEDING:
                            robot.setUptakeRpm(uptakeTargetRpm);
                            uptakeCmd = uptakeForwardAllowed ? uptakeTargetRpm : 0;
                            // Intake off, except short pulses that push a ball still sitting in
                            // the beam onto the uptake rollers.
                            intakePower = launchSequence.isIntakeNudging()
                                    ? ShootTuning.NUDGE_INTAKE_POWER : 0;
                            uptakeStatus = !uptakeForwardAllowed ? "FEEDING (BLOCKED)"
                                    : launchSequence.isIntakeNudging() ? "FEEDING (NUDGE)"
                                    : "FEEDING";
                            break;
                        case SETTLING:
                            robot.setUptakePower(0);
                            uptakeCmd = 0;
                            intakePower = 0;
                            uptakeStatus = "SETTLING AFTER SHOT";
                            break;
                        case REVERSING:
                            robot.setUptakePower(ShootTuning.UNJAM_POWER);
                            uptakeCmd = ShootTuning.UNJAM_POWER;
                            intakePower = 0;
                            uptakeStatus = launchSequence.isPrestaging()
                                    ? "PRE-STAGE - REVERSING TO BEAM" : "STUCK - REVERSING";
                            break;
                        case PUSHING:
                            robot.setUptakePower(ShootTuning.STUCK_PUSH_POWER);
                            uptakeCmd = uptakeForwardAllowed
                                    ? ShootTuning.STUCK_PUSH_POWER : 0;
                            intakePower = launchSequence.isIntakeNudging()
                                    ? ShootTuning.NUDGE_INTAKE_POWER : 0;
                            uptakeStatus = launchSequence.isIntakeNudging()
                                    ? "STUCK - PUSHING 100% (NUDGE)" : "STUCK - PUSHING 100%";
                            break;
                        case WAITING:
                            robot.setUptakePower(0);
                            uptakeCmd = 0;
                            intakePower = 0;
                            uptakeStatus = "WAITING FOR FLYWHEEL";
                            break;
                        case STUCK:
                            robot.setUptakePower(0);
                            uptakeCmd = 0;
                            intakePower = 0;
                            uptakeStatus = "STUCK - CLEAR MANUALLY";
                            break;
                        case HOLDING:
                        case IDLE:
                        default:
                            robot.setUptakePower(0);
                            uptakeCmd = 0;
                            intakePower = driverIntakePower;
                            uptakeStatus = shootState == LaunchSequence.State.HOLDING
                                    ? "BALL READY" : "OFF";
                            break;
                    }
                }
                robot.setIntakePower(intakePower);
                double uptakeRpm = robot.getUptakeRpm();
                shootLog.update(nowSec, launchSequence, shootState, shootActive, ballPresent,
                        flywheelOn ? flywheelSetpointRpm : 0, flywheelRpm, uptakeRpm,
                        intakePower, volts);

                // Flower wedge (gamepad2): Y toggles between the up and down presets.
                if (gamepad2.y && !yPreviouslyPressed) {
                    wedgeUp = !wedgeUp;
                    robot.setFlowerWedgePosition(
                            wedgeUp ? Constants.FLOWER_WEDGE_UP : Constants.FLOWER_WEDGE_DOWN);
                }
                yPreviouslyPressed = gamepad2.y;

                // ---- Log (flushed about once a second) ----
                if (log != null) {
                    try {
                        log.write(String.format(Locale.US,
                                "%.4f,%.2f,%d,%d,%d,%.0f,%.1f,%d,%d,%d,%s,%s,%.2f,%.1f,%d,%d,%.2f,"
                                        + "%.2f",
                                nowSec, dtSec * 1000.0, ballPresent ? 1 : 0,
                                launchSequence.isBallPending() ? 1 : 0, flywheelOn ? 1 : 0,
                                flywheelOn ? flywheelSetpointRpm : 0, flywheelRpm,
                                launchSequence.isFlywheelReady() ? 1 : 0,
                                launchSequence.getShots(), shootHeld ? 1 : 0, shootState,
                                uptakeStatus.replace(' ', '_'), uptakeCmd, uptakeRpm,
                                launchSequence.getUnjams(), launchSequence.getUnjamsThisBall(),
                                intakePower, volts)
                                + String.format(Locale.US, ",%s,%d,%.1f,%.1f,%.1f\n",
                                headingSource, pinpointOk ? 1 : 0,
                                Math.toDegrees(pinpointHeading), Math.toDegrees(imuYaw),
                                headingValid ? AngleUnit.normalizeDegrees(Math.toDegrees(heading))
                                        : Double.NaN));
                        if (nowSec - lastFlushSec > 1.0) {
                            log.flush();
                            lastFlushSec = nowSec;
                        }
                    } catch (IOException e) {
                        logPath = "LOG WRITE FAILED: " + e.getMessage();
                        log = null;
                    }
                }

                // Lights: police lights while the shoot command runs (A held, flywheel on);
                // otherwise alliance color, or purple/green snakes with a ball loaded.
                robot.lights.update(robot.isRedAlliance(), shootActive, ballPresent, flywheelOn,
                        flywheelSetpointRpm, flywheelRpm);
                double targetDistanceIn = RobotHardware.getTargetDistanceInches(vision,
                        robot.getSelectedVisionPipeline());

                // ---- Driver Station: alerts (only when they apply), then a short status ----
                if (shootState == LaunchSequence.State.STUCK) {
                    telemetry.addLine("!! BALL STUCK - release A or hold LB");
                }
                if (gamepad2.a && shootCancelled) {
                    telemetry.addLine("!! Shoot cancelled (flywheel off) - release A");
                }
                if (emergency) {
                    telemetry.addLine("!! EMERGENCY " + (emergencyDirection > 0 ? "FWD" : "REV"));
                }
                if (log == null || shootLog.getError() != null) {
                    telemetry.addLine("!! Log not recording");
                }
                String tuned = ShootTuning.changedFromDefaults();
                if (!tuned.isEmpty()) telemetry.addLine("!! Panels tuning active: " + tuned);
                telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
                telemetry.addData("Setpoint (RPM)", "%.0f", flywheelSetpointRpm);
                telemetry.addData("Actual (RPM)", "%.0f", flywheelRpm);
                telemetry.addData("Target dist (in)", Double.isNaN(targetDistanceIn)
                        ? "no target" : String.format(Locale.US, "%.1f", targetDistanceIn));
                telemetry.addData("Tracking", turretAiming ? "ON" : "OFF");
                telemetry.update();

                // ---- Panels: full detail; plain numbers so they can be graphed ----
                panels.addData("Flywheel on", flywheelOn ? 1.0 : 0.0);
                panels.addData("Flywheel setpoint (RPM)",
                        (double) Math.round(flywheelOn ? flywheelSetpointRpm : 0));
                panels.addData("Flywheel actual (RPM)", (double) Math.round(flywheelRpm));
                panels.addData("Flywheel selected (RPM)", (double) Math.round(flywheelSetpointRpm));
                PIDFCoefficients pidf = robot.getFlywheelPidf();
                panels.addData("Flywheel PIDF", String.format(Locale.US,
                        "P %.2f I %.2f D %.2f F %.2f", pidf.p, pidf.i, pidf.d, pidf.f));
                panels.addData("Target distance (in)", targetDistanceIn);
                panels.addData("Shoot active", shootActive ? 1.0 : 0.0);
                panels.addData("Shoot state", String.valueOf(shootState));
                panels.addData("Uptake status", uptakeStatus);
                panels.addData("Uptake setpoint (RPM)", (double) Math.round(
                        shootState == LaunchSequence.State.LOADING
                                || shootState == LaunchSequence.State.FEEDING
                                ? uptakeCmd : 0));
                panels.addData("Uptake actual (RPM)", (double) Math.round(uptakeRpm));
                panels.addData("Uptake speed (%)", uptakeSpeedPercent);
                panels.addData("Uptake forward allowed", uptakeForwardAllowed ? 1.0 : 0.0);
                panels.addData("Ball at beam", ballPresent ? 1.0 : 0.0);
                panels.addData("Ball in uptake, not shot",
                        launchSequence.isBallPending() ? 1.0 : 0.0);
                panels.addData("Shots", launchSequence.getShots());
                panels.addData("Unjams", launchSequence.getUnjams());
                panels.addData("Unjams on this ball", launchSequence.getUnjamsThisBall());
                panels.addData("Intake nudging", launchSequence.isIntakeNudging() ? 1.0 : 0.0);
                panels.addData("Intake nudges", launchSequence.getNudges());
                // Shot timing (ShootLog): graph these while tuning.
                panels.addData("Flywheel underspeed (RPM)",
                        flywheelOn ? flywheelSetpointRpm - flywheelRpm : 0.0);
                panels.addData("Flywheel ready", launchSequence.isFlywheelReady() ? 1.0 : 0.0);
                panels.addData("Burst shots", shootLog.getBurstShots());
                panels.addData("Burst elapsed (s)", shootLog.getBurstElapsedSec(nowSec));
                panels.addData("Last shot interval (s)", shootLog.getLastIntervalSec());
                panels.addData("Last feed-to-shot (s)", shootLog.getLastFeedToShotSec());
                panels.addData("Last recovery (s)", shootLog.getLastRecoverySec());
                panels.addData("Last load next (s)", shootLog.getLastLoadSec());
                panels.addData("Last RPM at contact", shootLog.getLastContactRpm());
                panels.addData("Last min RPM", shootLog.getLastMinRpm());
                panels.addData("Last burst", shootLog.getLastBurstSummary());
                panels.addData("Panels tuning changed", tuned.isEmpty() ? "none" : tuned);
                panels.addData("Intake power", intakePower);
                panels.addData("Lights", String.valueOf(robot.lights.getState()));
                panels.addData("Flower wedge", wedgeUp ? "UP" : "DOWN");
                panels.addData("Alliance", robot.isRedAlliance() ? "RED" : "BLUE");
                panels.addData("Start position", robot.getStartPositionName());
                panels.addData("Drive speed", speed);
                panels.addData("Heading source", String.valueOf(headingSource));
                panels.addData("Pinpoint status", pinpointRead
                        ? String.valueOf(robot.pinpoint.getDeviceStatus()) : "READ FAILED");
                panels.addData("Pinpoint heading (deg)", Math.toDegrees(pinpointHeading));
                panels.addData("IMU yaw (deg)", Math.toDegrees(imuYaw));
                panels.addData("IMU pitch (deg)", robot.getImuPitchDegrees());
                panels.addData("IMU roll (deg)", robot.getImuRollDegrees());
                panels.addData("X (in)", pose.getX(DistanceUnit.INCH));
                panels.addData("Y (in)", pose.getY(DistanceUnit.INCH));
                panels.addData("Heading drive (deg)", headingValid
                        ? AngleUnit.normalizeDegrees(Math.toDegrees(heading)) : Double.NaN);
                panels.addData("Turret", turretAiming
                        ? "TRACKING " + turretTracker.getState() : "OFF (hold)");
                panels.addData("Turret position", robot.getTurretPosition());
                panels.addData("Turret angle cmd (deg)", turretTracker.getCommandedAngleDeg());
                addPanelsVision(panels, vision);
                panels.addData("Battery (V)", volts);
                panels.addData("Log", logPath);
                panels.update();
                idle();
            }
        } finally {
            try {
                robot.stopDrive();
            } finally {
                try {
                    robot.stopMechanisms();
                } finally {
                    try {
                        robot.stopVision();
                    } finally {
                        if (shootLog != null) shootLog.close();
                        if (log != null) {
                            try {
                                log.close();
                            } catch (IOException ignored) {
                                // Nothing else to do at stop.
                            }
                        }
                    }
                }
            }
        }
    }

    /** INIT screen: the two toggle switches and a one-line setup reminder. */
    private void addInitTelemetry() {
        telemetry.addData("Alliance", robot.isRedAlliance() ? "RED" : "BLUE");
        telemetry.addData("Start position", robot.getStartPositionName());
        telemetry.addLine("Back to wall, front facing left. Keep still.");
        String tuned = ShootTuning.changedFromDefaults();
        if (!tuned.isEmpty()) telemetry.addLine("!! Panels tuning active: " + tuned);
    }

    /** Limelight details for Panels. */
    private void addPanelsVision(TelemetryManager panels, LLResult result) {
        panels.addData("Limelight pipeline (requested)", robot.getSelectedVisionPipeline());
        if (result == null) {
            panels.addData("Limelight", "no data");
            return;
        }
        panels.addData("Limelight pipeline (actual)", result.getPipelineIndex());
        panels.addData("Limelight age (ms)", result.getStaleness());
        boolean valid = result.isValid()
                && result.getPipelineIndex() == robot.getSelectedVisionPipeline();
        panels.addData("Limelight", valid ? "target" : "no target");
        if (valid) {
            panels.addData("Limelight tx (deg)", result.getTx());
            panels.addData("Limelight ty (deg)", result.getTy());
        }
    }
}
