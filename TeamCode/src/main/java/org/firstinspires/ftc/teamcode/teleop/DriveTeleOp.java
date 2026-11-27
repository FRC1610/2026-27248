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
import org.firstinspires.ftc.teamcode.ShootSequence;
import org.firstinspires.ftc.teamcode.TurretTracker;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

@TeleOp(name = "Main 2026", group = "TeleOp")
public class DriveTeleOp extends LinearOpMode {
    private final RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() {
        // Driver Station: short status only (telemetry). Panels (http://192.168.43.1:8001):
        // full detail and graphs (panels).
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
        BufferedWriter log = null;
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
            boolean turretTrackingOn = Constants.TURRET_TRACKING_ON_AT_START;
            boolean dpadUpPreviouslyPressed = false;
            boolean dpadDownPreviouslyPressed = false;
            boolean flywheelOn = false;
            double flywheelSetpointRpm = Constants.FLYWHEEL_DEFAULT_RPM;
            // Set when the flywheel is turned off; the shoot command (A) is ignored until A is
            // released.
            boolean shootCancelled = false;
            boolean yPreviouslyPressed = false;
            boolean dpadRightPreviouslyPressed = false;
            boolean dpadLeftPreviouslyPressed = false;
            double indexerSpeedPercent = Constants.INDEXER_DEFAULT_SPEED_PERCENT;
            ShootSequence shootSequence = new ShootSequence(
                    Constants.INDEXER_STUCK_RECOVERY_ENABLED, robot.isBallAtIndexer());

            // CSV log of the shooter/indexer every loop: /sdcard/FIRST/data/main_<time>.csv
            String logPath;
            try {
                File dir = AppUtil.ROBOT_DATA_DIR;
                //noinspection ResultOfMethodCallIgnored
                dir.mkdirs();
                File file = new File(dir, "main_"
                        + new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date())
                        + ".csv");
                log = new BufferedWriter(new FileWriter(file));
                log.write("t_s,loop_ms,beam_blocked,ball_pending,flywheel_on,setpoint_rpm,"
                        + "actual_rpm,flywheel_ready,shots,shoot_held,shoot_state,indexer_status,"
                        + "indexer_cmd,indexer_rpm,unjams,unjams_this_ball,intake_power,"
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
                // tracking. A ball at the indexer beam does not turn either on. Turning the
                // flywheel off also stops the indexer: the shoot command is cancelled until A is
                // released.
                boolean ballPresent = robot.isBallAtIndexer();
                if (gamepad2.b && !bPreviouslyPressed) {
                    flywheelOn = !flywheelOn;
                    if (!flywheelOn) shootCancelled = true;
                }
                bPreviouslyPressed = gamepad2.b;
                if (gamepad2.x && !xPreviouslyPressed) turretTrackingOn = !turretTrackingOn;
                xPreviouslyPressed = gamepad2.x;
                boolean turretAiming = turretTrackingOn;

                // Flywheel setpoint (gamepad2): d-pad up/down adjusts by 100 RPM. Commanded before
                // the indexer so the indexer interlock sees this loop's flywheel command.
                if (gamepad2.dpad_up && !dpadUpPreviouslyPressed) {
                    flywheelSetpointRpm += Constants.FLYWHEEL_RPM_STEP;
                }
                dpadUpPreviouslyPressed = gamepad2.dpad_up;
                if (gamepad2.dpad_down && !dpadDownPreviouslyPressed) {
                    flywheelSetpointRpm -= Constants.FLYWHEEL_RPM_STEP;
                }
                dpadDownPreviouslyPressed = gamepad2.dpad_down;
                flywheelSetpointRpm = Math.max(0,
                        Math.min(Constants.FLYWHEEL_MAX_RPM, flywheelSetpointRpm));
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

                // Emergency (gamepad2): hold RB = intake + indexer full forward, LB = full
                // reverse. Holding both cancels out and leaves the normal controls in charge.
                int emergencyDirection = (gamepad2.right_bumper ? 1 : 0)
                        - (gamepad2.left_bumper ? 1 : 0);
                boolean emergency = emergencyDirection != 0;

                // Shoot command (gamepad2): hold A. Needs the flywheel on; after the flywheel is
                // turned off, A is ignored until it is released.
                if (!gamepad2.a) shootCancelled = false;
                boolean shootHeld = gamepad2.a && !shootCancelled;
                boolean shootActive = shootHeld && flywheelOn && !emergency;

                // Driver intake (gamepad2): right trigger forward, left trigger reverse.
                double driverIntakePower = gamepad2.right_trigger - gamepad2.left_trigger;
                if (Math.abs(driverIntakePower) < Constants.TRIGGER_DEADBAND) driverIntakePower = 0;

                // Indexer feed speed (gamepad2): d-pad right/left adjusts by 5%. Loading always uses
                // the slower INDEXER_LOAD_SPEED_PERCENT.
                if (gamepad2.dpad_right && !dpadRightPreviouslyPressed) {
                    indexerSpeedPercent = Math.min(100,
                            indexerSpeedPercent + Constants.INDEXER_SPEED_STEP_PERCENT);
                }
                dpadRightPreviouslyPressed = gamepad2.dpad_right;
                if (gamepad2.dpad_left && !dpadLeftPreviouslyPressed) {
                    indexerSpeedPercent = Math.max(0,
                            indexerSpeedPercent - Constants.INDEXER_SPEED_STEP_PERCENT);
                }
                dpadLeftPreviouslyPressed = gamepad2.dpad_left;
                double indexerTargetRpm = Constants.INDEXER_MAX_RPM * indexerSpeedPercent / 100.0;
                double indexerLoadRpm = Constants.INDEXER_MAX_RPM
                        * Constants.INDEXER_LOAD_SPEED_PERCENT / 100.0;

                // Loading and shooting (ShootSequence):
                // - RT intake runs the indexer too until a ball reaches the beam; the indexer then
                //   waits there while the intake keeps following RT.
                // - Holding A (flywheel on) feeds one ball at a time: wait for the flywheel to
                //   recover, feed with the intake stopped until the shot, then intake + indexer
                //   until the next ball reaches the beam, and repeat.
                // - A stuck ball (no shot) is reversed until it is back at the beam, then pushed
                //   at 100% until it shoots.
                ShootSequence.State shootState = shootSequence.update(ballPresent, shootActive,
                        driverIntakePower > 0 && !emergency,
                        flywheelOn ? flywheelSetpointRpm : 0, flywheelRpm, dtSec);
                // Emergency reverse pushes balls back out; forget any ball in the indexer.
                if (emergency && emergencyDirection < 0) shootSequence.clearPendingBall();

                boolean indexerForwardAllowed = robot.isIndexerForwardAllowed();
                double intakePower;
                double indexerCmd; // logged: RPM when running closed loop, power otherwise
                String indexerStatus;
                if (emergency) {
                    intakePower = emergencyDirection;
                    robot.setIndexerPower(emergencyDirection);
                    indexerCmd = emergencyDirection;
                    indexerStatus = emergencyDirection > 0 ? "EMERGENCY FWD" : "EMERGENCY REV";
                } else {
                    switch (shootState) {
                        case LOADING:
                            // Staging up to the beam only, so the flywheel interlock is bypassed.
                            // Slow (INDEXER_LOAD_SPEED_PERCENT) so the ball stops at the beam.
                            robot.setIndexerLoadRpm(indexerLoadRpm);
                            indexerCmd = indexerLoadRpm;
                            intakePower = shootActive ? Constants.SHOOT_INTAKE_POWER
                                    : driverIntakePower;
                            indexerStatus = "LOADING";
                            break;
                        case FEEDING:
                            robot.setIndexerRpm(indexerTargetRpm);
                            indexerCmd = indexerForwardAllowed ? indexerTargetRpm : 0;
                            // Intake off, except short pulses that push a ball still sitting in
                            // the beam onto the indexer rollers.
                            intakePower = shootSequence.isIntakeNudging()
                                    ? Constants.SHOOT_NUDGE_INTAKE_POWER : 0;
                            indexerStatus = !indexerForwardAllowed ? "FEEDING (BLOCKED)"
                                    : shootSequence.isIntakeNudging() ? "FEEDING (NUDGE)"
                                    : "FEEDING";
                            break;
                        case REVERSING:
                            robot.setIndexerPower(Constants.INDEXER_UNJAM_POWER);
                            indexerCmd = Constants.INDEXER_UNJAM_POWER;
                            intakePower = 0;
                            indexerStatus = "STUCK - REVERSING";
                            break;
                        case PUSHING:
                            robot.setIndexerPower(Constants.INDEXER_STUCK_PUSH_POWER);
                            indexerCmd = indexerForwardAllowed
                                    ? Constants.INDEXER_STUCK_PUSH_POWER : 0;
                            intakePower = shootSequence.isIntakeNudging()
                                    ? Constants.SHOOT_NUDGE_INTAKE_POWER : 0;
                            indexerStatus = shootSequence.isIntakeNudging()
                                    ? "STUCK - PUSHING 100% (NUDGE)" : "STUCK - PUSHING 100%";
                            break;
                        case WAITING:
                            robot.setIndexerPower(0);
                            indexerCmd = 0;
                            intakePower = 0;
                            indexerStatus = "WAITING FOR FLYWHEEL";
                            break;
                        case STUCK:
                            robot.setIndexerPower(0);
                            indexerCmd = 0;
                            intakePower = 0;
                            indexerStatus = "STUCK - CLEAR MANUALLY";
                            break;
                        case HOLDING:
                        case IDLE:
                        default:
                            robot.setIndexerPower(0);
                            indexerCmd = 0;
                            intakePower = driverIntakePower;
                            indexerStatus = shootState == ShootSequence.State.HOLDING
                                    ? "BALL READY" : "OFF";
                            break;
                    }
                }
                robot.setIntakePower(intakePower);
                double indexerRpm = robot.getIndexerRpm();

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
                                shootSequence.isBallPending() ? 1 : 0, flywheelOn ? 1 : 0,
                                flywheelOn ? flywheelSetpointRpm : 0, flywheelRpm,
                                shootSequence.isFlywheelReady() ? 1 : 0,
                                shootSequence.getShots(), shootHeld ? 1 : 0, shootState,
                                indexerStatus.replace(' ', '_'), indexerCmd, indexerRpm,
                                shootSequence.getUnjams(), shootSequence.getUnjamsThisBall(),
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
                if (shootState == ShootSequence.State.STUCK) {
                    telemetry.addLine("!! BALL STUCK - release A or hold LB");
                }
                if (shootHeld && !flywheelOn) telemetry.addLine("!! Flywheel off - press B");
                if (emergency) {
                    telemetry.addLine("!! EMERGENCY " + (emergencyDirection > 0 ? "FWD" : "REV"));
                }
                if (log == null) telemetry.addLine("!! Log not recording");
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
                panels.addData("Indexer status", indexerStatus);
                panels.addData("Indexer setpoint (RPM)", (double) Math.round(
                        shootState == ShootSequence.State.LOADING
                                || shootState == ShootSequence.State.FEEDING
                                ? indexerCmd : 0));
                panels.addData("Indexer actual (RPM)", (double) Math.round(indexerRpm));
                panels.addData("Indexer speed (%)", indexerSpeedPercent);
                panels.addData("Indexer forward allowed", indexerForwardAllowed ? 1.0 : 0.0);
                panels.addData("Ball at beam", ballPresent ? 1.0 : 0.0);
                panels.addData("Ball in indexer, not shot",
                        shootSequence.isBallPending() ? 1.0 : 0.0);
                panels.addData("Shots", shootSequence.getShots());
                panels.addData("Unjams", shootSequence.getUnjams());
                panels.addData("Unjams on this ball", shootSequence.getUnjamsThisBall());
                panels.addData("Intake nudging", shootSequence.isIntakeNudging() ? 1.0 : 0.0);
                panels.addData("Intake nudges", shootSequence.getNudges());
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
