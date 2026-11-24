package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Constants;
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
        // Send all telemetry to both the Driver Station and Panels (http://192.168.43.1:8001).
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        BufferedWriter log = null;
        try {
            robot.init(hardwareMap);
            VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
            // Flower wedge starts UP, as in the test OpModes (moves when INIT is pressed).
            boolean wedgeUp = true;
            robot.setFlowerWedgePosition(Constants.FLOWER_WEDGE_UP);
            // Turret starts at the hold position (facing forward; moves when INIT is pressed).
            // While tracking is on (gamepad2 B), TurretTracker aims it; otherwise it holds the
            // hold position. Commands are clipped to TURRET_MIN/MAX_POSITION.
            robot.setTurretPosition(Constants.TURRET_HOLD_POSITION);
            TurretTracker turretTracker = new TurretTracker(Constants.TURRET_HOLD_POSITION);
            telemetry.addLine("Keep robot stationary while Pinpoint calibrates.");
            telemetry.addLine("Left stick: move; right stick X: turn; left stick click: half speed.");
            telemetry.addLine("Robot must start with its front facing the drivers' RIGHT"
                    + " (-90 deg start offset).");
            telemetry.addLine("GP1 (driver): Back/Share makes current facing 'forward' (no offset).");
            telemetry.addLine("GP2: X flywheel on/off, dpad up/down RPM +/-100.");
            telemetry.addLine("GP2: B turret tracking on/off.");
            telemetry.addLine("GP2: hold A = SHOOT (one ball at a time, flywheel must be on).");
            telemetry.addLine("GP2: RT intake (indexer loads a ball to the beam), LT reverse.");
            telemetry.addLine("GP2: dpad right/left indexer +/-5%; Y wedge up/down.");
            telemetry.addLine("GP2: hold RB/LB = EMERGENCY intake+indexer full fwd/rev.");
            telemetry.addLine("Stuck-ball recovery (reverse to beam, push 100%) is "
                    + (Constants.INDEXER_STUCK_RECOVERY_ENABLED ? "ON." : "OFF."));
            telemetry.addLine("GP2: right stick X = manual turret while tracking is on.");
            telemetry.addLine("Turret tracking starts "
                    + (Constants.TURRET_TRACKING_ON_AT_START ? "ON." : "OFF (turret at start)."));
            telemetry.addData("Limelight requested pipeline", robot.getSelectedVisionPipeline());
            telemetry.update();

            // INIT loop instead of waitForStart() so the lights (and Limelight pipeline) follow
            // allianceButton until START.
            while (opModeInInit()) {
                robot.selectAllianceLimelightPipeline();
                robot.lights.showAlliance(robot.isRedAlliance());
                idle();
            }
            if (isStopRequested()) return;

            boolean leftStickPreviouslyPressed = false;
            boolean backPreviouslyPressed = false;
            // Start heading offset (-90 deg, front facing drivers' right) applies until the
            // driver resets the heading.
            boolean startHeadingOffsetActive = true;
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
                        + "battery_v\n");
                logPath = file.getAbsolutePath();
            } catch (IOException e) {
                log = null;
                logPath = "LOG FAILED: " + e.getMessage();
            }
            double volts = battery.getVoltage();
            double lastFlushSec = 0;
            long loops = 0;

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
                robot.pinpoint.update();
                Pose2D pose = robot.pinpoint.getPosition();

                if (gamepad1.left_stick_button && !leftStickPreviouslyPressed) {
                    speed = speed == Constants.DRIVE_FULL_SPEED
                            ? Constants.DRIVE_SLOW_SPEED : Constants.DRIVE_FULL_SPEED;
                }
                leftStickPreviouslyPressed = gamepad1.left_stick_button;

                // Heading reset (gamepad1): Back/Share makes the current facing field-centric
                // "forward" (the start offset is dropped). X/Y are kept. Takes effect from the next
                // Pinpoint update.
                if (gamepad1.back && !backPreviouslyPressed) {
                    robot.resetHeading();
                    startHeadingOffsetActive = false;
                    turretTracker.resetHeadingReference(); // the jump is not a robot turn
                    pose = new Pose2D(DistanceUnit.INCH, pose.getX(DistanceUnit.INCH),
                            pose.getY(DistanceUnit.INCH), AngleUnit.RADIANS, 0);
                }
                backPreviouslyPressed = gamepad1.back;

                // Field-centric heading = Pinpoint heading + start offset (until reset).
                double heading = pose.getHeading(AngleUnit.RADIANS)
                        + (startHeadingOffsetActive ? robot.getHeadingOffsetRadians() : 0);
                boolean headingValid =
                        robot.pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
                                && !Double.isNaN(heading) && !Double.isInfinite(heading);
                if (headingValid) {
                    robot.fieldCentricDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y,
                            gamepad1.right_stick_x, heading, speed);
                } else {
                    robot.stopDrive();
                    telemetry.addLine("Drive stopped: Pinpoint must be ready with a valid heading.");
                }

                // Flywheel and turret tracking on/off (gamepad2): X toggles the flywheel, B toggles
                // tracking. A ball at the indexer beam does not turn either on. Turning the
                // flywheel off also stops the indexer: the shoot command is cancelled until A is
                // released.
                boolean ballPresent = robot.isBallAtIndexer();
                if (gamepad2.x && !xPreviouslyPressed) {
                    flywheelOn = !flywheelOn;
                    if (!flywheelOn) shootCancelled = true;
                }
                xPreviouslyPressed = gamepad2.x;
                if (gamepad2.b && !bPreviouslyPressed) turretTrackingOn = !turretTrackingOn;
                bPreviouslyPressed = gamepad2.b;
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
                            intakePower = 0;
                            indexerStatus = indexerForwardAllowed ? "FEEDING" : "FEEDING (BLOCKED)";
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
                            intakePower = 0;
                            indexerStatus = "STUCK - PUSHING 100%";
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
                                        + "%.2f\n",
                                nowSec, dtSec * 1000.0, ballPresent ? 1 : 0,
                                shootSequence.isBallPending() ? 1 : 0, flywheelOn ? 1 : 0,
                                flywheelOn ? flywheelSetpointRpm : 0, flywheelRpm,
                                shootSequence.isFlywheelReady() ? 1 : 0,
                                shootSequence.getShots(), shootHeld ? 1 : 0, shootState,
                                indexerStatus.replace(' ', '_'), indexerCmd, indexerRpm,
                                shootSequence.getUnjams(), shootSequence.getUnjamsThisBall(),
                                intakePower, volts));
                        if (nowSec - lastFlushSec > 1.0) {
                            log.flush();
                            lastFlushSec = nowSec;
                        }
                    } catch (IOException e) {
                        logPath = "LOG WRITE FAILED: " + e.getMessage();
                        log = null;
                    }
                }

                if (shootState == ShootSequence.State.STUCK) {
                    telemetry.addLine("INDEXER STUCK - CLEAR MANUALLY: "
                            + Constants.INDEXER_MAX_UNJAMS_PER_BALL
                            + " reverse/push tries failed. Release A to retry, or hold LB to reverse.");
                }
                if (shootHeld && !flywheelOn) {
                    telemetry.addLine("SHOOT ignored: flywheel is off (GP2 X).");
                }
                if (emergency) {
                    telemetry.addData("EMERGENCY", "intake + indexer full %s",
                            emergencyDirection > 0 ? "FORWARD" : "REVERSE");
                }
                telemetry.addData("Ball at indexer", "%s (indexBeam1 %s)",
                        ballPresent ? "YES" : "no", robot.getIndexBeam1State() ? "HIGH" : "LOW");
                // Flywheel/indexer RPMs are sent as plain numbers so Panels can graph them.
                // "Setpoint" is the commanded RPM (0 when off); "selected" is what X turns on.
                // Lights: alliance color, or purple/green snakes with a ball loaded.
                robot.lights.update(robot.isRedAlliance(), ballPresent, flywheelOn,
                        flywheelSetpointRpm, flywheelRpm);
                telemetry.addData("Lights", robot.lights.isPresent()
                        ? String.valueOf(robot.lights.getState()) : "no prism configured");
                telemetry.addData("Flywheel status", flywheelOn ? "ON" : "OFF");
                telemetry.addData("Flywheel setpoint (RPM)",
                        (double) Math.round(flywheelOn ? flywheelSetpointRpm : 0));
                telemetry.addData("Flywheel actual (RPM)", (double) Math.round(flywheelRpm));
                telemetry.addData("Flywheel selected (RPM)", (double) Math.round(flywheelSetpointRpm));
                PIDFCoefficients pidf = robot.getFlywheelPidf();
                telemetry.addData("Flywheel PIDF", "P %.2f I %.2f D %.2f F %.2f",
                        pidf.p, pidf.i, pidf.d, pidf.f);
                telemetry.addData("Shoot (hold A)", shootActive ? "ACTIVE" : "off");
                telemetry.addData("Shoot state", shootState);
                telemetry.addData("Intake power", "%.2f", intakePower);
                telemetry.addData("Intake L/R velocity (ticks/s)", "%.0f / %.0f",
                        robot.getIntakeLeftVelocity(), robot.getIntakeRightVelocity());
                telemetry.addData("Indexer status", indexerStatus);
                telemetry.addData("Indexer setpoint (RPM)", (double) Math.round(
                        shootState == ShootSequence.State.LOADING
                                || shootState == ShootSequence.State.FEEDING
                                ? indexerCmd : 0));
                telemetry.addData("Indexer actual (RPM)", (double) Math.round(indexerRpm));
                telemetry.addData("Indexer speed (%)", indexerSpeedPercent);
                telemetry.addData("Stuck recovery", Constants.INDEXER_STUCK_RECOVERY_ENABLED
                        ? "ON" : "OFF");
                telemetry.addData("Ball in indexer, not shot",
                        shootSequence.isBallPending() ? 1.0 : 0.0);
                telemetry.addData("Shots (flywheel dips)", shootSequence.getShots());
                telemetry.addData("Unjams", shootSequence.getUnjams());
                telemetry.addData("Unjams on this ball", shootSequence.getUnjamsThisBall());
                telemetry.addData("Battery (V)", volts);
                telemetry.addData("Log", logPath);
                if (!indexerForwardAllowed) {
                    telemetry.addData("Indexer feed", "BLOCKED: flywheel off or < %.0f RPM",
                            Constants.INDEXER_MIN_FLYWHEEL_RPM);
                }
                telemetry.addData("Flower wedge", "%s (%.2f)", wedgeUp ? "UP" : "DOWN",
                        robot.getFlowerWedgePosition());
                telemetry.addData("Alliance", robot.isRedAlliance() ? "RED" : "BLUE");
                telemetry.addData("Drive speed", speed);
                telemetry.addData("Pinpoint status", robot.pinpoint.getDeviceStatus());
                telemetry.addData("X (in)", "%.3f", pose.getX(DistanceUnit.INCH));
                telemetry.addData("Y (in)", "%.3f", pose.getY(DistanceUnit.INCH));
                telemetry.addData("Heading (deg)", "Pinpoint %.1f, drive %.1f%s",
                        pose.getHeading(AngleUnit.DEGREES),
                        AngleUnit.normalizeDegrees(Math.toDegrees(heading)),
                        startHeadingOffsetActive ? " (start offset)" : "");
                telemetry.addData("Turret", "%s, pos %.3f",
                        turretAiming ? "TRACKING ON - " + turretTracker.getState()
                                : "TRACKING OFF (hold)",
                        robot.getTurretPosition());
                telemetry.addData("Turret angle (deg)", "cmd %.1f, aim %.1f, range %.1f..%.1f",
                        turretTracker.getCommandedAngleDeg(), turretTracker.getAimAngleDeg(),
                        turretTracker.getMinAngleDeg(), turretTracker.getMaxAngleDeg());
                addVisionTelemetry();
                telemetry.update();
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

    private void addVisionTelemetry() {
        telemetry.addData("Limelight requested pipeline", robot.getSelectedVisionPipeline());
        LLResult result = robot.getLatestLimelightResult();
        if (result == null) {
            telemetry.addData("Limelight", "Waiting for data");
            return;
        }

        telemetry.addData("Limelight actual pipeline", result.getPipelineIndex());
        telemetry.addData("Limelight data age (ms)", result.getStaleness());
        if (result.getPipelineIndex() != robot.getSelectedVisionPipeline()) {
            telemetry.addData("Limelight", "Waiting for pipeline switch");
        } else if (result.isValid()) {
            telemetry.addData("Limelight", "Target detected (check data age)");
            telemetry.addData("Limelight tx/ty (deg)", "%.2f / %.2f", result.getTx(), result.getTy());
            telemetry.addData("Limelight target area (%)", "%.2f", result.getTa());
            Pose3D botPose = result.getBotpose();
            if (botPose != null) telemetry.addData("Limelight field pose", botPose);
        } else {
            telemetry.addData("Limelight", "No valid target");
        }
    }
}
