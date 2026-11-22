package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.TurretTracker;

@TeleOp(name = "Main 2026", group = "TeleOp")
public class DriveTeleOp extends LinearOpMode {
    private final RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() {
        // Send all telemetry to both the Driver Station and Panels (http://192.168.43.1:8001).
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        try {
            robot.init(hardwareMap);
            // Flower wedge starts UP, as in the test OpModes (moves when INIT is pressed).
            boolean wedgeUp = true;
            robot.setFlowerWedgePosition(Constants.FLOWER_WEDGE_UP);
            // Turret starts at the hold position (facing forward; moves when INIT is pressed).
            // If TURRET_TRACKING_ENABLED, TurretTracker aims it while the flywheel is on;
            // otherwise it stays locked at the hold position. Commands are clipped to
            // TURRET_MIN/MAX_POSITION.
            robot.setTurretPosition(Constants.TURRET_HOLD_POSITION);
            TurretTracker turretTracker = new TurretTracker(Constants.TURRET_HOLD_POSITION);
            telemetry.addLine("Keep robot stationary while Pinpoint calibrates.");
            telemetry.addLine("Left stick: move; right stick X: turn; left stick click: half speed.");
            telemetry.addLine("Robot must start with its front facing the drivers' RIGHT"
                    + " (-90 deg start offset).");
            telemetry.addLine("GP1 (driver): Back/Share makes current facing 'forward' (no offset).");
            telemetry.addLine("GP2: A flywheel + turret aim on/off, dpad up/down RPM +/-100.");
            telemetry.addLine("A ball reaching the indexer beam turns flywheel + aim on.");
            telemetry.addLine("GP2: B indexer on/off, dpad right/left indexer +/-5%.");
            telemetry.addLine("GP2: RT intake, LT reverse intake; Y wedge up/down.");
            telemetry.addLine("GP2: hold RB/LB = EMERGENCY intake+indexer full fwd/rev.");
            if (Constants.TURRET_TRACKING_ENABLED) {
                telemetry.addLine("GP2: right stick X = manual turret while aiming is on.");
            } else {
                telemetry.addLine("Turret tracking DISABLED: turret locked at start position.");
            }
            telemetry.addData("Limelight requested pipeline", robot.getSelectedVisionPipeline());
            telemetry.update();

            waitForStart();
            if (isStopRequested()) return;

            boolean leftStickPreviouslyPressed = false;
            boolean backPreviouslyPressed = false;
            // Start heading offset (-90 deg, front facing drivers' right) applies until the
            // driver resets the heading.
            boolean startHeadingOffsetActive = true;
            double speed = Constants.DRIVE_FULL_SPEED;
            boolean aPreviouslyPressed = false;
            boolean dpadUpPreviouslyPressed = false;
            boolean dpadDownPreviouslyPressed = false;
            boolean flywheelOn = false;
            double flywheelSetpointRpm = Constants.FLYWHEEL_DEFAULT_RPM;
            boolean bPreviouslyPressed = false;
            boolean yPreviouslyPressed = false;
            boolean dpadRightPreviouslyPressed = false;
            boolean dpadLeftPreviouslyPressed = false;
            boolean indexerOn = false;
            double indexerSpeedPercent = Constants.INDEXER_DEFAULT_SPEED_PERCENT;
            // A ball already in the beam at START does not count as newly detected.
            boolean ballPreviouslyPresent = robot.isBallAtIndexer();
            ElapsedTime loopTimer = new ElapsedTime();
            double lastLoopSec = loopTimer.seconds();
            while (opModeIsActive()) {
                double nowSec = loopTimer.seconds();
                double dtSec = nowSec - lastLoopSec;
                lastLoopSec = nowSec;
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

                // Flywheel + turret aim on/off. A ball newly breaking the indexer beam turns them
                // on (only on the not-blocked -> blocked change, so turning them off with A while
                // a ball is still in the beam does not turn them straight back on). Gamepad2 A
                // toggles them; it is checked after the beam so a press always wins.
                boolean ballPresent = robot.isBallAtIndexer();
                if (ballPresent && !ballPreviouslyPresent) flywheelOn = true;
                ballPreviouslyPresent = ballPresent;
                if (gamepad2.a && !aPreviouslyPressed) flywheelOn = !flywheelOn;
                aPreviouslyPressed = gamepad2.a;
                boolean turretAiming = flywheelOn && Constants.TURRET_TRACKING_ENABLED;

                // Turret: while aiming, TurretTracker uses the turret-mounted Limelight's tx plus
                // Pinpoint heading changes (gamepad2 right stick X = manual). Only a valid, fresh
                // result from the selected alliance pipeline counts as a target. Otherwise the
                // turret returns to the hold position and the tracker starts fresh next time.
                if (turretAiming) {
                    LLResult vision = robot.getLatestLimelightResult();
                    boolean targetVisible = vision != null && vision.isValid()
                            && vision.getPipelineIndex() == robot.getSelectedVisionPipeline()
                            && vision.getStaleness() <= Constants.TURRET_MAX_VISION_AGE_MS;
                    double turretPosition = turretTracker.update(
                            headingValid ? heading : Double.NaN,
                            targetVisible, targetVisible ? vision.getTx() : Double.NaN,
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

                // Intake (gamepad2): right trigger forward, left trigger reverse.
                double intakePower = gamepad2.right_trigger - gamepad2.left_trigger;
                if (Math.abs(intakePower) < Constants.TRIGGER_DEADBAND) intakePower = 0;
                if (emergency) intakePower = emergencyDirection;
                robot.setIntakePower(intakePower);

                // Indexer (gamepad2): B toggles, d-pad right/left adjusts speed by 5%.
                // Toggle/speed changes still register during an emergency and apply on release.
                if (gamepad2.b && !bPreviouslyPressed) indexerOn = !indexerOn;
                bPreviouslyPressed = gamepad2.b;
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
                if (emergency) {
                    robot.setIndexerPower(emergencyDirection);
                } else {
                    robot.setIndexerRpm(indexerOn ? indexerTargetRpm : 0);
                }

                // Flower wedge (gamepad2): Y toggles between the up and down presets.
                if (gamepad2.y && !yPreviouslyPressed) {
                    wedgeUp = !wedgeUp;
                    robot.setFlowerWedgePosition(
                            wedgeUp ? Constants.FLOWER_WEDGE_UP : Constants.FLOWER_WEDGE_DOWN);
                }
                yPreviouslyPressed = gamepad2.y;

                // Flywheel (gamepad2): on/off is set above (A or ball detected); d-pad up/down
                // adjusts the setpoint by 100 RPM.
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

                if (emergency) {
                    telemetry.addData("EMERGENCY", "intake + indexer full %s",
                            emergencyDirection > 0 ? "FORWARD" : "REVERSE");
                }
                telemetry.addData("Ball at indexer", "%s (indexBeam1 %s)",
                        ballPresent ? "YES" : "no", robot.getIndexBeam1State() ? "HIGH" : "LOW");
                // Flywheel/indexer RPMs are sent as plain numbers so Panels can graph them.
                // "Setpoint" is the commanded RPM (0 when off); "selected" is what A turns on.
                double flywheelRpm = robot.getFlywheelRpm();
                telemetry.addData("Flywheel status", flywheelOn ? "ON" : "OFF");
                telemetry.addData("Flywheel setpoint (RPM)",
                        (double) Math.round(flywheelOn ? flywheelSetpointRpm : 0));
                telemetry.addData("Flywheel actual (RPM)", (double) Math.round(flywheelRpm));
                telemetry.addData("Flywheel selected (RPM)", (double) Math.round(flywheelSetpointRpm));
                PIDFCoefficients pidf = robot.getFlywheelPidf();
                telemetry.addData("Flywheel PIDF", "P %.2f I %.2f D %.2f F %.2f",
                        pidf.p, pidf.i, pidf.d, pidf.f);
                telemetry.addData("Intake power", "%.2f", intakePower);
                telemetry.addData("Intake L/R velocity (ticks/s)", "%.0f / %.0f",
                        robot.getIntakeLeftVelocity(), robot.getIntakeRightVelocity());
                boolean indexerForwardAllowed = robot.isIndexerForwardAllowed();
                String indexerStatus;
                if (emergency) {
                    indexerStatus = emergencyDirection > 0 ? "EMERGENCY FWD" : "EMERGENCY REV";
                } else if (indexerOn) {
                    indexerStatus = indexerForwardAllowed ? "ON" : "ON (BLOCKED)";
                } else {
                    indexerStatus = "OFF";
                }
                telemetry.addData("Indexer status", indexerStatus);
                telemetry.addData("Indexer setpoint (RPM)",
                        (double) Math.round(indexerOn && !emergency ? indexerTargetRpm : 0));
                telemetry.addData("Indexer actual (RPM)", (double) Math.round(robot.getIndexerRpm()));
                telemetry.addData("Indexer speed (%)", indexerSpeedPercent);
                if (!indexerForwardAllowed) {
                    telemetry.addData("Indexer forward", "BLOCKED: flywheel off or < %.0f RPM",
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
                        turretAiming ? turretTracker.getState()
                                : Constants.TURRET_TRACKING_ENABLED ? "OFF (hold)" : "LOCKED (tracking disabled)",
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
                    robot.stopVision();
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
