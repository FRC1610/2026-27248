package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

/** Copy of Main 2026 with every control on gamepad 1, kept for single-driver testing. */
@Utility(name = "Single Gamepad Test",
        description = "Main 2026 with every control on gamepad 1")
public class SingleGamepadTest extends LinearOpMode {
    private final RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() {
        try {
            robot.init(hardwareMap);
            // Flower wedge starts UP, as in the test OpModes (moves when INIT is pressed).
            boolean wedgeUp = true;
            robot.setFlowerWedgePosition(Constants.FLOWER_WEDGE_UP);
            // Turret is held at a fixed position for now (moves when INIT is pressed).
            robot.setTurretPosition(Constants.TURRET_HOLD_POSITION);
            telemetry.addLine("Keep robot stationary while Pinpoint calibrates.");
            telemetry.addLine("Left stick: move; right stick X: turn; left stick click: half speed.");
            telemetry.addLine("Field-centric 'forward' = the direction the robot faces at INIT.");
            telemetry.addLine("GP1: A flywheel on/off, dpad up/down RPM +/-100.");
            telemetry.addLine("GP1: B indexer on/off, dpad right/left indexer +/-5%.");
            telemetry.addLine("GP1: RT intake, LT reverse intake; Y wedge up/down.");
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
            while (opModeIsActive()) {
                robot.selectAllianceLimelightPipeline();
                robot.refreshLimelightResult();
                robot.pinpoint.update();
                Pose2D pose = robot.pinpoint.getPosition();

                if (gamepad1.left_stick_button && !leftStickPreviouslyPressed) {
                    speed = speed == Constants.DRIVE_FULL_SPEED
                            ? Constants.DRIVE_SLOW_SPEED : Constants.DRIVE_FULL_SPEED;
                }
                leftStickPreviouslyPressed = gamepad1.left_stick_button;

                // Alliance heading offset (+/-90 deg) removed for now; RobotHardware still provides
                // getHeadingOffsetRadians() if it's needed again.
                double heading = pose.getHeading(AngleUnit.RADIANS);
                if (robot.pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
                        && !Double.isNaN(heading) && !Double.isInfinite(heading)) {
                    robot.fieldCentricDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y,
                            gamepad1.right_stick_x, heading, speed);
                } else {
                    robot.stopDrive();
                    telemetry.addLine("Drive stopped: Pinpoint must be ready with a valid heading.");
                }

                // Intake (gamepad1): right trigger forward, left trigger reverse.
                double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
                if (Math.abs(intakePower) < Constants.TRIGGER_DEADBAND) intakePower = 0;
                robot.setIntakePower(intakePower);

                // Indexer (gamepad1): B toggles, d-pad right/left adjusts speed by 5%.
                if (gamepad1.b && !bPreviouslyPressed) indexerOn = !indexerOn;
                bPreviouslyPressed = gamepad1.b;
                if (gamepad1.dpad_right && !dpadRightPreviouslyPressed) {
                    indexerSpeedPercent = Math.min(100,
                            indexerSpeedPercent + Constants.INDEXER_SPEED_STEP_PERCENT);
                }
                dpadRightPreviouslyPressed = gamepad1.dpad_right;
                if (gamepad1.dpad_left && !dpadLeftPreviouslyPressed) {
                    indexerSpeedPercent = Math.max(0,
                            indexerSpeedPercent - Constants.INDEXER_SPEED_STEP_PERCENT);
                }
                dpadLeftPreviouslyPressed = gamepad1.dpad_left;
                double indexerTargetRpm = Constants.INDEXER_MAX_RPM * indexerSpeedPercent / 100.0;
                robot.setIndexerRpm(indexerOn ? indexerTargetRpm : 0);

                // Flower wedge (gamepad1): Y toggles between the up and down presets.
                if (gamepad1.y && !yPreviouslyPressed) {
                    wedgeUp = !wedgeUp;
                    robot.setFlowerWedgePosition(
                            wedgeUp ? Constants.FLOWER_WEDGE_UP : Constants.FLOWER_WEDGE_DOWN);
                }
                yPreviouslyPressed = gamepad1.y;

                // Flywheel (gamepad1): A toggles, d-pad up/down adjusts setpoint by 100 RPM.
                if (gamepad1.a && !aPreviouslyPressed) flywheelOn = !flywheelOn;
                aPreviouslyPressed = gamepad1.a;
                if (gamepad1.dpad_up && !dpadUpPreviouslyPressed) {
                    flywheelSetpointRpm += Constants.FLYWHEEL_RPM_STEP;
                }
                dpadUpPreviouslyPressed = gamepad1.dpad_up;
                if (gamepad1.dpad_down && !dpadDownPreviouslyPressed) {
                    flywheelSetpointRpm -= Constants.FLYWHEEL_RPM_STEP;
                }
                dpadDownPreviouslyPressed = gamepad1.dpad_down;
                flywheelSetpointRpm = Math.max(0,
                        Math.min(Constants.FLYWHEEL_MAX_RPM, flywheelSetpointRpm));
                robot.setFlywheelRpm(flywheelOn ? flywheelSetpointRpm : 0);

                telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
                telemetry.addData("Flywheel setpoint (RPM)", "%.0f", flywheelSetpointRpm);
                double flywheelRpm = robot.getFlywheelRpm();
                // Lights: alliance color, or pulsing orange/green with a ball loaded.
                robot.lights.update(robot.isRedAlliance(), robot.isBallAtIndexer(), flywheelOn,
                        flywheelSetpointRpm, flywheelRpm);
                telemetry.addData("Lights", robot.lights.isPresent()
                        ? String.valueOf(robot.lights.getState()) : "no prism configured");
                telemetry.addData("Flywheel current (RPM)", "%.0f", flywheelRpm);
                if (flywheelOn) {
                    telemetry.addData("Flywheel error (RPM)", "%.0f",
                            flywheelSetpointRpm - flywheelRpm);
                }
                PIDFCoefficients pidf = robot.getFlywheelPidf();
                telemetry.addData("Flywheel PIDF", "P %.2f I %.2f D %.2f F %.2f",
                        pidf.p, pidf.i, pidf.d, pidf.f);
                telemetry.addData("Intake power", "%.2f", intakePower);
                telemetry.addData("Intake L/R velocity (ticks/s)", "%.0f / %.0f",
                        robot.getIntakeLeftVelocity(), robot.getIntakeRightVelocity());
                telemetry.addData("Indexer", "%s, %.0f%% (target %.0f RPM)",
                        indexerOn ? "ON" : "OFF", indexerSpeedPercent, indexerTargetRpm);
                telemetry.addData("Indexer current (RPM)", "%.0f", robot.getIndexerRpm());
                if (!robot.isIndexerForwardAllowed()) {
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
                telemetry.addData("Heading (deg)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
                telemetry.addData("Turret", "%.2f (held)", robot.getTurretPosition());
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
