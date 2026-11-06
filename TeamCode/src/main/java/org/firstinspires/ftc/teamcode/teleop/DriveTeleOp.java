package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "Mecanum Drive", group = "TeleOp")
public class DriveTeleOp extends LinearOpMode {
    private final RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() {
        try {
            robot.init(hardwareMap);
            telemetry.addLine("Keep robot stationary while Pinpoint calibrates.");
            telemetry.addLine("Left stick: move; right stick X: turn; left stick click: half speed.");
            telemetry.addLine("Start sideways as last year; allianceButton selects red/blue heading.");
            telemetry.addLine("GP1: A flywheel on/off, dpad up/down RPM +/-50, LT/RT indexer fwd/rev.");
            telemetry.addLine("GP2: RT intake, LT reverse intake.");
            telemetry.addData("Limelight requested pipeline", robot.getSelectedVisionPipeline());
            telemetry.update();

            waitForStart();
            if (isStopRequested()) return;

            boolean leftStickPreviouslyPressed = false;
            double speed = Constants.DRIVE_FULL_SPEED;
            boolean aPreviouslyPressed = false;
            boolean dpadUpPreviouslyPressed = false;
            boolean dpadDownPreviouslyPressed = false;
            boolean flywheelOn = false;
            double flywheelSetpointRpm = Constants.FLYWHEEL_DEFAULT_RPM;
            while (opModeIsActive()) {
                robot.selectAllianceLimelightPipeline();
                robot.refreshLimelightResult();
                robot.pinpoint.update();
                Pose2D pose = robot.pinpoint.getPosition();
                double headingOffset = robot.getHeadingOffsetRadians();

                if (gamepad1.left_stick_button && !leftStickPreviouslyPressed) {
                    speed = speed == Constants.DRIVE_FULL_SPEED
                            ? Constants.DRIVE_SLOW_SPEED : Constants.DRIVE_FULL_SPEED;
                }
                leftStickPreviouslyPressed = gamepad1.left_stick_button;

                double heading = pose.getHeading(AngleUnit.RADIANS) + headingOffset;
                if (robot.pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
                        && !Double.isNaN(heading) && !Double.isInfinite(heading)) {
                    robot.fieldCentricDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y,
                            gamepad1.right_stick_x, heading, speed);
                } else {
                    robot.stopDrive();
                    telemetry.addLine("Drive stopped: Pinpoint must be ready with a valid heading.");
                }

                // Intake (gamepad2): right trigger forward, left trigger reverse.
                double intakePower = gamepad2.right_trigger - gamepad2.left_trigger;
                robot.setIntakePower(intakePower);

                // Indexer (gamepad1): left trigger forward, right trigger reverse.
                double indexerPower = gamepad1.left_trigger - gamepad1.right_trigger;
                robot.setIndexerPower(indexerPower);

                // Flywheel (gamepad1): A toggles, d-pad up/down adjusts setpoint by 50 RPM.
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
                telemetry.addData("Indexer power", "%.2f", indexerPower);
                telemetry.addData("Indexer velocity (ticks/s)", "%.0f", robot.getIndexerVelocity());
                telemetry.addData("Alliance", robot.isRedAlliance() ? "RED" : "BLUE");
                telemetry.addData("Drive speed", speed);
                telemetry.addData("Pinpoint status", robot.pinpoint.getDeviceStatus());
                telemetry.addData("X (in)", "%.3f", pose.getX(DistanceUnit.INCH));
                telemetry.addData("Y (in)", "%.3f", pose.getY(DistanceUnit.INCH));
                telemetry.addData("Heading (deg)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
                telemetry.addData("Heading offset (deg)", "%.1f", Math.toDegrees(headingOffset));
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
