package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
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
            telemetry.addData("Limelight requested pipeline", robot.getSelectedVisionPipeline());
            telemetry.update();

            waitForStart();
            if (isStopRequested()) return;

            boolean leftStickPreviouslyPressed = false;
            double speed = 1.0;
            while (opModeIsActive()) {
                robot.selectAllianceLimelightPipeline();
                robot.refreshLimelightResult();
                robot.pinpoint.update();
                Pose2D pose = robot.pinpoint.getPosition();
                double headingOffset = robot.getHeadingOffsetRadians();

                if (gamepad1.left_stick_button && !leftStickPreviouslyPressed) {
                    speed = speed == 1.0 ? 0.5 : 1.0;
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
                robot.stopVision();
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
