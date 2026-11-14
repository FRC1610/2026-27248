package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * Robot-centric mecanum drive test: drive motors only, no odometry or field-oriented driving.
 *
 * <p>Gamepad1 (same as DriveTeleOp): left stick moves, right stick X turns,
 * left stick click toggles half speed.
 */
@TeleOp(name = "Mecanum Drive Test", group = "Test")
public class MecanumDriveTest extends LinearOpMode {

    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Directions and zero-power behavior match RobotHardware.
        configureMotor(leftFront, DcMotor.Direction.FORWARD);
        configureMotor(leftBack, DcMotor.Direction.FORWARD);
        configureMotor(rightFront, DcMotor.Direction.REVERSE);
        configureMotor(rightBack, DcMotor.Direction.REVERSE);

        telemetry.addLine("Mecanum Drive Test ready (robot-centric).");
        telemetry.addLine("Left stick: move; right stick X: turn; left stick click: half speed.");
        telemetry.update();

        waitForStart();

        try {
            double speed = Constants.DRIVE_FULL_SPEED;
            boolean leftStickPreviouslyPressed = false;

            while (opModeIsActive()) {
                if (gamepad1.left_stick_button && !leftStickPreviouslyPressed) {
                    speed = speed == Constants.DRIVE_FULL_SPEED
                            ? Constants.DRIVE_SLOW_SPEED : Constants.DRIVE_FULL_SPEED;
                }
                leftStickPreviouslyPressed = gamepad1.left_stick_button;

                double x = gamepad1.left_stick_x * Constants.STRAFE_CORRECTION;
                double y = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;

                // Same mixing as RobotHardware.fieldCentricDrive with heading fixed at 0.
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);
                double scale = Math.max(0, Math.min(speed, 1));
                double leftFrontPower = (y + x + turn) / denominator * scale;
                double leftBackPower = (y - x + turn) / denominator * scale;
                double rightFrontPower = (y - x - turn) / denominator * scale;
                double rightBackPower = (y + x - turn) / denominator * scale;

                leftFront.setPower(leftFrontPower);
                leftBack.setPower(leftBackPower);
                rightFront.setPower(rightFrontPower);
                rightBack.setPower(rightBackPower);

                telemetry.addData("Drive speed", speed);
                telemetry.addData("Stick x / y / turn", "%.2f / %.2f / %.2f",
                        gamepad1.left_stick_x, y, turn);
                telemetry.addData("LF / RF power", "%.2f / %.2f", leftFrontPower, rightFrontPower);
                telemetry.addData("LB / RB power", "%.2f / %.2f", leftBackPower, rightBackPower);
                telemetry.addData("LF / RF velocity (ticks/s)", "%.0f / %.0f",
                        leftFront.getVelocity(), rightFront.getVelocity());
                telemetry.addData("LB / RB velocity (ticks/s)", "%.0f / %.0f",
                        leftBack.getVelocity(), rightBack.getVelocity());
                telemetry.update();
            }
        } finally {
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
        }
    }

    private void configureMotor(DcMotorEx motor, DcMotor.Direction direction) {
        motor.setPower(0);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
