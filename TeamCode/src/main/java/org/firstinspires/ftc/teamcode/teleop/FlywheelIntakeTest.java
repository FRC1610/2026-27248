package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * Bench test for the flywheel, intake and flower wedge.
 *
 * A: flywheel on/off, D-pad up/down: flywheel +/-100 RPM.
 * RT: intake forward, LT: intake reverse (proportional).
 * Y: flower wedge toggle between FLOWER_WEDGE_UP and FLOWER_WEDGE_DOWN (starts UP at INIT).
 */
@TeleOp(name = "Flywheel Intake Test", group = "Test")
public class FlywheelIntakeTest extends LinearOpMode {

    // Intake encoder ticks per output revolution
    private static final double INTAKE_TICKS_PER_REV = 103.6;

    private static final double FLYWHEEL_RPM_STEP = 100.0;
    private static final double TRIGGER_DEADBAND = 0.05;

    private DcMotorEx flywheel;
    private DcMotorEx intakeLeft;
    private DcMotorEx intakeRight;
    private DigitalChannel indexBeam1;
    private ServoImplEx flowerWedgeLeft;
    private ServoImplEx flowerWedgeRight;

    private double targetRPM = Constants.FLYWHEEL_DEFAULT_RPM;
    private boolean flywheelEnabled = false;

    private boolean wedgeUp = true;

    private boolean previousA = false;
    private boolean previousY = false;
    private boolean previousUp = false;
    private boolean previousDown = false;

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        intakeLeft = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotorEx.class, "intakeRight");
        indexBeam1 = hardwareMap.get(DigitalChannel.class, "indexBeam1");
        indexBeam1.setMode(DigitalChannel.Mode.INPUT);
        flowerWedgeLeft = hardwareMap.get(ServoImplEx.class, "flowerWedgeLeft");
        flowerWedgeRight = hardwareMap.get(ServoImplEx.class, "flowerWedgeRight");

        // Directions match RobotHardware.
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setVelocity(0);

        intakeLeft.setDirection(DcMotor.Direction.REVERSE);
        intakeRight.setDirection(DcMotor.Direction.FORWARD);
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeLeft.setPower(0);
        intakeRight.setPower(0);

        // Mounted opposite each other on the same mechanism; directions match RobotHardware.
        flowerWedgeLeft.setDirection(Servo.Direction.FORWARD);
        flowerWedgeRight.setDirection(Servo.Direction.REVERSE);
        setWedge();

        telemetry.addLine("Flywheel Intake Test ready (wedge moved UP)");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {
                boolean currentA = gamepad1.a;
                boolean currentY = gamepad1.y;
                boolean currentUp = gamepad1.dpad_up;
                boolean currentDown = gamepad1.dpad_down;

                // ---- Flywheel ----
                if (currentA && !previousA) {
                    flywheelEnabled = !flywheelEnabled;
                }
                if (currentUp && !previousUp) {
                    targetRPM = Math.min(Constants.FLYWHEEL_MAX_RPM, targetRPM + FLYWHEEL_RPM_STEP);
                }
                if (currentDown && !previousDown) {
                    targetRPM = Math.max(0.0, targetRPM - FLYWHEEL_RPM_STEP);
                }

                double flywheelTargetTPS = targetRPM * Constants.FLYWHEEL_TICKS_PER_REV / 60.0;
                flywheel.setVelocity(flywheelEnabled ? flywheelTargetTPS : 0);

                // ---- Intake ----
                double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
                if (Math.abs(intakePower) < TRIGGER_DEADBAND) {
                    intakePower = 0.0;
                }
                intakeLeft.setPower(intakePower);
                intakeRight.setPower(intakePower);

                // ---- Flower wedge ----
                if (currentY && !previousY) {
                    wedgeUp = !wedgeUp;
                    setWedge();
                }

                previousA = currentA;
                previousY = currentY;
                previousUp = currentUp;
                previousDown = currentDown;

                // ---- Telemetry ----
                double flywheelActualRPM = flywheel.getVelocity() * 60.0 / Constants.FLYWHEEL_TICKS_PER_REV;
                double intakeLeftRPM = intakeLeft.getVelocity() * 60.0 / INTAKE_TICKS_PER_REV;
                double intakeRightRPM = intakeRight.getVelocity() * 60.0 / INTAKE_TICKS_PER_REV;

                telemetry.addLine("===== FLYWHEEL =====");
                telemetry.addData("Flywheel Status", flywheelEnabled ? "ON" : "OFF");
                telemetry.addData("Flywheel Target RPM", "%.0f", targetRPM);
                telemetry.addData("Flywheel Actual RPM", "%.0f", flywheelActualRPM);
                telemetry.addLine();

                telemetry.addLine("===== INTAKE =====");
                telemetry.addData("Intake Power", "%.0f%%", intakePower * 100.0);
                telemetry.addData("Intake Left RPM", "%.0f", intakeLeftRPM);
                telemetry.addData("Intake Right RPM", "%.0f", intakeRightRPM);
                telemetry.addLine();

                telemetry.addLine("===== FLOWER WEDGE =====");
                telemetry.addData("Wedge", "%s (%.2f)", wedgeUp ? "UP" : "DOWN",
                        flowerWedgeLeft.getPosition());
                telemetry.addLine();

                telemetry.addLine("===== SENSORS =====");
                telemetry.addData("indexBeam1", indexBeam1.getState() ? "HIGH (true)" : "LOW (false)");
                telemetry.addLine();

                telemetry.addLine("===== CONTROLS =====");
                telemetry.addLine("A: Flywheel ON/OFF");
                telemetry.addLine("UP/DOWN: Flywheel +/-100 RPM");
                telemetry.addLine("RT: Intake Forward");
                telemetry.addLine("LT: Intake Reverse");
                telemetry.addLine("Y: Wedge UP/DOWN");
                telemetry.update();
            }
        } finally {
            flywheel.setPower(0);
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
            flowerWedgeLeft.setPwmDisable();
            flowerWedgeRight.setPwmDisable();
        }
    }

    /** Commands both wedge servos to the current preset (right is reversed). */
    private void setWedge() {
        double position = wedgeUp ? Constants.FLOWER_WEDGE_UP : Constants.FLOWER_WEDGE_DOWN;
        flowerWedgeLeft.setPosition(position);
        flowerWedgeRight.setPosition(position);
    }
}
