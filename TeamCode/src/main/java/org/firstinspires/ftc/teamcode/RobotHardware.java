package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/** Drive configuration carried over from 2025-27248 RobotHardware and PedroConstants. */
public class RobotHardware {
    public GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    private LLResult latestLimelightResult;
    private int selectedVisionPipeline = -1;
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private DcMotorEx intakeLeft;
    private DcMotorEx intakeRight;
    private DcMotorEx indexer;
    private DcMotorEx flywheel;
    private PIDFCoefficients flywheelPidf;
    private DigitalChannel allianceButton;
    private DigitalChannel indexBeam1;
    private ServoImplEx flowerWedgeLeft;
    private ServoImplEx flowerWedgeRight;
    private ServoImplEx turret;

    public void init(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        configureMotor(leftFront, DcMotorSimple.Direction.FORWARD);
        configureMotor(leftBack, DcMotorSimple.Direction.FORWARD);
        configureMotor(rightFront, DcMotorSimple.Direction.REVERSE);
        configureMotor(rightBack, DcMotorSimple.Direction.REVERSE);

        intakeLeft = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotorEx.class, "intakeRight");
        indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        // Intake motors face each other, so one is reversed to spin the rollers together.
        // Directions match the TestMode OpMode verified on the robot.
        configureMotor(intakeLeft, DcMotorSimple.Direction.REVERSE);
        configureMotor(intakeRight, DcMotorSimple.Direction.FORWARD);
        configureMotor(indexer, DcMotorSimple.Direction.FORWARD);
        // FLOAT lets the flywheel coast down instead of braking hard on the gearbox.
        configureMotor(flywheel, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT);
        // setVelocity() in RUN_USING_ENCODER runs the hub's closed-loop velocity PIDF.
        // Read the active gains once so they can be shown on the Driver Station.
        flywheelPidf = flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // Flower wedge servos are mounted opposite each other on the same mechanism, so the
        // right one is reversed: one shared position turns them in opposite directions.
        // Directions match ServoTest and ServoPositionTest. No position is commanded
        // here, so the servos don't move during INIT until up/down positions are known.
        flowerWedgeLeft = hardwareMap.get(ServoImplEx.class, "flowerWedgeLeft");
        flowerWedgeRight = hardwareMap.get(ServoImplEx.class, "flowerWedgeRight");
        turret = hardwareMap.get(ServoImplEx.class, "turret");
        flowerWedgeLeft.setDirection(Servo.Direction.FORWARD);
        flowerWedgeRight.setDirection(Servo.Direction.REVERSE);
        turret.setDirection(Servo.Direction.FORWARD);

        allianceButton = hardwareMap.get(DigitalChannel.class, "allianceButton");
        allianceButton.setMode(DigitalChannel.Mode.INPUT);

        indexBeam1 = hardwareMap.get(DigitalChannel.class, "indexBeam1");
        indexBeam1.setMode(DigitalChannel.Mode.INPUT);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(Constants.FORWARD_POD_Y, Constants.STRAFE_POD_X, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        // Keep the robot stationary during initialization while the IMU calibrates.
        pinpoint.resetPosAndIMU();

        // Same hardware name and alliance pipelines as 2025-27248.
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(Constants.LIMELIGHT_POLL_RATE_HZ);
        selectAllianceLimelightPipeline();
        limelight.start();
        refreshLimelightResult();
    }

    /** Pipeline switching is asynchronous; the result reports the actual pipeline. */
    public void selectAllianceLimelightPipeline() {
        int pipeline = isRedAlliance()
                ? Constants.RED_VISION_PIPELINE : Constants.BLUE_VISION_PIPELINE;
        if (pipeline != selectedVisionPipeline) {
            limelight.pipelineSwitch(pipeline);
            selectedVisionPipeline = pipeline;
            latestLimelightResult = null;
        }
    }

    /** Cache one result per control loop for telemetry and future vision consumers. */
    public void refreshLimelightResult() {
        latestLimelightResult = limelight == null ? null : limelight.getLatestResult();
    }

    public LLResult getLatestLimelightResult() {
        return latestLimelightResult;
    }

    public int getSelectedVisionPipeline() {
        return selectedVisionPipeline;
    }

    public void stopVision() {
        if (limelight != null) limelight.stop();
        latestLimelightResult = null;
    }

    private void configureMotor(DcMotorEx motor, DcMotorSimple.Direction direction) {
        configureMotor(motor, direction, DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void configureMotor(DcMotorEx motor, DcMotorSimple.Direction direction,
                                DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setPower(0);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /** Positive power intakes, negative ejects; clipped to [-1, 1]. Both motors move together. */
    public void setIntakePower(double power) {
        double p = clip(power, -1, 1);
        intakeLeft.setPower(p);
        intakeRight.setPower(p);
    }

    /** Positive power runs the indexer forward; clipped to [-1, 1]. */
    public void setIndexerPower(double power) {
        indexer.setPower(clip(power, -1, 1));
    }

    /** Closed-loop flywheel speed in motor/flywheel RPM, clipped to [0, FLYWHEEL_MAX_RPM]. */
    public void setFlywheelRpm(double rpm) {
        double target = clip(rpm, 0, Constants.FLYWHEEL_MAX_RPM);
        if (target == 0) {
            flywheel.setPower(0);
        } else {
            flywheel.setVelocity(target * Constants.FLYWHEEL_TICKS_PER_REV / 60.0);
        }
    }

    /** Moves both flower wedge servos to the same position (right is reversed); clipped to [0, 1]. */
    public void setFlowerWedgePosition(double position) {
        double p = clip(position, 0, 1);
        flowerWedgeLeft.setPwmEnable();
        flowerWedgeRight.setPwmEnable();
        flowerWedgeLeft.setPosition(p);
        flowerWedgeRight.setPosition(p);
    }

    /** Last commanded flower wedge position (servos have no position feedback). */
    public double getFlowerWedgePosition() {
        return flowerWedgeLeft.getPosition();
    }

    /** Turret servo position, clipped to [0, 1]. */
    public void setTurretPosition(double position) {
        turret.setPwmEnable();
        turret.setPosition(clip(position, 0, 1));
    }

    /** Last commanded turret position (servo has no position feedback). */
    public double getTurretPosition() {
        return turret.getPosition();
    }

    /** Raw encoder velocities in ticks/sec (gear ratios not yet specified for these motors). */
    public double getIntakeLeftVelocity() {
        return intakeLeft.getVelocity();
    }

    public double getIntakeRightVelocity() {
        return intakeRight.getVelocity();
    }

    public double getIndexerVelocity() {
        return indexer.getVelocity();
    }

    /** Velocity PIDF gains active on the flywheel's hub port (read at init). */
    public PIDFCoefficients getFlywheelPidf() {
        return flywheelPidf;
    }

    public double getFlywheelRpm() {
        return flywheel.getVelocity() * 60.0 / Constants.FLYWHEEL_TICKS_PER_REV;
    }

    public boolean isRedAlliance() {
        return !allianceButton.getState();
    }

    /** Raw digital state of indexBeam1 (true = HIGH). Which state means "beam broken" is not yet confirmed. */
    public boolean getIndexBeam1State() {
        return indexBeam1.getState();
    }

    public double getHeadingOffsetRadians() {
        return isRedAlliance()
                ? Constants.RED_START_HEADING_OFFSET_RAD : Constants.BLUE_START_HEADING_OFFSET_RAD;
    }

    public void fieldCentricDrive(double x, double y, double turn, double heading, double speed) {
        double rotX = (x * Math.cos(-heading) - y * Math.sin(-heading))
                * Constants.STRAFE_CORRECTION;
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double scale = Math.max(0, Math.min(speed, 1));

        leftFront.setPower((rotY + rotX + turn) / denominator * scale);
        leftBack.setPower((rotY - rotX + turn) / denominator * scale);
        rightFront.setPower((rotY - rotX - turn) / denominator * scale);
        rightBack.setPower((rotY + rotX - turn) / denominator * scale);
    }

    /** Also safe to call if initialization stopped partway through. */
    public void stopDrive() {
        if (leftFront != null) leftFront.setPower(0);
        if (rightFront != null) rightFront.setPower(0);
        if (leftBack != null) leftBack.setPower(0);
        if (rightBack != null) rightBack.setPower(0);
    }

    /**
     * Stops intake, indexer and flywheel and relaxes the servos (as the servo test OpModes do);
     * safe to call if initialization stopped partway.
     */
    public void stopMechanisms() {
        if (intakeLeft != null) intakeLeft.setPower(0);
        if (intakeRight != null) intakeRight.setPower(0);
        if (indexer != null) indexer.setPower(0);
        if (flywheel != null) flywheel.setPower(0);
        if (flowerWedgeLeft != null) flowerWedgeLeft.setPwmDisable();
        if (flowerWedgeRight != null) flowerWedgeRight.setPwmDisable();
        if (turret != null) turret.setPwmDisable();
    }
}
