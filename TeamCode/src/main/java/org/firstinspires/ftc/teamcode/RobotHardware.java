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
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/** Drive configuration carried over from 2025-27248 RobotHardware and PedroConstants. */
public class RobotHardware {
    public GoBildaPinpointDriver pinpoint;
    /** goBILDA Prism status lights (does nothing if "prism" isn't configured). */
    public RobotLights lights;
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
    /** Last flywheel setpoint passed to setFlywheelRpm (0 = off). */
    private double flywheelTargetRpm = 0;
    private DigitalChannel allianceButton;
    private DigitalChannel indexBeam1;
    private ServoImplEx flowerWedgeLeft;
    private ServoImplEx flowerWedgeRight;
    private ServoImplEx turret;

    // Drive ramp state: last commanded power per wheel (LF, LB, RF, RB) and when it was sent.
    private final double[] lastDrivePowers = new double[4];
    private long lastDriveNanos = 0;

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
        // setVelocity() in RUN_USING_ENCODER runs the hub's closed-loop velocity PIDF. Use the
        // gains from Launcher Tuning if enabled, then read the active gains for the Driver Station.
        if (Constants.FLYWHEEL_USE_TUNED_PIDF) {
            flywheel.setVelocityPIDFCoefficients(Constants.FLYWHEEL_PIDF_P,
                    Constants.FLYWHEEL_PIDF_I, Constants.FLYWHEEL_PIDF_D, Constants.FLYWHEEL_PIDF_F);
        }
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
        configureTurretServo(turret);

        allianceButton = hardwareMap.get(DigitalChannel.class, "allianceButton");
        allianceButton.setMode(DigitalChannel.Mode.INPUT);

        indexBeam1 = hardwareMap.get(DigitalChannel.class, "indexBeam1");
        indexBeam1.setMode(DigitalChannel.Mode.INPUT);

        // Status lights: yellow from power-on until here, then the alliance color.
        lights = new RobotLights(hardwareMap);
        lights.showAlliance(isRedAlliance());

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(Constants.FORWARD_POD_Y, Constants.STRAFE_POD_X, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        // Keep the robot stationary during initialization while the IMU calibrates.
        pinpoint.resetPosAndIMU();

        // Starts on the alliance's default pipeline: 8 = all red tags, 9 = all blue tags.
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

    /**
     * True when the indexer may run forward: the flywheel is commanded on and measures at least
     * INDEXER_MIN_FLYWHEEL_RPM.
     */
    public boolean isIndexerForwardAllowed() {
        return flywheelTargetRpm > 0 && getFlywheelRpm() >= Constants.INDEXER_MIN_FLYWHEEL_RPM;
    }

    /**
     * Positive power runs the indexer forward; clipped to [-1, 1]. Forward power is replaced
     * with 0 unless isIndexerForwardAllowed(); reverse is always allowed.
     */
    public void setIndexerPower(double power) {
        double p = clip(power, -1, 1);
        if (p > 0 && !isIndexerForwardAllowed()) p = 0;
        indexer.setPower(p);
    }

    /**
     * Closed-loop indexer speed in RPM (forward), clipped to [0, INDEXER_MAX_RPM]. Stops the
     * indexer instead unless isIndexerForwardAllowed().
     */
    public void setIndexerRpm(double rpm) {
        double target = clip(rpm, 0, Constants.INDEXER_MAX_RPM);
        if (target > 0 && !isIndexerForwardAllowed()) target = 0;
        if (target == 0) {
            indexer.setPower(0);
        } else {
            indexer.setVelocity(target * Constants.INDEXER_TICKS_PER_REV / 60.0);
        }
    }

    public double getIndexerRpm() {
        return indexer.getVelocity() * 60.0 / Constants.INDEXER_TICKS_PER_REV;
    }

    /** Closed-loop flywheel speed in motor/flywheel RPM, clipped to [0, FLYWHEEL_MAX_RPM]. */
    public void setFlywheelRpm(double rpm) {
        double target = clip(rpm, 0, Constants.FLYWHEEL_MAX_RPM);
        flywheelTargetRpm = target;
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

    /**
     * Direction and full-range PWM (TURRET_PWM_MIN_US..MAX_US) for the turret's Axon servo.
     * Used by every OpMode that drives the turret so a given position means the same angle
     * whether the port is configured as "Servo" or "Full Range Servo". Does not move the servo.
     */
    public static void configureTurretServo(ServoImplEx servo) {
        servo.setDirection(Servo.Direction.FORWARD);
        servo.setPwmRange(new PwmControl.PwmRange(
                Constants.TURRET_PWM_MIN_US, Constants.TURRET_PWM_MAX_US));
    }

    /** Turret servo position, clipped to [TURRET_MIN_POSITION, TURRET_MAX_POSITION]. */
    public void setTurretPosition(double position) {
        turret.setPwmEnable();
        turret.setPosition(clip(position,
                Constants.TURRET_MIN_POSITION, Constants.TURRET_MAX_POSITION));
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

    /** True when indexBeam1 reads INDEX_BEAM_BALL_STATE, i.e. a ball is blocking the beam. */
    public boolean isBallAtIndexer() {
        return indexBeam1.getState() == Constants.INDEX_BEAM_BALL_STATE;
    }

    /**
     * Makes the robot's current facing the new 0 heading (field-centric "forward") while keeping
     * the Pinpoint's X/Y. Uses the pose from the last pinpoint.update().
     */
    public void resetHeading() {
        Pose2D pose = pinpoint.getPosition();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,
                pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH),
                AngleUnit.RADIANS, 0));
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

        double[] targets = {
                (rotY + rotX + turn) / denominator * scale,
                (rotY - rotX + turn) / denominator * scale,
                (rotY - rotX - turn) / denominator * scale,
                (rotY + rotX - turn) / denominator * scale,
        };

        // Seconds since the last drive command. Capped so a long pause between calls
        // can't allow a jump to full power; the first call after a stop allows no increase.
        long now = System.nanoTime();
        double dt = lastDriveNanos == 0 ? 0 : Math.min((now - lastDriveNanos) / 1e9, 0.1);
        lastDriveNanos = now;
        double maxIncrease = Constants.DRIVE_RAMP_UP_SECONDS > 0
                ? dt / Constants.DRIVE_RAMP_UP_SECONDS : Double.POSITIVE_INFINITY;

        for (int i = 0; i < targets.length; i++) {
            lastDrivePowers[i] = rampPower(lastDrivePowers[i], targets[i], maxIncrease);
        }
        leftFront.setPower(lastDrivePowers[0]);
        leftBack.setPower(lastDrivePowers[1]);
        rightFront.setPower(lastDrivePowers[2]);
        rightBack.setPower(lastDrivePowers[3]);
    }

    /**
     * Limits how fast a wheel's power magnitude can grow. Reducing power (including stopping)
     * is immediate; a direction reversal drops to 0 first and then ramps up the other way.
     */
    private static double rampPower(double current, double target, double maxIncrease) {
        if (current != 0 && Math.signum(target) != Math.signum(current)) return 0;
        if (Math.abs(target) <= Math.abs(current)) return target;
        double increase = Math.min(Math.abs(target) - Math.abs(current), maxIncrease);
        return Math.signum(target) * (Math.abs(current) + increase);
    }

    /** Also safe to call if initialization stopped partway through. Resets the drive ramp. */
    public void stopDrive() {
        if (leftFront != null) leftFront.setPower(0);
        if (rightFront != null) rightFront.setPower(0);
        if (leftBack != null) leftBack.setPower(0);
        if (rightBack != null) rightBack.setPower(0);
        java.util.Arrays.fill(lastDrivePowers, 0);
        lastDriveNanos = 0;
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
        // Flywheel is off now: back to the yellow idle lights (don't leave snakes showing).
        if (lights != null) lights.show(RobotLights.State.YELLOW_IDLE);
    }
}
