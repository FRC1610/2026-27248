package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/** Drive configuration carried over from 2025-27248 RobotHardware and PedroConstants. */
public class RobotHardware {
    // Forward pod's sideways offset and strafe pod's forward offset, in inches.
    public static final double FORWARD_POD_Y = -3.375;
    public static final double STRAFE_POD_X = 5.5625;
    public static final int BLUE_VISION_PIPELINE = 0;
    public static final int RED_VISION_PIPELINE = 4;

    public GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    private LLResult latestLimelightResult;
    private int selectedVisionPipeline = -1;
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private DigitalChannel allianceButton;

    public void init(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        configureMotor(leftFront, DcMotorSimple.Direction.FORWARD);
        configureMotor(leftBack, DcMotorSimple.Direction.FORWARD);
        configureMotor(rightFront, DcMotorSimple.Direction.REVERSE);
        configureMotor(rightBack, DcMotorSimple.Direction.REVERSE);

        allianceButton = hardwareMap.get(DigitalChannel.class, "allianceButton");
        allianceButton.setMode(DigitalChannel.Mode.INPUT);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(FORWARD_POD_Y, STRAFE_POD_X, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        // Keep the robot stationary during initialization while the IMU calibrates.
        pinpoint.resetPosAndIMU();

        // Same hardware name and alliance pipelines as 2025-27248.
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        selectAllianceLimelightPipeline();
        limelight.start();
        refreshLimelightResult();
    }

    /** Pipeline switching is asynchronous; the result reports the actual pipeline. */
    public void selectAllianceLimelightPipeline() {
        int pipeline = isRedAlliance() ? RED_VISION_PIPELINE : BLUE_VISION_PIPELINE;
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
        motor.setPower(0);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isRedAlliance() {
        return !allianceButton.getState();
    }

    public double getHeadingOffsetRadians() {
        // Last year's sideways start: blue faces right, red faces left.
        return isRedAlliance() ? -Math.PI / 2.0 : Math.PI / 2.0;
    }

    public void fieldCentricDrive(double x, double y, double turn, double heading, double speed) {
        double rotX = (x * Math.cos(-heading) - y * Math.sin(-heading)) * 1.1;
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
}
