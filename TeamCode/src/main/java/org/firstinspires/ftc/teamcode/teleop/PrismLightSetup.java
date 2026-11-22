package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.RobotLights;

/**
 * One-time setup for the goBILDA Prism ("prism"): saves the robot's light Artboards to the
 * Prism and makes Artboard 0 (yellow) play at power-on. The Prism keeps these across power
 * cycles, so this only needs to run again if the Artboards are changed or overwritten (for
 * example with the Prism Configurator).
 *
 * <p>Gamepad1 after START: A = save the Artboards; d-pad left/right = preview each one.
 */
@TeleOp(name = "Prism Light Setup", group = "Test")
public class PrismLightSetup extends LinearOpMode {

    private static final String[] NAMES = {
            "0: Power on / idle (yellow snakes)", "1: Red alliance (red snakes)",
            "2: Blue alliance (blue snakes)", "3: Ball loaded, spinning up (purple snakes)",
            "4: Ball loaded, at speed (green snakes)"};

    @Override
    public void runOpMode() throws InterruptedException {
        GoBildaPrismDriver prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

        telemetry.addData("Prism device ID (3 = OK)", prism.getDeviceID());
        telemetry.addData("Firmware", prism.getFirmwareVersionString());
        telemetry.addLine("After START: A = save robot Artboards 0-4 and set yellow as power-on.");
        telemetry.addLine("D-pad left/right = preview Artboards.");
        telemetry.addLine("This overwrites Artboards 0-4 on the Prism.");
        telemetry.update();

        waitForStart();

        int preview = 0;
        String status = "Press A to save the Artboards.";
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                telemetry.addLine("Saving Artboards...");
                telemetry.update();
                RobotLights.saveArtboards(prism);
                preview = 0;
                status = "Saved Artboards 0-4; yellow (0) plays at power-on.";
            }
            if (gamepad1.dpadRightWasPressed()) {
                preview = (preview + 1) % NAMES.length;
                prism.loadAnimationsFromArtboard(RobotLights.artboard(preview));
            }
            if (gamepad1.dpadLeftWasPressed()) {
                preview = (preview + NAMES.length - 1) % NAMES.length;
                prism.loadAnimationsFromArtboard(RobotLights.artboard(preview));
            }

            telemetry.addLine(status);
            telemetry.addData("Previewing", NAMES[preview]);
            telemetry.addLine("A: save   D-pad left/right: preview");
            telemetry.update();
            sleep(20);
        }
    }
}
