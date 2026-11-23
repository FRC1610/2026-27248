package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;

/**
 * Sets the Prism lights back to the yellow idle Artboard (PRISM_ARTBOARD_POWER_ON) after every
 * OpMode stops, including test OpModes that never touch the lights, so a "ball loaded" color
 * isn't left showing.
 *
 * <p>The SDK calls {@link #register} once when the Robot Controller's event loop is created; it
 * then gets a callback after each OpMode stops. Does nothing if no "prism" is configured. The
 * Prism Configurator is skipped so the Artboard just created stays visible.
 */
public class LightsResetOnStop implements OpModeManagerNotifier.Notifications {

    @OnCreateEventLoop
    public static void register(Context context, FtcEventLoop eventLoop) {
        eventLoop.getOpModeManager().registerListener(new LightsResetOnStop());
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {
        // Nothing: OpModes that use the lights set them themselves.
    }

    @Override
    public void onOpModePreStart(OpMode opMode) {
        // Nothing.
    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        if (opMode instanceof GoBildaPrismConfigurator) return;
        try {
            GoBildaPrismDriver prism = opMode.hardwareMap == null
                    ? null : opMode.hardwareMap.tryGet(GoBildaPrismDriver.class, "prism");
            if (prism != null) {
                prism.loadAnimationsFromArtboard(
                        RobotLights.artboard(Constants.PRISM_ARTBOARD_POWER_ON));
            }
        } catch (RuntimeException e) {
            // Never let a lights problem disturb the Robot Controller.
            RobotLog.ww("LightsResetOnStop", e, "Could not reset Prism lights");
        }
    }
}
