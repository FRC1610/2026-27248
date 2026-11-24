package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

/**
 * Robot status lights on the goBILDA Prism LED driver (I2C, configured as "prism").
 *
 * <p>The Artboards are saved on the Prism once by Prism Light Setup (see {@link #saveArtboards});
 * during a match this class only switches between them, and only sends an I2C write when the
 * state changes. If no "prism" is in the robot configuration, every call does nothing.
 * The snake animations run at PRISM_SNAKE_SPEED, PRISM_SNAKE_SPACING apart, at PRISM_BRIGHTNESS;
 * the police lights run at PRISM_SHOOTING_BRIGHTNESS. All cover LEDs 0 to PRISM_LED_COUNT - 1.
 *
 * <ul>
 *   <li>YELLOW_IDLE: yellow snakes - power-on (Artboard 0, played by the Prism itself), and
 *       during the match whenever no ball is loaded and the flywheel is off
 *   <li>ALLIANCE_RED / ALLIANCE_BLUE: red / blue snakes (from allianceToggle) during INIT,
 *       and during the match with no ball loaded while the flywheel is running
 *   <li>BALL_SPINUP: purple snakes - ball loaded, flywheel off or not within
 *       LIGHTS_FLYWHEEL_READY_TOLERANCE_RPM of its setpoint
 *   <li>BALL_READY: green snakes - ball loaded and flywheel at speed
 *   <li>SHOOTING: police lights - while the shoot command runs (overrides the others)
 * </ul>
 */
public class RobotLights {
    public enum State {
        YELLOW_IDLE(Constants.PRISM_ARTBOARD_POWER_ON),
        ALLIANCE_RED(Constants.PRISM_ARTBOARD_RED),
        ALLIANCE_BLUE(Constants.PRISM_ARTBOARD_BLUE),
        BALL_SPINUP(Constants.PRISM_ARTBOARD_BALL_SPINUP),
        BALL_READY(Constants.PRISM_ARTBOARD_BALL_READY),
        SHOOTING(Constants.PRISM_ARTBOARD_SHOOTING);

        public final int artboard;

        State(int artboard) {
            this.artboard = artboard;
        }
    }

    private final GoBildaPrismDriver prism;
    private State current = null;

    public RobotLights(HardwareMap hardwareMap) {
        prism = hardwareMap.tryGet(GoBildaPrismDriver.class, "prism");
    }

    /** True if a "prism" device is in the robot configuration. */
    public boolean isPresent() {
        return prism != null;
    }

    /** Last state sent (null before the first one). */
    public State getState() {
        return current;
    }

    /** Shows a state; only writes to the Prism when it differs from the last one shown. */
    public void show(State state) {
        if (prism == null || state == current) return;
        prism.loadAnimationsFromArtboard(artboard(state.artboard));
        current = state;
    }

    /** Alliance color from allianceToggle (used during INIT). */
    public void showAlliance(boolean redAlliance) {
        show(redAlliance ? State.ALLIANCE_RED : State.ALLIANCE_BLUE);
    }

    /**
     * Match-time lights:
     * <ul>
     *   <li>shoot command running: police lights
     *   <li>ball loaded: green snakes if the flywheel is on and within
     *       LIGHTS_FLYWHEEL_READY_TOLERANCE_RPM of its setpoint, otherwise purple snakes
     *   <li>no ball, flywheel running: alliance color
     *   <li>no ball, flywheel off: yellow
     * </ul>
     */
    public void update(boolean redAlliance, boolean shooting, boolean ballLoaded,
                       boolean flywheelOn, double flywheelSetpointRpm, double flywheelActualRpm) {
        if (shooting) {
            show(State.SHOOTING);
        } else if (ballLoaded) {
            show(isFlywheelAtSpeed(flywheelOn, flywheelSetpointRpm, flywheelActualRpm)
                    ? State.BALL_READY : State.BALL_SPINUP);
        } else if (flywheelOn) {
            showAlliance(redAlliance);
        } else {
            show(State.YELLOW_IDLE);
        }
    }

    public static boolean isFlywheelAtSpeed(boolean flywheelOn, double setpointRpm,
                                            double actualRpm) {
        return flywheelOn && setpointRpm > 0
                && Math.abs(actualRpm - setpointRpm) <= Constants.LIGHTS_FLYWHEEL_READY_TOLERANCE_RPM;
    }

    /**
     * Writes the six Artboards to the Prism's memory, sets the strip length to PRISM_LED_COUNT
     * and makes Artboard 0 (yellow) the power-on default. Run once (Prism Light Setup); the Prism
     * keeps them across power cycles.
     */
    public static void saveArtboards(GoBildaPrismDriver prism) throws InterruptedException {
        prism.setStripLength(Constants.PRISM_LED_COUNT);
        Thread.sleep(50);

        saveSnakes(prism, Constants.PRISM_ARTBOARD_POWER_ON, Color.YELLOW);
        saveSnakes(prism, Constants.PRISM_ARTBOARD_RED, Color.RED);
        saveSnakes(prism, Constants.PRISM_ARTBOARD_BLUE, Color.BLUE);
        saveSnakes(prism, Constants.PRISM_ARTBOARD_BALL_SPINUP, Color.PURPLE);
        saveSnakes(prism, Constants.PRISM_ARTBOARD_BALL_READY, Color.GREEN);
        PrismAnimations.PoliceLights police = new PrismAnimations.PoliceLights();
        save(prism, Constants.PRISM_ARTBOARD_SHOOTING, police,
                Constants.PRISM_SHOOTING_BRIGHTNESS);

        GoBildaPrismDriver.Artboard powerOn = artboard(Constants.PRISM_ARTBOARD_POWER_ON);
        prism.setDefaultBootArtboard(powerOn);
        prism.enableDefaultBootArtboard(true);
        Thread.sleep(50);
        prism.loadAnimationsFromArtboard(powerOn);
    }

    /**
     * Snakes of the color over an unlit background, PRISM_SNAKE_SPACING LEDs apart, at
     * PRISM_SNAKE_SPEED.
     */
    private static void saveSnakes(GoBildaPrismDriver prism, int artboard, Color color)
            throws InterruptedException {
        PrismAnimations.Snakes snakes = new PrismAnimations.Snakes(color);
        snakes.setBackgroundColor(Color.TRANSPARENT);
        snakes.setSpacingBetween(Constants.PRISM_SNAKE_SPACING);
        snakes.setSpeed(Constants.PRISM_SNAKE_SPEED);
        save(prism, artboard, snakes, Constants.PRISM_BRIGHTNESS);
    }

    /** Applies the brightness and shared LED range, then saves the animation to an Artboard. */
    private static void save(GoBildaPrismDriver prism, int artboard,
                             PrismAnimations.AnimationBase animation, int brightness)
            throws InterruptedException {
        animation.setBrightness(brightness);
        animation.setStartIndex(0);
        animation.setStopIndex(Constants.PRISM_LED_COUNT - 1);

        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, animation);
        Thread.sleep(50);
        prism.saveCurrentAnimationsToArtboard(artboard(artboard));
        Thread.sleep(100); // give the Prism time to store it before the next write
    }

    public static GoBildaPrismDriver.Artboard artboard(int index) {
        return GoBildaPrismDriver.Artboard.values()[index];
    }
}
