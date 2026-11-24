# Main 2026 TeleOp

Select **Main 2026** on the Driver Station. Gamepad 1 (driver) left stick controls
field-centric translation; right stick X controls rotation. Click the left stick
to toggle between full and half speed. Press **Back/Share** to make the robot's
current facing the new field-centric forward (X/Y are kept).

Gamepad 2 (operator) runs the mechanisms:

| Control | Action |
| --- | --- |
| RT / LT | Intake forward / reverse (proportional). RT also runs the indexer until a ball reaches the beam |
| Hold A | **Shoot**: feed one ball at a time (flywheel must be on) |
| D-pad right / left | Indexer feed speed +/-5% (default 70%; loading stays at 40%) |
| X | Flywheel on/off (off also cancels shoot until A is released) |
| B | Turret tracking on/off |
| D-pad up / down | Flywheel setpoint +/-100 RPM |
| Y | Flower wedge up/down |
| Hold RB / LB | **Emergency**: intake and indexer full forward / reverse |
| Right stick X | Manual turret while tracking is on |

The flywheel and turret tracking only turn on from the buttons (a ball at the beam no longer
turns them on).

**Loading:** while the intake runs forward, the indexer runs slowly
(`INDEXER_LOAD_SPEED_PERCENT`, 40%) until `indexBeam1` sees a ball, then stops with the ball
staged at the beam. The intake keeps following RT.

**Shoot sequence (hold A, `ShootSequence`):** wait until the flywheel is back within
`SHOOT_READY_TOLERANCE_RPM` of its setpoint, feed the staged ball with the intake stopped
until the shot is detected (flywheel dip), then run the intake and indexer until the next ball
reaches the beam, and repeat. If no shot follows within `INDEXER_STUCK_TIMEOUT_SEC`, the
indexer reverses until the ball is back at the beam, then pushes at 100% until it shoots
(up to `INDEXER_MAX_UNJAMS_PER_BALL` tries, then STUCK until A is released).

The turret auto-aims at the selected alliance pipeline's AprilTag target using the
turret-mounted Limelight (`tx`) plus Pinpoint heading changes (`TurretTracker`). When the
target passes the end of the turret's travel, the turret swings around to the other end.
While tracking is off the turret stays at `TURRET_HOLD_POSITION`; it starts off unless
`TURRET_TRACKING_ON_AT_START` is true. Measured turret positions (full-range PWM): 0.00 = 90 deg left
(counter-clockwise limit), 0.23 = forward toward the intake (start/hold), 0.45 = 90 deg right,
0.67 = rear, 0.87 = clockwise limit. Configure tag grouping or a priority tag ID in the Limelight web
interface; the code uses the pipeline's `tx`.

**Single Gamepad Test** (Test group) keeps the previous layout with every
control on gamepad 1.

Configure these exact hardware names on the Robot Controller:

| Name | Device / setting |
| --- | --- |
| `leftFront`, `leftBack` | Drive motors, forward direction |
| `rightFront`, `rightBack` | Drive motors, reversed direction |
| `pinpoint` | goBILDA Pinpoint odometry computer |
| `allianceButton` | Digital input; low = red, high = blue |
| `limelight` | Limelight 3A (Ethernet Device after a robot configuration scan) |

All drive motors use `RUN_USING_ENCODER` and brake at zero power.
Pinpoint uses goBILDA 4-bar pods, forward encoder FORWARD, strafe encoder REVERSED.
The forward pod sideways offset is **-3.375 inches**; the strafe pod forward
offset is **5.5625 inches**. These are passed to `setOffsets` in that order.

Initialization resets position and calibrates the Pinpoint IMU: keep the robot
stationary. Start with the same sideways orientation as last year: blue robot
front points right, red robot front points left relative to the drivers. The
alliance input applies +90 degrees for blue or -90 degrees for red.

Configuration and drive math were reviewed against `RobotHardware.java`,
`Competition.java`, `Testing/FieldCentricDriveTest.java`, and
`pedroPathing/PedroConstants.java` in FRC1610/2025-27248 at commit
`9e5de4d3e616af825a30111d2c6bedba205cbdc1`. The current FTC SDK's built-in
Pinpoint driver is used, with no additional libraries required.

Motor powers are normalized to [-1, 1], and all drive motors stop when the
OpMode exits or Pinpoint is not ready with a finite heading.
Wiring, wheel directions, pod tracking, and
field-centric controls still require verification on the physical robot.

## goBILDA Prism status lights

The Prism RGB LED driver connects to an I2C port. In **Configure Robot**, add it on that
port's I2C bus as **goBILDA® Prism RGB LED Driver** named `prism`. The driver and
**Prism Configurator** come from goBILDA's
[FtcRobotController-Add-Prism](https://github.com/goBILDA-Official/FtcRobotController-Add-Prism/tree/Add-Prism/TeamCode/src/main/java/org/firstinspires/ftc/teamcode)
(`TeamCode/.../Prism/`, `GoBildaPrismConfigurator.java`).

Run **Prism Light Setup** (Test group) once and press A: it sets the strip length to 48 LEDs,
saves these Artboards to the Prism and makes Artboard 0 play at power-on. They persist across
power cycles. All animations are snakes (speed 0.3, 4 LEDs apart) at 50% brightness on
LEDs 0-47.

| Artboard | Lights | When |
| --- | --- | --- |
| 0 | Yellow snakes | Power on (played by the Prism itself); during the match with no ball loaded and the flywheel off |
| 1 / 2 | Red / blue snakes | During INIT, from `allianceButton`; during the match with no ball loaded and the flywheel running |
| 3 | Purple snakes | Ball at the indexer beam, flywheel off or not within 50 RPM of its setpoint |
| 4 | Green snakes | Ball at the indexer beam and flywheel within 50 RPM of its setpoint |

Main 2026 and Single Gamepad Test only switch Artboards when the state changes. Saving an
Artboard 0-4 with the Prism Configurator overwrites these; rerun Prism Light Setup to restore
them. Without a `prism` in the configuration, the lights code does nothing.

## Limelight 3A

Connect the camera's USB-C port to the Control Hub's blue USB 3.0 port using a
USB-C to USB-A cable. Scan in Driver Station **Configure Robot**, then name the
detected Ethernet Device `limelight` and save the configuration.

The code uses the existing FTC SDK Limelight driver, polls at 100 Hz, and starts
polling during initialization. Pipelines configured on the camera:

| Slot | Pipeline | Constant |
| --- | --- | --- |
| 0 | Red Scoring Side | `PIPELINE_RED_SCORING_SIDE` |
| 1 | Red Audience Side | `PIPELINE_RED_AUDIENCE_SIDE` |
| 2 | Blue Audience Side | `PIPELINE_BLUE_AUDIENCE_SIDE` |
| 3 | Blue Scoring Side | `PIPELINE_BLUE_SCORING_SIDE` |
| 8 | All Red tags (**red alliance default**) | `PIPELINE_ALL_RED_TAGS` |
| 9 | All Blue tags (**blue alliance default**) | `PIPELINE_ALL_BLUE_TAGS` |

`allianceButton` (low = red, high = blue) selects pipeline **8** or **9**, at INIT and
whenever the button changes; requests are sent only when the selection changes.
The camera must already have these pipelines configured in its web interface.

To configure the camera, connect it to a computer, open Limelight Hardware
Manager or `http://limelight.local:5801`, and set team number **27248**. Configure
the pipelines in the slots listed above. For AprilTag field-pose telemetry, load
the correct season's field map, enable **Full 3D**, and enter the measured camera
position and orientation relative to the robot. Mount measurements and target
settings must match the actual robot and field.

TeleOp displays requested and actual pipeline, result age in milliseconds,
target validity, tx/ty in degrees, target area, and the available field pose.
Result age remains visible because a disconnected camera can leave an old
result cached. Vision data is displayed only; driving continues to use Pinpoint.
Limelight polling stops in the OpMode's termination cleanup, including an early
stop or initialization failure. Physical connectivity and detection have not
been verified on hardware.

References:
- [Limelight 3A quick-start](https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3a)
- [FTC programming guide](https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming)
- [Last year's RobotHardware](https://github.com/FRC1610/2025-27248/blob/9e5de4d3e616af825a30111d2c6bedba205cbdc1/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotHardware.java)
