# Mecanum Drive TeleOp

Select **Mecanum Drive** on the Driver Station. Gamepad 1's left stick controls
field-centric translation; right stick X controls rotation. Click the left stick
to toggle between full and half speed.

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
This OpMode requires no launcher, intake, turret, or lighting hardware.
Wiring, wheel directions, pod tracking, and
field-centric controls still require verification on the physical robot.

## Limelight 3A

Connect the camera's USB-C port to the Control Hub's blue USB 3.0 port using a
USB-C to USB-A cable. Scan in Driver Station **Configure Robot**, then name the
detected Ethernet Device `limelight` and save the configuration.

The code uses the existing FTC SDK Limelight driver, polls at 100 Hz, and starts
polling during initialization. As in last year's `RobotHardware.java`, blue uses
pipeline **0** and red uses pipeline **4**. Changing `allianceButton` selects the
corresponding pipeline; requests are sent only when the selection changes.
The camera must already have those pipelines configured in its web interface.

To configure the camera, connect it to a computer, open Limelight Hardware
Manager or `http://limelight.local:5801`, and set team number **27248**. Configure
the desired pipelines in slots 0 and 4. For AprilTag field-pose telemetry, load
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
