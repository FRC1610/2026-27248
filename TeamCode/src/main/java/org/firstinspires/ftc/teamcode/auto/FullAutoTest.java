package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "FullAutoTest", group = "Autonomous")
public class FullAutoTest extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(34.9602, 11.0061, 90);
    private final Pose path1 = poseFactory.of(9.8838, 32.8185, 180);
    private final Pose path1Control1 = poseFactory.of(38.8144, 48.1494, 0);
    private final Pose point2 = poseFactory.of(34.8228, 106.8619, -108.6143);
    private final Pose point3Start = poseFactory.of(34.8228, 106.8619, 0);
    private final Pose point3 = poseFactory.of(107.6491, 105.929, 0);
    private final Pose point3Control1 = poseFactory.of(66.9963, 137.9102, 0);
    private final Pose point4 = poseFactory.of(107.2554, 32.2232, 89.694);
    private final Pose point5 = poseFactory.of(35.3656, 10.7187, -6.521);
    private final Pose point5Control1 = poseFactory.of(79.8907, 5.6292, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.curve(start, path1Control1, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(path1, point2).reverseTangent();
    }

    public Path path3() {
        return Paths.curve(point3Start, point3Control1, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.line(point3, point4).reverseTangent();
    }

    public Path path5() {
        return Paths.curve(point4, point5Control1, point5).reverseTangent();
    }
}
