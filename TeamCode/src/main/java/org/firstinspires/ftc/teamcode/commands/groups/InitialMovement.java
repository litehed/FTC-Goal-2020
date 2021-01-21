package org.firstinspires.ftc.teamcode.commands.groups;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.Com_PutDown;
import org.firstinspires.ftc.teamcode.commands.rr.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

public class InitialMovement extends SequentialCommandGroup{

    private Pose2d startPose = new Pose2d(-63.0, -40.0, Math.toRadians(180.0));
    private Pose2d secondPose = new Pose2d(-65.0, -51.0, Math.toRadians(0.0));

    public InitialMovement(MecanumDriveSubsystem drive, WobbleSubsystem wobbleSystem){
        drive.setPoseEstimate(startPose);
        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .strafeLeft(8)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .back(4.0)
                .splineToConstantHeading(new Vector2d(0.0, -60.0), 0.0)
                .build();

        Vector2d shootPose = traj1.end().vec().plus(new Vector2d(-12.0, 22.0));

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), true)
                .lineToConstantHeading(shootPose)
                .build();

        Vector2d secondWobble = traj2.end().vec().plus(new Vector2d(-30.0, 12.0));

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), 0.0)
                .splineToLinearHeading(new Pose2d(secondWobble, 0.0), Math.toRadians(-90.0))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end(), false)
                .splineToConstantHeading(traj3.end().vec().plus(new Vector2d(-3.0, 3.0)), 0.0)
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end(), 0.0)
                .splineTo(traj4.end().vec().plus(new Vector2d(16.0, 0.0)), 0.0)
                .splineToLinearHeading(traj1.end().plus(new Pose2d(-5.0, 0.0, Math.toRadians(180.0))), 0.0)
                .build();

        addCommands(
                new TrajectoryFollowerCommand(drive, traj0),
                new TrajectoryFollowerCommand(drive, traj1),
                new Com_PutDown(wobbleSystem),
                new InstantCommand(wobbleSystem::openGrabber, wobbleSystem),
                new WaitCommand(1000),
                new TrajectoryFollowerCommand(drive, traj2),
                new TrajectoryFollowerCommand(drive, traj3),
                new TrajectoryFollowerCommand(drive, traj4),
                new TrajectoryFollowerCommand(drive, traj5)
        );
    }
}