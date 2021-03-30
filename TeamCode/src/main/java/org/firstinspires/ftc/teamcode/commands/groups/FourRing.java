package org.firstinspires.ftc.teamcode.commands.groups;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.Com_PickUp;
import org.firstinspires.ftc.teamcode.commands.Com_PutDown;
import org.firstinspires.ftc.teamcode.commands.RapidFireCommand;
import org.firstinspires.ftc.teamcode.commands.rr.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

@Config
public class FourRing extends SequentialCommandGroup {

    public static double boxTwoX = 45.0, boxTwoY = 0.0;
    public static double finalX = -8.8, finalY = -1.0;

    private Pose2d startPose = new Pose2d(-63.0, -40.0, Math.toRadians(180.0));

    public FourRing(MecanumDriveSubsystem drive, WobbleSubsystem wobbleSystem, ShooterSubsystem shooter,
                    IntakeSubsystem intakeSystem){
        drive.setPoseEstimate(startPose);
        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .strafeLeft(14.5)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .lineToLinearHeading(new Pose2d(-20.0, -51.0, Math.toRadians(20.0)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), traj1.end().getHeading())
                .splineToLinearHeading(new Pose2d(-20.0, -62.0, Math.toRadians(90.0)), 0.0)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), traj2.end().getHeading())
                .lineToConstantHeading(new Vector2d(-22.0, -42.0))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .strafeRight(10.0)
                .build();

        addCommands(
                new TrajectoryFollowerCommand(drive, traj0),
                new TrajectoryFollowerCommand(drive, traj1),
                new RapidFireCommand(shooter),
                new TrajectoryFollowerCommand(drive, traj2),
                new ParallelDeadlineGroup(
                    new TrajectoryFollowerCommand(drive, traj3),
                    new InstantCommand(intakeSystem::start)
                ),
                new TrajectoryFollowerCommand(drive, traj4)
        );
    }
}