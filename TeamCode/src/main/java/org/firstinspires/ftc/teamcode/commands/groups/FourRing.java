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

    public static double traj1X = -20.0, traj1Y = -51.0, traj1H = 190.0;
    public static double traj2X = -24.0, traj2Y = -62.0, traj2H = 90.0;
    public static double traj3X = -26.0, traj3Y = -42.0;

    private Pose2d startPose = new Pose2d(-63.0, -40.0, Math.toRadians(180.0));

    public FourRing(MecanumDriveSubsystem drive, WobbleSubsystem wobbleSystem, ShooterSubsystem shooter,
                    IntakeSubsystem intakeSystem){
        drive.setPoseEstimate(startPose);

        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .back(1.0)
                .build();

        Trajectory trajHalf = drive.trajectoryBuilder(traj0.end())
                .strafeLeft(14.5)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(trajHalf.end())
                .lineToLinearHeading(new Pose2d(traj1X, traj1Y, Math.toRadians(traj1H)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), traj1.end().getHeading())
                .splineToLinearHeading(new Pose2d(traj2X, traj2Y, Math.toRadians(traj2H)), 0.0)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), traj2.end().getHeading())
                .lineToConstantHeading(new Vector2d(traj3X, traj3Y))
                .build();

//        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
//                .strafeRight(10.0)
//                .build();

        addCommands(
                new TrajectoryFollowerCommand(drive, traj0),
                new TrajectoryFollowerCommand(drive, traj1),
                new RapidFireCommand(shooter, 3),
                new TrajectoryFollowerCommand(drive, traj2),
                new ParallelDeadlineGroup(
                    new TrajectoryFollowerCommand(drive, traj3),
                    new InstantCommand(intakeSystem::start)
                )
        );
    }
}