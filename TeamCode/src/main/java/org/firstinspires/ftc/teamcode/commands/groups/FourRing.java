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

    public static double traj1X = 44.0, traj1Y = -54.0;
    public static double traj2X = -27.0, traj2Y = -22.0, traj2H = 190.0;
    public static double traj3D = 14.0;
    public static double traj4X = 0.0, traj4Y = 21.0;

    private Pose2d startPose = new Pose2d(-63.0, -40.0, Math.toRadians(180.0));

    public FourRing(MecanumDriveSubsystem drive, WobbleSubsystem wobbleSystem, ShooterSubsystem shooter,
                    IntakeSubsystem intakeSystem){
        drive.setPoseEstimate(startPose);

        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .back(1.0)
                .build();

        Trajectory trajHalf = drive.trajectoryBuilder(traj0.end())
                .strafeLeft(16)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(trajHalf.end())
                .back(2.0)
                .splineToConstantHeading(new Vector2d(traj1X, traj1Y),0.0)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineToSplineHeading(new Pose2d(-5.0, -20.0, Math.toRadians(traj2H)), 0.0)
                .splineToConstantHeading(new Vector2d(traj2X, traj2Y), 0.0)
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(traj3D)
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .forward(traj4Y)
                .build();


        addCommands(
                new TrajectoryFollowerCommand(drive, traj0),
                new ParallelDeadlineGroup(
                        new TrajectoryFollowerCommand(drive, trajHalf),
                        new Com_PutDown(wobbleSystem)
                ),
                new TrajectoryFollowerCommand(drive, traj1),
                new InstantCommand(wobbleSystem::openGrabber, wobbleSystem),
                new WaitCommand(500),
                new ParallelDeadlineGroup(
                        new TrajectoryFollowerCommand(drive, traj2),
                        new Com_PickUp(wobbleSystem)
                ),
                new RapidFireCommand(shooter),
                new TrajectoryFollowerCommand(drive, traj3)
//                new TrajectoryFollowerCommand(drive, traj4)
        );
    }
}