package org.firstinspires.ftc.teamcode.commands.groups;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.rr.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class InitialMovement extends SequentialCommandGroup{

    private Pose2d startPose = new Pose2d(-65.0, -41.0, Math.toRadians(180));
    private TrajectoryFollowerCommand splineFollower;

    public InitialMovement(MecanumDriveSubsystem drive){
        Trajectory traj = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(10, -61), Math.toRadians(180))
                .build();
        addCommands(
                splineFollower = new TrajectoryFollowerCommand(drive, traj)
        );
    }
}
