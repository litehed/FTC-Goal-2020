package org.firstinspires.ftc.teamcode.subsystems.commands.groups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystems.commands.drive.Com_DriveTime;

public class GroupZero extends SequentialCommandGroup {
    public GroupZero(DriveSystem drive, ElapsedTime time) {
        addCommands(
                new Com_DriveTime(drive, 0.5, 0D, 0D, time, 2.9),
                new Com_DriveTime(drive, 0D, 0.55, 0D, time, 3.0)
                );
    }
}
