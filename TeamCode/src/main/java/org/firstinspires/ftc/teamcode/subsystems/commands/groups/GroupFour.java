package org.firstinspires.ftc.teamcode.subsystems.commands.groups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSystem;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_PutDown;
import org.firstinspires.ftc.teamcode.subsystems.commands.drive.Com_DriveTime;
import org.firstinspires.ftc.teamcode.subsystems.commands.drive.Com_Rotate;

public class GroupFour extends SequentialCommandGroup {
    public GroupFour(DriveSystem drive, ElapsedTime time, VoltageSensor voltageSensor, RevIMU imu, WobbleSystem wobbleSystem) {
        addCommands(
                new Com_DriveTime(drive,0.5, 0D, 0D, time, 2.0),
                new Com_DriveTime(drive, (13/voltageSensor.getVoltage())*-0.55, 0D, 0D, time, 3.4),
                new Com_DriveTime(drive, 0D, (13/voltageSensor.getVoltage())*-0.55, 0D, time, 4.1
                ),
                new Com_Rotate(drive, imu, 180),
                new Com_PutDown(wobbleSystem, time),
                new Com_DriveTime(drive, 0D, (13/voltageSensor.getVoltage())*0.55, 0D, time, 3)
        );
    }
}