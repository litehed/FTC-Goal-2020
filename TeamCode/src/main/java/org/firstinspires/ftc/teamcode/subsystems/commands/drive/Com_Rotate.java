package org.firstinspires.ftc.teamcode.subsystems.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;

public class Com_Rotate extends CommandBase {
    private final DriveSystem driveSystem;
    private final int degrees;
    public Com_Rotate(DriveSystem subby, int degreesIn){
        driveSystem = subby;
        degrees = degreesIn;

        addRequirements(subby);

    }
}
