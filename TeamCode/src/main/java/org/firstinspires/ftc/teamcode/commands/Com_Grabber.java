package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

public class Com_Grabber extends CommandBase {

    private WobbleSubsystem wobbleSystem;

    public Com_Grabber(WobbleSubsystem subsystem){
        wobbleSystem = subsystem;

        addRequirements(wobbleSystem);
    }

    @Override
    public void execute(){
        if(wobbleSystem.isGrabbing())
            wobbleSystem.openGrabber();
        else
            wobbleSystem.closeGrabber();
    }


}
