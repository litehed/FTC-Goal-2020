package org.firstinspires.ftc.teamcode.subsystems.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WobbleSystem;

public class Com_PutDown extends CommandBase {

    private final WobbleSystem wobblySystem;

    public Com_PutDown(WobbleSystem subby){
        wobblySystem = subby;
        addRequirements(subby);
    }
    @Override
    public void execute(){
        wobblySystem.putMeDownUwU();
    }
}

