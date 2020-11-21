package org.firstinspires.ftc.teamcode.subsystems.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.WobbleSystem;

public class Com_PutDown extends CommandBase {

    private final WobbleSystem wobblySystem;
    private final ElapsedTime time;

    public Com_PutDown(WobbleSystem subby, ElapsedTime elapsedTime){
        wobblySystem = subby;
        time = elapsedTime;
        addRequirements(subby);
    }
    @Override
    public void initialize(){
        time.reset();
    }
    @Override
    public void execute(){
        wobblySystem.motorDown();
    }
    @Override
    public void end(boolean interrupted){
        wobblySystem.motorStop();
        wobblySystem.putMeDownUwU();
    }

    @Override
    public boolean isFinished(){
        return time.seconds() >= 1;
    }
}

