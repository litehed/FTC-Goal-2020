package org.firstinspires.ftc.teamcode.subsystems.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class Com_Shoot extends CommandBase{

    private final Shooter shooter;

    public Com_Shoot(Shooter subby){
        shooter = subby;
        addRequirements(shooter);
    }
    @Override
    public void initialize(){
        shooter.shot();
    }
    @Override
    public boolean isFinished(){
        return true;
    }
}
