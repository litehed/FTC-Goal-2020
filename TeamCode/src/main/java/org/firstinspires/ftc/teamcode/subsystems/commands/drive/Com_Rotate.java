package org.firstinspires.ftc.teamcode.subsystems.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;

public class Com_Rotate extends CommandBase {
    private final DriveSystem driveSystem;
    private final ElapsedTime timer;
    private final int degrees;
    //Degrees per second
    private final double DPS = 45; //placeholder value until I can test
    private double timeNeeded;

    public Com_Rotate(DriveSystem subby, int degreesIn, ElapsedTime elapsedTime){
        driveSystem = subby;
        degrees = degreesIn;
        timer = elapsedTime;

        timeNeeded = degrees / DPS;
        addRequirements(subby);

    }
    @Override
    public void initialize(){
        timer.reset();
    }
    @Override
    public void execute() {
        if(timeNeeded >= 0)
            driveSystem.drive(0, 0, 0.5);
        else
            driveSystem.drive(0, 0, -0.5);
    }
    @Override
    public void end(boolean interrupted){
        driveSystem.halt();
    }
    @Override
    public boolean isFinished(){
        return timer.seconds() >= Math.abs(timeNeeded);
    }
}

