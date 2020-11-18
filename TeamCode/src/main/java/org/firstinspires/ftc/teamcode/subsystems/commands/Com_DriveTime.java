package org.firstinspires.ftc.teamcode.subsystems.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;

public class Com_DriveTime extends CommandBase {

    private final DriveSystem driveSystem;
    private final Double strafe, forward, turn;
    private final ElapsedTime timey;
    private final Double timeLength;

    public Com_DriveTime(DriveSystem subsystem, Double strafeSpeed,
                         Double forwardSpeed, Double turnSpeed, ElapsedTime time, Double amnt){
        driveSystem = subsystem;
        strafe = strafeSpeed;
        forward = forwardSpeed;
        turn = turnSpeed;
        timey = time;
        timeLength = amnt;

        addRequirements(subsystem);
    }
    @Override
    public void initialize(){
        timey.reset();
    }
    @Override
    public void execute(){
        driveSystem.drive(strafe, forward, turn);
    }
    @Override
    public void end(boolean interrupted){
        driveSystem.halt();
    }

    @Override
    public boolean isFinished(){
        return timey.seconds() >= timeLength;
    }
}
