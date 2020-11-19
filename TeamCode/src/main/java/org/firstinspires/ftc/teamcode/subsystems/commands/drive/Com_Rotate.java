package org.firstinspires.ftc.teamcode.subsystems.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;

public class Com_Rotate extends CommandBase {
    private final DriveSystem driveSystem;
    private final RevIMU imu;
    private final double degrees;
    //TODO: I got lazy add p control for imu later
    //thx -past me
    public Com_Rotate(DriveSystem subby, RevIMU revIMU, int degreesIn){
        driveSystem = subby;
        imu = revIMU;
        degrees = degreesIn + imu.getRotation2d().getDegrees();
        addRequirements(subby);

    }
    @Override
    public void execute() {

            driveSystem.drive(0, 0, 0.5);
    }
    @Override
    public void end(boolean interrupted){
        driveSystem.halt();
    }
    @Override
    public boolean isFinished(){
        return imu.getHeading() == degrees;
    }
}

