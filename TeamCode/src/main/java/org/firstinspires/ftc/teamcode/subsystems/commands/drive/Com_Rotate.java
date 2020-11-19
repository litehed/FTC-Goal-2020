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
        //Question to Jackson I did this because I dont know what the imus max value is so I just
        //said if it exceeds 360 subtract 360, does it have a max value is this needed I dont know
        degrees = (degreesIn + imu.getHeading()) >= 360 ? (degreesIn + imu.getHeading())-360 : degreesIn + imu.getHeading();
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

