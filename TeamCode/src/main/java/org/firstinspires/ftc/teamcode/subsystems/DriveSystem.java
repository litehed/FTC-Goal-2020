package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSystem extends SubsystemBase {
    private MecanumDrive drive;
    private MotorEx fL, bL, fR, bR;

    public DriveSystem(MotorEx frontL, MotorEx frontR, MotorEx backL, MotorEx backR){
        fL = frontL;
        fR = frontR;
        bL = backL;
        bR = backR;

        drive = new MecanumDrive(fL, fR, bL, bR);
    }

    public DriveSystem(HardwareMap hMap, String fLName, String fRName, String bLName, String bRName){
        this(new MotorEx(hMap, fLName), new MotorEx(hMap, fRName), new MotorEx(hMap, bLName), new MotorEx(hMap, bRName));
    }

    //Strafe Speed, Forward Speed, and Turn Speed
    public void drive(double strfSpd, double fSpd, double trnSpd){
        drive.driveRobotCentric(strfSpd, fSpd, trnSpd);
    }
}
