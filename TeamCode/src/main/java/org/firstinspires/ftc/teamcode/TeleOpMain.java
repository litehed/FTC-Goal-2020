package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.CommandOpMode;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_Drive;

@TeleOp(name = "CommandBaseTest")
public class TeleOpMain extends CommandOpMode {

    private Motor fL, bL, fR, bR;

    private DriveSystem mecDrive;
    private Com_Drive driveCommand;

    @Override
    public void initialize() {
        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");

        mecDrive = new DriveSystem(fL, fR, bL, bR);

        driveCommand = new Com_Drive(mecDrive, ()->driverOp.getLeftX(), ()->driverOp.getLeftY(), ()->driverOp.getRightX());

        mecDrive.setDefaultCommand(driveCommand);

        register(mecDrive);
    }
}


