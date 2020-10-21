package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_Drive;

@TeleOp(name = "CommandBaseTest")
public class TeleOpMain extends CommandOpMode {

    private MotorEx fL, bL, fR, bR;

    private DriveSystem mecDrive;
    private Com_Drive driveCommand;

    @Override
    public void initialize() {
        fL = new MotorEx(hardwareMap, "fL");
        fR = new MotorEx(hardwareMap, "fR");
        bL = new MotorEx(hardwareMap, "bL");
        bR = new MotorEx(hardwareMap, "bR");

        mecDrive = new DriveSystem(fL, fR, bL, bR);

        driveCommand = new Com_Drive(mecDrive, ()->driverOp.getLeftX(), ()->driverOp.getLeftY(), ()->driverOp.getRightX());

        mecDrive.setDefaultCommand(driveCommand);

        register(mecDrive);
    }
}


