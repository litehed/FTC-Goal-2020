package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.CommandOpMode;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_Drive;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_NoShoot;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_Shoot;

@TeleOp(name = "CommandBaseTest")
public class TeleOpMain extends CommandOpMode {

    private Motor fL, bL, fR, bR;

    private DriveSystem mecDrive;
    private Com_Drive driveCommand;
    private Shooter shooter;
    private Com_Shoot shot;
    private Com_NoShoot stp;
    private GamepadEx m_driverOp;
    private Button pew, pow;

    @Override
    public void initialize() {
        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");

        mecDrive = new DriveSystem(fL, fR, bL, bR);

        m_driverOp = new GamepadEx(gamepad1);

        driveCommand = new Com_Drive(mecDrive, ()->m_driverOp.getLeftX(), ()->m_driverOp.getLeftY(), ()->m_driverOp.getRightX());
        shooter = new Shooter(hardwareMap, "shot");
        shot = new Com_Shoot(shooter);
        stp = new Com_NoShoot(shooter);

        pew = (new GamepadButton(m_driverOp, GamepadKeys.Button.A))
                .whenPressed(shot);
        pew = (new GamepadButton(m_driverOp, GamepadKeys.Button.B))
                .whenPressed(stp);

        mecDrive.setDefaultCommand(driveCommand);

        register(mecDrive, shooter);
        waitForStart();
    }
}


