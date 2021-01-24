package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.drive.Com_Drive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.TimedAction;

@TeleOp
public class ShooterTesting extends CommandOpMode {

    private Motor fL, fR, bL, bR;
    private Motor flyWheel;
    private SimpleServo flicker;

    private DriveSystem driveSystem;
    private Com_Drive driveCommand;

    private ShooterSubsystem shooterSystem;

    private GamepadEx m_driverOp;
    private TimedAction flickerAction;
    public double mult = 1.0;

    @Override
    public void initialize() {
        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");

        flyWheel = new Motor(hardwareMap, "shoot");
        flicker = new SimpleServo(hardwareMap, "flicker", 0, 270);

        m_driverOp = new GamepadEx(gamepad1);

        flickerAction = new TimedAction(
                ()-> flicker.setPosition(0.5),
                ()-> flicker.setPosition(0.27),
                600,
                true
        );

        driveSystem = new DriveSystem(fL, fR, bL, bR);
        driveCommand = new Com_Drive(driveSystem, m_driverOp::getLeftX, m_driverOp::getLeftY,
                m_driverOp::getRightX, ()->mult);
        shooterSystem = new ShooterSubsystem(flyWheel, flicker, flickerAction, telemetry);

        m_driverOp.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                ()->{
                    if(!flickerAction.running()) {
                        flickerAction.reset();
                    }
                    flickerAction.run();
                }
        );

        m_driverOp.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(
                ()->shooterSystem.shoot(), ()->shooterSystem.stop());

        m_driverOp.getGamepadButton(GamepadKeys.Button.Y)
                .toggleWhenPressed(()->mult = 0.5, ()->mult = 1.0);

        register(driveSystem);
        driveSystem.setDefaultCommand(driveCommand);
    }
}
