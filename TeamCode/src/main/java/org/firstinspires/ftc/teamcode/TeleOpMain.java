package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.CommandOpMode;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSystem;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_Drive;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_IntakeStart;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_IntakeStop;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_NoShoot;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_PickUp;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_Shoot;

@TeleOp(name = "CommandBaseTest")
public class TeleOpMain extends CommandOpMode {

    public double pwrSelect;

    private Motor fL, bL, fR, bR, intake;
    private MotorEx shot, pickup;

    private DriveSystem mecDrive;
    private Com_Drive driveCommand;
    //Shooter subsystem and commands initialization
    private ShooterSystem shooterSystem;
    private Com_Shoot shootCommand;
    private Com_NoShoot stopCommand;
    //Wobble goal pickup subsystem and command initialization
    private WobbleSystem wobbleSystem;
    private Com_PickUp pickupCommand;
    //Intake subsystem and commands initialization
    private IntakeSystem intakeSystem;
    private Com_IntakeStart startIntakeCommand;
    private Com_IntakeStop stopIntakeCommand;

    public GamepadEx m_driverOp, m_toolOp;
    private Button shooterStart, shooterStop, dpadUp, dpadDown, goalLift, toggleIntake;

    @Override
    public void initialize() {
        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");

        shot = new MotorEx(hardwareMap, "shot", Motor.GoBILDA.BARE);
        pickup = new MotorEx(hardwareMap, "wobble", Motor.GoBILDA.BARE);
        shot.setRunMode(Motor.RunMode.VelocityControl);
        pickup.setRunMode(Motor.RunMode.PositionControl);

        mecDrive = new DriveSystem(fL, fR, bL, bR);

        m_driverOp = new GamepadEx(gamepad1);
        m_toolOp = new GamepadEx(gamepad2);

        dpadDown = new GamepadButton(m_driverOp, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    if (pwrSelect < 0.05) {
                        pwrSelect = 1;
                    } else {
                        pwrSelect -= 0.25;
                    }
                }));
        dpadUp = new GamepadButton(m_driverOp, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    if (pwrSelect > 0.95) {
                        pwrSelect = 0;
                    } else {
                        pwrSelect += 0.25;
                    }
                }));

        driveCommand = new Com_Drive(mecDrive, m_driverOp::getLeftX, m_driverOp::getLeftY, m_driverOp::getRightX);

        //IMPORTANT: Note to self remember in the Drive System class I just flipped the turn speed and strafe speed
        shooterSystem = new ShooterSystem(shot, telemetry, () -> pwrSelect);
        shootCommand = new Com_Shoot(shooterSystem);
        stopCommand = new Com_NoShoot(shooterSystem);
        shooterStart = (new GamepadButton(m_driverOp, GamepadKeys.Button.A))
                .whenPressed(shootCommand);
        shooterStop = (new GamepadButton(m_driverOp, GamepadKeys.Button.B))
                .whenPressed(stopCommand);

        wobbleSystem = new WobbleSystem(pickup);
        pickupCommand = new Com_PickUp(wobbleSystem);
        goalLift = (new GamepadButton(m_driverOp, GamepadKeys.Button.Y))
                .whenPressed(pickupCommand);

        intakeSystem = new IntakeSystem(intake);
        startIntakeCommand = new Com_IntakeStart(intakeSystem);
        stopIntakeCommand = new Com_IntakeStop(intakeSystem);
        toggleIntake = intakeSystem.intakeActive ? new GamepadButton(m_driverOp, GamepadKeys.Button.X)
                .whenPressed(stopIntakeCommand) : new GamepadButton(m_driverOp, GamepadKeys.Button.X)
                .whenPressed(startIntakeCommand);

        mecDrive.setDefaultCommand(driveCommand);

        register(mecDrive, shooterSystem, wobbleSystem, intakeSystem);

        schedule(driveCommand);
    }
}


