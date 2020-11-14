package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.RingPushSystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSystem;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_Drive;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_IntakeStart;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_IntakeStop;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_NoShoot;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_PickUp;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_Push;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_Shoot;

@TeleOp(name="Kanye West")
public class TeleopShooter extends CommandOpMode {

    public double pwrSelect;

    private Motor fL, bL, fR, bR;
    private MotorEx shot;
    private ServoEx servo;

    private DriveSystem mecDrive;
    private Com_Drive driveCommand;
    //Shooter subsystem and commands initialization
    private ShooterSystem shooterSystem;
    private Com_Shoot shootCommand;
    private Com_NoShoot stopCommand;

    //TODO: Add servo stuff here
    private RingPushSystem pushSystem;
    private Com_Push pushCommand;

    public GamepadEx m_driverOp, m_toolOp;
    private Button toggleShooter, dpadUp, dpadDown, servoMove;

    @Override
    public void initialize() {
        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");

        servo = new SimpleServo(hardwareMap, "push");
        //one of our motors is messed up so it has to be inverted woooooo
        bL.setInverted(true);

        shot = new MotorEx(hardwareMap, "shot", Motor.GoBILDA.BARE);
        shot.setRunMode(Motor.RunMode.VelocityControl);
 
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
        toggleShooter = new GamepadButton(m_driverOp, GamepadKeys.Button.A)
                .whenPressed(new ConditionalCommand(shootCommand, stopCommand, shooterSystem::active))
                .whenReleased(new InstantCommand(shooterSystem::toggle));

        pushSystem = new RingPushSystem(servo);
        pushCommand = new Com_Push(pushSystem);

        servoMove = (new GamepadButton(m_driverOp, GamepadKeys.Button.B))
                .whenPressed(pushCommand);

        mecDrive.setDefaultCommand(driveCommand);

        register(mecDrive, shooterSystem, pushSystem);

        schedule(driveCommand);
    }
}


