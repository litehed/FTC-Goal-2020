package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSystem;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_IntakeStart;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_IntakeStop;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_NoShoot;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_PickUp;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_PutDown;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_Shoot;
import org.firstinspires.ftc.teamcode.subsystems.commands.drive.Com_Drive;

@TeleOp(name="Kanye South")
public class TeleopComp2 extends CommandOpMode {

    public double pwrSelect = 1.0;

    private Motor fL, bL, fR, bR;
    private Motor shot, intake;
    private SimpleServo servo;

    private DriveSystem mecDrive;
    private Com_Drive driveCommand;
    //Shooter subsystem and commands initialization
    private ShooterSystem shooterSystem;
    private Com_Shoot shootCommand;
    private Com_NoShoot stopCommand;

    private IntakeSystem intakeSystem;
    private Com_IntakeStart intakeStartCommand;
    private Com_IntakeStop intakeStopCommand;

    private WobbleSystem wobbleSystem;
    private Com_PickUp pickUpCommand;
    private Com_PutDown putDownCommand;

    private GamepadEx m_driverOp, m_toolOp;
    private Button toggleShooter, dpadUp, dpadDown, toggleIntake;
    private RevIMU imu;
    @Override
    public void initialize() {
        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");
        imu = new RevIMU(hardwareMap);


        //one of our motors is messed up so it has to be inverted woooooo
        bL.setInverted(true);
        intake = new Motor(hardwareMap, "intake", Motor.GoBILDA.BARE);
        shot = new Motor(hardwareMap, "shot", Motor.GoBILDA.BARE);

        servo = new SimpleServo(hardwareMap, "servo");
//        shot.setRunMode(Motor.RunMode.VelocityControl);

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

        shooterSystem = new ShooterSystem(shot, telemetry, () -> pwrSelect);
        shootCommand = new Com_Shoot(shooterSystem);
        stopCommand = new Com_NoShoot(shooterSystem);
        toggleShooter = new GamepadButton(m_driverOp, GamepadKeys.Button.A)
                .toggleWhenPressed(shootCommand);
        intakeSystem = new IntakeSystem(intake);
        intakeStartCommand = new Com_IntakeStart(intakeSystem);
        intakeStopCommand = new Com_IntakeStop(intakeSystem);
        toggleIntake = new GamepadButton(m_driverOp, GamepadKeys.Button.X)
                .toggleWhenPressed(intakeStartCommand);
//        wobbleSystem = new WobbleSystem(servo);
//        pickUpCommand = new Com_PickUp(wobbleSystem);
//        putDownCommand = new Com_PutDown(wobbleSystem);

        mecDrive.setDefaultCommand(driveCommand);

        register(mecDrive);

        schedule(driveCommand);
    }
}
