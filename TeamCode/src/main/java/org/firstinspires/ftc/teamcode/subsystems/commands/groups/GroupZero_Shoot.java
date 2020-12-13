package org.firstinspires.ftc.teamcode.subsystems.commands.groups;

import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.jacksonSama.shhhnopeaking.ElapsedWait;
import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSystem;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_AutonDown;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_AutonLift;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_IntakeStart;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_NoShoot;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_PickUp;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_PutDown;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_Shoot;
import org.firstinspires.ftc.teamcode.subsystems.commands.drive.Com_DriveTime;
import org.firstinspires.ftc.teamcode.subsystems.commands.drive.Com_Rotate;
import org.firstinspires.ftc.teamcode.subsystems.commands.drive.Com_RotateTo;

import java.time.Instant;

public class GroupZero_Shoot extends SequentialCommandGroup {
    public GroupZero_Shoot(DriveSystem drive, ElapsedTime time, VoltageSensor voltageSensor, RevIMU imu,
                           WobbleSystem wobbleSystem, ShooterSystem shooterSystem, IntakeSystem intakeSystem) {
        addCommands(
                new Com_DriveTime(drive,0.5, 0D, 0D, time, 3.5),
                new InstantCommand(shooterSystem::shoot, shooterSystem),
                new ElapsedWait(1800),
                new InstantCommand(intakeSystem::down, intakeSystem),
                new ElapsedWait(700),
                new InstantCommand(intakeSystem::stop, intakeSystem),
                new ElapsedWait(1000),
                new InstantCommand(intakeSystem::down, intakeSystem),
                new ElapsedWait(3000),
                new InstantCommand(intakeSystem::stop, intakeSystem),
                new ElapsedWait(500),
                new InstantCommand(shooterSystem::stop, intakeSystem),
                new Com_RotateTo(drive, imu, 0),
                new Com_DriveTime(drive, -0.55, 0D, 0D, time, 3.8),
                new Com_RotateTo(drive, imu, 0),
                new Com_DriveTime(drive, 0D, -0.55, 0D, time, 3.15),
                new Com_Rotate(drive, imu, 180),
                new Com_DriveTime(drive, -0.55, 0D, 0D, time, 1.3),
                new Com_AutonDown(wobbleSystem, time),
                new ElapsedWait(2000),
                new FunctionalCommand(
                        () -> { return; }, wobbleSystem::putMeDownUwU,
                        bool -> wobbleSystem.servoStop(), () -> true, wobbleSystem),
                new ElapsedWait(2000),
                new Com_AutonLift(wobbleSystem, time));
    }
}
