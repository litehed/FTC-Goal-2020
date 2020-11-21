package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.vision.UGRectDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSystem;
import org.firstinspires.ftc.teamcode.subsystems.commands.drive.Com_DriveTime;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_Vision;
import org.firstinspires.ftc.teamcode.subsystems.commands.drive.Com_Rotate;
import org.firstinspires.ftc.teamcode.subsystems.commands.groups.GroupFour;
import org.firstinspires.ftc.teamcode.subsystems.commands.groups.GroupZero;

import java.util.HashMap;

@Autonomous(name="Kanye North")
public class AutonomousKanye extends CommandOpMode {
    private Motor fL, bL, fR, bR;
    private Motor test;
    private UGRectDetector ugRectDetector;
    private DriveSystem mecDrive;
    private Com_DriveTime driveTime;

    private VisionSystem visionSystem;
    private Com_Vision visionCommand;

    private ElapsedTime time;
    private RevIMU imu;
    @Override
    public void initialize() {
        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");
        //one of our motors is messed up so it has to be inverted woooooo
        bL.setInverted(true);
        //named shot purely because im too lazy to change config
        test = new Motor(hardwareMap, "shot");
        ugRectDetector = new UGRectDetector(hardwareMap);
        ugRectDetector.init();
        ugRectDetector.setTopRectangle(0.35, 0.40);
        ugRectDetector.setRectangleSize(10, 35);
        imu = new RevIMU(hardwareMap);
        imu.init();


        time = new ElapsedTime();
        mecDrive = new DriveSystem(fL, fR, bL, bR);
        visionSystem = new VisionSystem(ugRectDetector, telemetry);
        visionCommand = new Com_Vision(visionSystem);
        register(mecDrive, new SubsystemBase(){
            @Override
            public void periodic() {
                telemetry.addData("imu heading", imu.getHeading());
                telemetry.update();
            }
        });

        SequentialCommandGroup wobbleGoal = new SequentialCommandGroup(
                visionCommand,
                new SelectCommand(new HashMap<Object, Command>() {{
                    put(VisionSystem.Size.ZERO, new GroupZero(mecDrive, time));
                    put(VisionSystem.Size.ONE, new Com_DriveTime(mecDrive,
                            0D, 0.5, 0D, time, 4.0));
                    put(VisionSystem.Size.FOUR, new GroupFour(mecDrive, time));
                }},visionSystem::getStackSize)
        );
        schedule(wobbleGoal);
    }
}
