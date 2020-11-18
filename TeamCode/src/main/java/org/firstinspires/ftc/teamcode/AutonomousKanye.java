package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.vision.UGRectDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSystem;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_PickUp;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_Vision;

import java.util.HashMap;

@Autonomous(name="Kanye North")
public class AutonomousKanye extends CommandOpMode {
    private Motor fL, bL, fR, bR;
    private Motor test;
    private UGRectDetector ugRectDetector;
    private DriveSystem mecDrive;

    private VisionSystem visionSystem;
    private Com_Vision visionCommand;
    private ElapsedTime time;

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


        time = new ElapsedTime();
        mecDrive = new DriveSystem(fL, fR, bL, bR);
        visionSystem = new VisionSystem(ugRectDetector, telemetry);
        visionCommand = new Com_Vision(visionSystem);
        register(mecDrive);

        SequentialCommandGroup wobbleGoal = new SequentialCommandGroup(
                visionCommand,
                new SelectCommand(new HashMap<Object, Command>() {{
                    put(VisionSystem.Size.ZERO, new InstantCommand(() -> {test.set(1);}));
                    put(VisionSystem.Size.ONE, new InstantCommand(() -> {test.set(0.5);}));
                    put(VisionSystem.Size.FOUR, new InstantCommand(() -> {test.set(0.1);}));
                }},visionSystem::getStackSize)
        );
        schedule(wobbleGoal);
    }
}
