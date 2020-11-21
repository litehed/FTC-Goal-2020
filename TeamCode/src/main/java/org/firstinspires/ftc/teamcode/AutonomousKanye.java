package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.vision.UGRectDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSystem;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_PutDown;
import org.firstinspires.ftc.teamcode.subsystems.commands.drive.Com_DriveTime;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_Vision;
import org.firstinspires.ftc.teamcode.subsystems.commands.drive.Com_Rotate;
import org.firstinspires.ftc.teamcode.subsystems.commands.groups.GroupFour;
import org.firstinspires.ftc.teamcode.subsystems.commands.groups.GroupOne;
import org.firstinspires.ftc.teamcode.subsystems.commands.groups.GroupZero;

import java.util.HashMap;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.BRAKE;

@Autonomous(name="Kanye North")
public class AutonomousKanye extends CommandOpMode {
    private Motor fL, bL, fR, bR;
    private Motor wobble, test;
    private CRServo servo;
    private UGRectDetector ugRectDetector;
    private DriveSystem mecDrive;
    private Com_DriveTime intialStrafe;

    private VisionSystem visionSystem;
    private Com_Vision visionCommand;

    private WobbleSystem wobbleSystem;

    private ElapsedTime time;
    private RevIMU imu;
    private VoltageSensor voltageSensor;
    @Override
    public void initialize() {
        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");
        //one of our motors is messed up so it has to be inverted woooooo
        bL.setInverted(true);

        fL.setZeroPowerBehavior(BRAKE);
        fR.setZeroPowerBehavior(BRAKE);
        bL.setZeroPowerBehavior(BRAKE);
        bR.setZeroPowerBehavior(BRAKE);

        wobble = new Motor(hardwareMap, "wobble");
        servo = new CRServo(hardwareMap, "servo");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        //named shot purely because im too lazy to change config
        test = new Motor(hardwareMap, "shot");
        ugRectDetector = new UGRectDetector(hardwareMap);
        ugRectDetector.init();
        ugRectDetector.setTopRectangle(0.46, 0.45);
        ugRectDetector.setBottomRectangle(0.46, 0.39);
        ugRectDetector.setRectangleSize(10, 30);
        imu = new RevIMU(hardwareMap);
        imu.init();


        time = new ElapsedTime();
        mecDrive = new DriveSystem(fL, fR, bL, bR);
        wobbleSystem = new WobbleSystem(servo, wobble);
        visionSystem = new VisionSystem(ugRectDetector, telemetry);
        visionCommand = new Com_Vision(visionSystem);

        intialStrafe = new Com_DriveTime(mecDrive,0.5, 0D, 0D, time, 2.0);
                register(mecDrive, new SubsystemBase(){
            @Override
            public void periodic() {
                telemetry.addData("imu heading", imu.getHeading());
                telemetry.addData("rings", visionSystem.getStackSize());
                telemetry.update();
            }
        });

        SequentialCommandGroup wobbleGoal = new SequentialCommandGroup(
                intialStrafe,
                visionCommand,
                new SelectCommand(new HashMap<Object, Command>() {{
                    put(VisionSystem.Size.ZERO, new GroupZero(mecDrive, time, voltageSensor, imu));
                    put(VisionSystem.Size.ONE, new GroupOne(mecDrive, time, voltageSensor));
                    put(VisionSystem.Size.FOUR, new GroupFour(mecDrive, time, voltageSensor, imu));
                }},visionSystem::getStackSize), new Com_PutDown(wobbleSystem, time)
//                new SelectCommand(new HashMap<Object, Command>() {{
//                    put(VisionSystem.Size.ZERO, new Com_DriveTime(mecDrive,
//                            0D, -0.5, 0D, time, 2.0));
//                    put(VisionSystem.Size.ONE, new Com_DriveTime(mecDrive,
//                            0D, 0.5, 0D, time, 1.0));
//                    put(VisionSystem.Size.FOUR, new Com_DriveTime(mecDrive,
//                            0D, 0.5, 0D, time, 1.0));
//                }},visionSystem::getStackSize)

        );

        schedule(wobbleGoal);
    }
}
