package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.vision.UGContourRingDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.groups.InitialMovement;
import org.firstinspires.ftc.teamcode.commands.vision.Com_Contour;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ContourVisionSystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

@Autonomous(name="PogU")
public class AutonMain extends CommandOpMode {
    //Servos and Motors
    private Motor fL, fR, bL, bR, arm;
    private SimpleServo grabber;

    //Subsystems
    private MecanumDriveSubsystem drive;
    private WobbleSubsystem wobble;

    //Vision
    private UGContourRingDetector ugContourRingDetector;
    private ContourVisionSystem visionSystem;
    private Com_Contour visionCommand;

    //Extranious
    private ElapsedTime time;

    //Poses

    //Trajectories

    @Override
    public void initialize() {
//        grabber = new SimpleServo(hardwareMap, "wobbleS", 0, 270);
//        grabber.setInverted(true);
//        grabber.setPosition(1);
        arm = new Motor(hardwareMap, "wobble", Motor.GoBILDA.RPM_312);
        grabber = new SimpleServo(hardwareMap, "wobbleS", 0, 270);
        time = new ElapsedTime();

        ugContourRingDetector = new UGContourRingDetector(hardwareMap, "poopcam", telemetry, true);
        ugContourRingDetector.init();
        visionSystem = new ContourVisionSystem(ugContourRingDetector, telemetry);
        visionCommand = new Com_Contour(visionSystem, time);

        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        wobble = new WobbleSubsystem(arm, grabber);

        SequentialCommandGroup autonomous = new SequentialCommandGroup(
                new WaitUntilCommand(this::isStarted),
                visionCommand,
                new InitialMovement(drive, wobble)
        );
        schedule(autonomous);
    }
}
