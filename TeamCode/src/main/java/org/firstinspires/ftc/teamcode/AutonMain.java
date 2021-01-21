package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.vision.UGContourRingDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.groups.InitialMovement;
import org.firstinspires.ftc.teamcode.commands.vision.Com_Contour;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ContourVisionSystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;
import org.firstinspires.ftc.teamcode.util.TimedAction;

@Autonomous(name="PogU")
public class AutonMain extends CommandOpMode {
    //Servos and Motors
    private Motor fL, fR, bL, bR, arm, flyWheel;
    private SimpleServo flicker, grabber;

    //Subsystems
    private MecanumDriveSubsystem drive;
    private WobbleSubsystem wobble;
    private ShooterSubsystem shooterSystem;

    //Vision
    private UGContourRingDetector ugContourRingDetector;
    private ContourVisionSystem visionSystem;
    private Com_Contour visionCommand;

    //Extranious
    private TimedAction flickerAction;
    private ElapsedTime time;

    //Poses

    //Trajectories

    @Override
    public void initialize() {
//        grabber = new SimpleServo(hardwareMap, "wobbleS", 0, 270);
//        grabber.setInverted(true);
//        grabber.setPosition(1);
        arm = new Motor(hardwareMap, "wobble", Motor.GoBILDA.RPM_312);
        arm.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        grabber = new SimpleServo(hardwareMap, "wobbleS", 0, 270);
        time = new ElapsedTime();

        flyWheel = new Motor(hardwareMap, "shoot");
        flicker = new SimpleServo(hardwareMap, "flicker", 0, 270);

        flickerAction = new TimedAction(
                ()-> flicker.setPosition(0.5),
                ()-> flicker.setPosition(0.27),
                600,
                true
        );

        shooterSystem = new ShooterSubsystem(flyWheel, flicker, flickerAction, telemetry);

        ugContourRingDetector = new UGContourRingDetector(hardwareMap, "poopcam", telemetry, true);
        ugContourRingDetector.init();
        visionSystem = new ContourVisionSystem(ugContourRingDetector, telemetry);
        arm.resetEncoder();
        visionCommand = new Com_Contour(visionSystem, time);

        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        wobble = new WobbleSubsystem(arm, grabber);

        SequentialCommandGroup autonomous = new SequentialCommandGroup(
                new InstantCommand(wobble::closeGrabber),
                new WaitUntilCommand(this::isStarted),
                visionCommand,
                new InitialMovement(drive, wobble,shooterSystem)
        );
        schedule(autonomous);
    }
}
