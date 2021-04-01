//package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.UGContourRingDetector;

/*
Four ring autonomous path made in roadrunner which picks up and drops off both wobble goals,
shoots the three rings preloaded in the robot, then intakes the four on the ground.
All I need are the trajectories
 */

@Autonomous(name="Road Runner Test")
public class RoadRunner extends LinearOpMode {
    private Motor fL, fR, bL, bR, intake;
    private Motor shooterF;
    private SimpleServo grabWobble;

    @Override
    public void runOpMode() throws InterruptedException {

        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        shooterF = new Motor(hardwareMap, "shooterF");
        intake = new Motor(hardwareMap, "intake");

        grabWobble = new SimpleServo(hardwareMap, "grabWobble", -90, 90);


        shooterF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        fR.encoder.setDirection(Motor.Direction.REVERSE);

        waitForStart();

        fL.resetEncoder();
        fR.resetEncoder();
        bL.resetEncoder();
        bR.resetEncoder();

        //Pose2d startPose = new Pose2d(-63, -48, Math.toRadians(0));
        //drive.setPoseEstimate(startPose);

        new TrajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-40, -40, Math.toRadians(90)))
                .build();

        new TrajectoryBuilder(new Pose2d())
                .back(12)
                .build();

        // grab the wobble goal here

        new TrajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(80, 80, Math.toRadians(90)))
                .build();

        new TrajectoryBuilder(new Pose2d()) //(95,78)
                .strafeTo(new Vector2d(15, 10))
                .build();

        // drop the wobble goal here

        //bottom left 147 130
        new TrajectoryBuilder(new Pose2d())
                .splineToLinearHeading(new Pose2d(-147, -130, Math.toRadians(90)), Math.toRadians(0))
                .build();
        //pick 2nd wobble goal

        new TrajectoryBuilder(new Pose2d())
                .strafeLeft(80)
                .build();

        new TrajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(10, 80))
                .build();

        //drop wobble goal

        new TrajectoryBuilder(new Pose2d())
                .splineToLinearHeading(new Pose2d(-45, -40, Math.toRadians(90)), Math.toRadians(0))
                .build();
        //might need to rotate a bit to the right
        //shoot 3 rings


        new TrajectoryBuilder(new Pose2d())
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(90)), Math.toRadians(0))
                .build();

        //pick up the rings

        //reset to good starting position
    }
}
