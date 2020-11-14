package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Srihith")
public class AutonPlsWorkOwO extends LinearOpMode {

    private MotorEx fL, fR, bL, bR;
    private MotorEx encoderLeft, encoderRight;
    private MotorGroup left, right;
    private DifferentialDrive diffy;
    private DifferentialOdometry diffyOdom;
    private static final double TRACKWIDTH = 13.4;
    private static double TICKS_TO_INCHES;
    private static final double WHEEL_DIAMETER = 4.0;
    private static double TICKS_PER_REV;



    @Override
    public void runOpMode() throws InterruptedException {
        fL = new MotorEx(hardwareMap, "fL", Motor.GoBILDA.RPM_435);
        fR = new MotorEx(hardwareMap, "fR", Motor.GoBILDA.RPM_435);
        bL = new MotorEx(hardwareMap, "bL", Motor.GoBILDA.RPM_435);
        bR = new MotorEx(hardwareMap, "bR", Motor.GoBILDA.RPM_435);

        TICKS_PER_REV = fL.getCPR();
        TICKS_TO_INCHES = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

        left = new MotorGroup(fL, bL);
        right = new MotorGroup(fR, bR);

        Motor.Encoder leftEncoder = fL.encoder.setDistancePerPulse(TICKS_TO_INCHES);
        Motor.Encoder rightEncoder = fR.encoder.setDistancePerPulse(TICKS_TO_INCHES);
        rightEncoder.setDirection(Motor.Direction.REVERSE);

        diffy = new DifferentialDrive(left, right);
        diffyOdom = new DifferentialOdometry(leftEncoder::getDistance, rightEncoder::getDistance, TRACKWIDTH);
        waitForStart();
        while (opModeIsActive()&& !isStopRequested()) {
//            diffyOdom.updatePosition(5, 5); will forever remain
            diffyOdom.updatePose();
        }
    }
}
