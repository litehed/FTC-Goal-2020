package org.firstinspires.ftc.teamcode.jacksonSama;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name="Mooooto")
public class MotorTesting extends LinearOpMode {

    private Motor test;

    @Override
    public void runOpMode() throws InterruptedException {
        test = new Motor(hardwareMap, "shot", Motor.GoBILDA.BARE);
        test.setRunMode(Motor.RunMode.VelocityControl);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                test.set(1);
            }
            else test.set(0);
        }
        test.set(0);
    }

}