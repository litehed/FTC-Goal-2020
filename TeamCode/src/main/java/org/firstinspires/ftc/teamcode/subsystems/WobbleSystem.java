package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;


public class WobbleSystem extends SubsystemBase {
    CRServo crServo;
    Motor motor;
    public WobbleSystem(CRServo pickMeUpDaddy, Motor mator){
        crServo = pickMeUpDaddy;
        motor = mator;
    }

    public void spinMeRightRoundBaby(){
        crServo.set(-0.8);
    }
    public void putMeDownUwU(){
        crServo.set(1.0);
    }
    public void motorUp(){
        motor.set(0.3);
    }
    public void motorDown(){motor.set(-0.3);}
    public void motorStop(){motor.stopMotor();}
}

