package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Timing;

import java.util.concurrent.TimeUnit;


public class WobbleSystem extends SubsystemBase {
    SimpleServo simpleServo;
    public WobbleSystem(SimpleServo pickMeUpDaddy){
        simpleServo = pickMeUpDaddy;
    }

    public void spinMeRightRoundBaby(){
        simpleServo.rotate(0.25);
    }
    public void putMeDownUwU(){
        simpleServo.rotate(0.5);
    }
}

