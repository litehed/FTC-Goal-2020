package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class WobbleSystem extends SubsystemBase {
    MotorEx rundhi;

    public WobbleSystem(MotorEx pickMeUpDaddy){
        rundhi = pickMeUpDaddy;
    }

    public void spinMeRightRoundBaby(){
        //15-20milis at 0.1
        rundhi.set(0.1);
        try {
            Thread.sleep(15);
            rundhi.set(0);
        }catch(Exception q){
            System.out.print("Ima be honest something aint right");
        }
    }
}

