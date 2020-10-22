package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter extends SubsystemBase {
    private Motor kuunav;

    public Shooter(HardwareMap hmap, String name){
        kuunav = new Motor(hmap, name);
    }

    public void shot(){
        kuunav.set(1);
    }

    public void stoop(){
        kuunav.set(0);
    }
}
