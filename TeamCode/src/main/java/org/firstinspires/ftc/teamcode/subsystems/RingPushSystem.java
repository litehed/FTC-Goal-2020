package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.util.Timing;

import java.util.concurrent.TimeUnit;

public class RingPushSystem extends SubsystemBase {
    private ServoEx servoPush;
    private Timing.Timer timeyWimey;
    public RingPushSystem(ServoEx pushy) {
        servoPush = pushy;
        timeyWimey = new Timing.Timer(5, TimeUnit.SECONDS);
    }

    public void push(){
        servoPush.turnToAngle(40);
        timeyWimey.start();
        servoPush.turnToAngle(0);
    }
}
