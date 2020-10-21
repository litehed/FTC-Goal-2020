package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Robot;

public class CrimsonRobot extends Robot {

    Robot robot;
    //Used so robot no confuse things
    public enum CurOpMode{AUTON, TELEOP}

    public CrimsonRobot(CurOpMode mode){
        if (mode == CurOpMode.TELEOP)
            teleopInit();
        else
            autonInit();
    }

    public void teleopInit(){
        //Ill fill this in later my brain is fried rn
    }
    public void autonInit(){

    }
}

