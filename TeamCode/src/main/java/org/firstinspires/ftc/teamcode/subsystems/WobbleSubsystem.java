package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WobbleSubsystem extends SubsystemBase {

    private Motor arm;
    private SimpleServo grabber;
    private Telemetry telemetry;
    private boolean grabbing = false;

    public WobbleSubsystem(Motor arm, SimpleServo grabber, Telemetry telemetry){
            this.arm = arm;
            this.grabber = grabber;
            this.telemetry = telemetry;

            this.arm.setRunMode(Motor.RunMode.PositionControl);
            this.arm.resetEncoder();
    }

    //TODO: Set proper positions for grabber
    public void openGrabber(){
        grabbing = false;
        grabber.turnToAngle(0);
    }
    public void closeGrabber(){
        grabbing = true;
        grabber.turnToAngle(90);
    }
    public boolean isGrabbing(){
        return grabbing;
    }

    public Motor getMotor(){
        return arm;
    }
    public void stopMotor(){
        arm.stopMotor();
    }
    public void armUp(){
        arm.set(0.5);
    }
    public void armDown(){
        arm.set(0.3);
    }
    @Override
    public void periodic(){
        telemetry.addData("Position", arm.getCurrentPosition());
        telemetry.update();
    }
}
