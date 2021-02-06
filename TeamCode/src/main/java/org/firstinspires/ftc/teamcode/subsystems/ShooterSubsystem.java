package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.TimedAction;

import java.util.function.BooleanSupplier;

public class ShooterSubsystem extends SubsystemBase {

    private Motor flywheel;
    private SimpleServo flicker;
    private TimedAction timedAction;
    private BooleanSupplier powerShotMode;

    public ShooterSubsystem(Motor flywheel, SimpleServo flicker, TimedAction timedAction,
                            BooleanSupplier powerShotMode, VoltageSensor voltageSensor){
        this.flywheel = flywheel;

        this.flywheel.setRunMode(Motor.RunMode.VelocityControl);
        this.flywheel.setVeloCoefficients(1.1, 0, 0.05);
        this.flywheel.setFeedforwardCoefficients(0, 1.0 * 12 / voltageSensor.getVoltage());

        this.flicker = flicker;
        this.timedAction = timedAction;
        this.powerShotMode = powerShotMode;
    }

    public boolean isRunning() {
        return timedAction.running();
    }

    public void shoot(){
            flywheel.set(0.8);
            flywheel.set(1.0);
    }

    public void stop(){
        flywheel.setRunMode(Motor.RunMode.RawPower);
        flywheel.stopMotor();
    }

    public void flick(){
        timedAction.run();
    }

    public void resetEncoder(){
        flywheel.resetEncoder();
    }

    public void flickReset(){
        if (!timedAction.running())
            timedAction.reset();
    }
    public void homePos(){
        flicker.setPosition(0.27);
    }

    public void setRunMode(Motor.RunMode runMode){
        flywheel.setRunMode(runMode);
    }
}
