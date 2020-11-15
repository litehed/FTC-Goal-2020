package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOpMain;

import java.util.function.DoubleSupplier;

public class ShooterSystem extends SubsystemBase {
    private Motor shooterMotor;
    private Telemetry telemetry;
    private DoubleSupplier power;
    private boolean shooterActive;

    public ShooterSystem(Motor ShooterMotor, Telemetry telemetryIn, DoubleSupplier getPower){
        shooterMotor = ShooterMotor;
        telemetry = telemetryIn;
        power = getPower;
        shooterActive = true;

    }
    public boolean active() {
        return shooterActive;
    }

    public void toggle() {
        shooterActive = !shooterActive;
    }
    public void shoot(){
        telemetry.addData("Shooter speed", power.getAsDouble());
        telemetry.update();
        shooterMotor.set(power.getAsDouble());
    }

    public void stop(){
        shooterMotor.stopMotor();
    }
}
