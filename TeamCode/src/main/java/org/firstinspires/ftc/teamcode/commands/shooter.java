package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class Shooter extends CommandBase {

    private ShooterSubsystem simpleShooter;

    public Shooter(ShooterSubsystem shooterItem){
        addRequirements(shooterItem);
    }

    @Override
    public void initialize(){
        simpleShooter.flickReset();
        simpleShooter.resetEncoder();
        simpleShooter.setRunMode(); //IDK what to put in this
    }

    @Override
    public void shoot(){
        simpleShooter.set(1.0); //change power
    }
	
	@Override
    public void stopShooter(){
        simpleShooter.stopMotor(); //(or use .set(0))
    }
    
    
    public void orginalPos(){
        simpleShooter.homePos(); 
    }