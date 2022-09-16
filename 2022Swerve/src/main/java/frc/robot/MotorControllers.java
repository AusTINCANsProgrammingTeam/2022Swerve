package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.MotorDefaults;

public enum MotorControllers {
    //Swerve Modules
    FrontLeftModuleDrive(1, 50),
    FrontLeftModuleTurn(2),
    FrontRightModuleDrive(3, 50),
    FrontRightModuleTurn(4),
    BackLeftModuleDrive(5, 50),
    BackLeftModuleTurn(6),
    BackRightModuleDrive(7, 50),
    BackRightModuleTurn(8);

    private int ID;
    private int currentLimit;
    private double openLoopRampRate;
    private CANSparkMax motor;

    MotorControllers(int ID, int currentLimit, double openLoopRampRate){
        this.ID = ID;
        this.currentLimit = currentLimit;
        this.openLoopRampRate = openLoopRampRate;
        initializeMotor();
    }

    MotorControllers(int ID, int currentLimit){
        this.ID = ID;
        this.currentLimit = currentLimit;
        this.openLoopRampRate = MotorDefaults.kOpenLoopRampRate;
        initializeMotor();
    }

    MotorControllers(int ID){
        this.ID = ID;
        this.currentLimit = MotorDefaults.kCurrentLimit;
        this.openLoopRampRate = MotorDefaults.kOpenLoopRampRate;
        initializeMotor();
    }

    private void initializeMotor(){
        motor = new CANSparkMax(ID, MotorType.kBrushless);
        motor.setOpenLoopRampRate(openLoopRampRate);
        motor.setSmartCurrentLimit(currentLimit);
    }

    public CANSparkMax getMotor(){
        return motor;
    }
}
