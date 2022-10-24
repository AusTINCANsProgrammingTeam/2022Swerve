package frc.robot.hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.MotorDefaults;

public class MotorController {
    public static enum MotorControllers {
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
        private boolean reversed;

        MotorControllers(int ID, int currentLimit, double openLoopRampRate, boolean reversed){
            this.ID = ID;
            this.currentLimit = currentLimit;
            this.openLoopRampRate = openLoopRampRate;
            this.reversed = reversed;
        }

        MotorControllers(int ID, int currentLimit, double openLoopRampRate){
            this(ID, currentLimit, openLoopRampRate, false);
        }

        MotorControllers(int ID, int currentLimit, boolean reversed){
            this(ID, currentLimit, MotorDefaults.kOpenLoopRampRate, reversed);
        }

        MotorControllers(int ID, int currentLimit){
            this(ID, currentLimit, MotorDefaults.kOpenLoopRampRate, false);
        }

        MotorControllers(int ID, boolean reversed){
            this(ID, MotorDefaults.kCurrentLimit, MotorDefaults.kOpenLoopRampRate, reversed);
        }

        MotorControllers(int ID){
            this(ID, MotorDefaults.kCurrentLimit, MotorDefaults.kOpenLoopRampRate, false);
        }

        public int getID(){
            return ID;
        }

        public int getCurrentLimit(){
            return currentLimit;
        }

        public double getOpenLoopRampRate(){
            return openLoopRampRate;
        }

        public boolean getReversed(){
            return reversed;
        }
    }

    public static CANSparkMax constructMotor(MotorControllers config){
        CANSparkMax motor = new CANSparkMax(config.getID(), MotorType.kBrushless);
        motor.setSmartCurrentLimit(config.getCurrentLimit());
        motor.setOpenLoopRampRate(motor.getOpenLoopRampRate());
        motor.setInverted(motor.getInverted());
        return motor;
    }
}
