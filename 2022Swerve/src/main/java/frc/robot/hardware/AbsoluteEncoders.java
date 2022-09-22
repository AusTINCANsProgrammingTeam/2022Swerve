package frc.robot.hardware;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

public enum AbsoluteEncoders {
    //Swerve Modules
    FrontLeftModule(9),
    FrontRightModule(10),
    BackLeftModule(11),
    BackRightModule(12);
    
    private int ID;
    private boolean reversed;
    private double offset; //Offset in radians
    private AnalogInput encoder;

    AbsoluteEncoders(int ID, boolean reversed, double offset){
        this.ID = ID;
        this.reversed = reversed;
        this.offset = offset;
        encoder = new AnalogInput(this.ID);
    }

    AbsoluteEncoders(int ID, boolean reversed){
        this.ID = ID;
        this.reversed = reversed;
        this.offset = 0;
        encoder = new AnalogInput(this.ID);
    }

    AbsoluteEncoders(int ID){
        this.ID = ID;
        this.reversed = false;
        this.offset = 0;
        encoder = new AnalogInput(this.ID);
    }

    private double calculateAngleRadians(){ 
        //Calculate the value of the absolute encoder in radians
        double angle = encoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= offset;
        return angle * (reversed ? -1.0 : 1.0);
    }

    public AnalogInput getEncoder() {
        return encoder;
    }

    public Supplier<Double> getAngleSupplier() { 
        //Use this supplier to get the encoder value in radians
        return () -> calculateAngleRadians();
    }
}
