package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OI {
    //Operator Interface (OI) class containing all control information
    private static final int driverJoystickPort = 0;

    public static class Driver{
        private static final Joystick joystick = new Joystick(OI.driverJoystickPort);

        private static final int orientationButtonID = 0; //Toggle swerve orientation
        private static final int zeroButtonID = 0; //Zero the gyroscope

        private static final int xTranslationAxis = 0;
        private static final int yTranslationAxis = 0;
        private static final int rotationAxis = 0;

        private static final ControlCurve xTranslationCurve = new ControlCurve(0,0,0,0);
        private static final ControlCurve yTranslationCurve = new ControlCurve(0,0,0,0);
        private static final ControlCurve rotationCurve = new ControlCurve(0,0,0,0);

        public static Supplier<Double> getXTranslationSupplier(){
            return () -> xTranslationCurve.calculate(joystick.getRawAxis(xTranslationAxis));
        }

        public static Supplier<Double> getYTranslationSupplier(){
            return () -> yTranslationCurve.calculate(joystick.getRawAxis(yTranslationAxis));
        }

        public static Supplier<Double> getRotationSupplier(){
            return () -> rotationCurve.calculate(joystick.getRawAxis(rotationAxis));
        }

        public static JoystickButton getOrientationButton(){
            return new JoystickButton(joystick, orientationButtonID);
        }

        public static JoystickButton getZeroButton(){
            return new JoystickButton(joystick, zeroButtonID);
        }
    }

    public static final class Operator{

    }

    public static class ControlCurve{
        private double ySaturation;
        private double yIntercept;
        private double curvature;
        private double deadzone;

        public ControlCurve(double ySaturation, double yIntercept, double curvature, double deadzone){
            this.ySaturation = ySaturation;
            this.yIntercept = yIntercept;
            this.curvature = curvature;
            this.deadzone = deadzone;
        }

        public double calculate(double input){
            return Math.abs(input) <  deadzone ? 0 : 
            ySaturation * (Math.signum(input) * yIntercept + 
            (1 - yIntercept) * (curvature * Math.pow(input, 3) +
            (1 - curvature) * input));
        }
    }
}
