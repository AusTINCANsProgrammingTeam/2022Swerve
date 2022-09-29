package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OI {
    //Operator Interface (OI) class containing all control information
    private static final int kDriverJoystickPort = 0;

    public static class Driver{
        private static final Joystick kJoystick = new Joystick(OI.kDriverJoystickPort);

        private static final int kOrientationButtonID = 0; //Toggle swerve orientation
        private static final int kZeroButtonID = 0; //Zero the gyroscope

        private static final int kXTranslationAxis = 0;
        private static final int kYTranslationAxis = 0;
        private static final int kRotationAxis = 0;

        private static final ControlCurve kXTranslationCurve = new ControlCurve(0,0,0,0);
        private static final ControlCurve kYTranslationCurve = new ControlCurve(0,0,0,0);
        private static final ControlCurve kRotationCurve = new ControlCurve(0,0,0,0);

        public static Supplier<Double> getXTranslationSupplier(){
            return () -> kXTranslationCurve.calculate(kJoystick.getRawAxis(kXTranslationAxis));
        }

        public static Supplier<Double> getYTranslationSupplier(){
            return () -> kYTranslationCurve.calculate(kJoystick.getRawAxis(kYTranslationAxis));
        }

        public static Supplier<Double> getRotationSupplier(){
            return () -> kRotationCurve.calculate(kJoystick.getRawAxis(kRotationAxis));
        }

        public static JoystickButton getOrientationButton(){
            return new JoystickButton(kJoystick, kOrientationButtonID);
        }

        public static JoystickButton getZeroButton(){
            return new JoystickButton(kJoystick, kZeroButtonID);
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
