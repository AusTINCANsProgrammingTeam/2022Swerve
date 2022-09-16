package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorControllers;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase{
    private final SwerveModule frontLeft = new SwerveModule(
        MotorControllers.FrontLeftModuleDrive.getMotor(),
        MotorControllers.FrontLeftModuleTurn.getMotor(),
        DriveConstants.FrontLeftModule.driveMotorReversed, 
        DriveConstants.FrontLeftModule.turningMotorReversed, 
        DriveConstants.FrontLeftModule.absoluteEncoderID, 
        DriveConstants.FrontLeftModule.absoluteEncoderOffset, 
        DriveConstants.FrontLeftModule.absoluteEncoderReversed
        );

    private final SwerveModule frontRight = new SwerveModule(
        MotorControllers.FrontRightModuleDrive.getMotor(),
        MotorControllers.FrontRightModuleTurn.getMotor(),
        DriveConstants.FrontRightModule.driveMotorReversed, 
        DriveConstants.FrontRightModule.turningMotorReversed, 
        DriveConstants.FrontRightModule.absoluteEncoderID, 
        DriveConstants.FrontRightModule.absoluteEncoderOffset, 
        DriveConstants.FrontRightModule.absoluteEncoderReversed
        );

    private final SwerveModule backLeft = new SwerveModule(
        MotorControllers.BackLeftModuleDrive.getMotor(),
        MotorControllers.BackLeftModuleTurn.getMotor(),
        DriveConstants.BackLeftModule.driveMotorReversed, 
        DriveConstants.BackLeftModule.turningMotorReversed, 
        DriveConstants.BackLeftModule.absoluteEncoderID, 
        DriveConstants.BackLeftModule.absoluteEncoderOffset, 
        DriveConstants.BackLeftModule.absoluteEncoderReversed
        );

    private final SwerveModule backRight = new SwerveModule(
        MotorControllers.BackRightModuleDrive.getMotor(),
        MotorControllers.BackRightModuleTurn.getMotor(),
        DriveConstants.BackRightModule.driveMotorReversed, 
        DriveConstants.BackRightModule.turningMotorReversed, 
        DriveConstants.BackRightModule.absoluteEncoderID, 
        DriveConstants.BackRightModule.absoluteEncoderOffset, 
        DriveConstants.BackRightModule.absoluteEncoderReversed
        );

    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
        
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360); //Clamps angle output between -180 and 180 degrees
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
    }

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeed);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

}