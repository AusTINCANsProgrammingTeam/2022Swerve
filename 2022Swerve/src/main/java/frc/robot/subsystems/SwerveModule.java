// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.classes.RelativeEncoderSim;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwerveModule extends SubsystemBase {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoderSim simDriveEncoder;
    private final RelativeEncoderSim simTurningEncoder;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPIDController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad; //The amount that the absolute encoder is offset from the wheel position, in radians

    private double simTurningEncoderDistance;
    private double simDriveEncoderDistance;

    private double driveOutput = 0;
    private double turnOutput = 0;

    //TODO: FIX THE NUMBERS!
    private final FlywheelSim moduleRotationSimModel = new FlywheelSim(
                  LinearSystemId.identifyVelocitySystem(0.16, 0.0348),
                  DCMotor.getNEO(1),
                  12.8
          );
        
          private final FlywheelSim moduleThrottleSimModel = new FlywheelSim(
                  LinearSystemId.identifyVelocitySystem(2, 1.24),
                  DCMotor.getNEO(1),
                  8.16
          );

    public SwerveModule(int driveMotorID, int turningMotorID, boolean driveMotorReversed, boolean turningMotorReversed,
             int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderID);

        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(SwerveModuleConstants.kDriveEncoderRotFactor);
        driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDriveEncoderRPMFactor);
        turningEncoder.setPositionConversionFactor(SwerveModuleConstants.kTurningEncoderRotFactor);
        turningEncoder.setVelocityConversionFactor(SwerveModuleConstants.kTurningEncoderRPMFactor);

        turningPIDController = new PIDController(SwerveModuleConstants.kPTurning, 0, 0);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        simDriveEncoder = new RelativeEncoderSim(driveMotor);
        simTurningEncoder = new RelativeEncoderSim(turningMotor);

        simTurningEncoderDistance = 0;
        simDriveEncoderDistance = 0;

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return simDriveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return simTurningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() { //Get the value of the absolute encoder in radians
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) { //Prevent wheels to returning to heading of 0 when controls released
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        turnOutput = turningPIDController.calculate(getTurningPosition(), state.angle.getRadians());
        driveOutput = state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeed;
        driveMotor.set(driveOutput);
        turningMotor.set(turnOutput);
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    @Override
    public void simulationPeriodic() {
        
        moduleRotationSimModel.setInputVoltage(turnOutput  * RobotController.getBatteryVoltage());
        moduleThrottleSimModel.setInputVoltage(driveOutput * RobotController.getBatteryVoltage());

        moduleRotationSimModel.update(Robot.kDefaultPeriod);
        moduleThrottleSimModel.update(Robot.kDefaultPeriod);

        if (moduleRotationSimModel.getAngularVelocityRadPerSec() >= Constants.kSimVelocityDeadzone) {
            simTurningEncoderDistance += moduleRotationSimModel.getAngularVelocityRadPerSec() * Robot.kDefaultPeriod;
        }
        simTurningEncoder.setPosition(simTurningEncoderDistance);
        simTurningEncoder.setSimVelocity(moduleRotationSimModel.getAngularVelocityRadPerSec());

        if (moduleThrottleSimModel.getAngularVelocityRadPerSec() >= Constants.kSimVelocityDeadzone) {
            simDriveEncoderDistance += moduleThrottleSimModel.getAngularVelocityRadPerSec() * Robot.kDefaultPeriod;
        }
        simDriveEncoder.setPosition(simDriveEncoderDistance);
        simDriveEncoder.setSimVelocity(moduleThrottleSimModel.getAngularVelocityRadPerSec());
  }
}