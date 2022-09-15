// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//This code belongs to Asher123456789



package frc.robot.subsystems;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SimulationSubsystem extends SubsystemBase {

  private final Field2d m_field;
  
  private final SwerveSubsystem swerveSubsystem;
  private final SwerveDriveOdometry m_odometry;
  private int navXSim = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
  private double simYaw = 0;
  /** Creates a new SimulationSubsystem. */
  public SimulationSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;

    //This defines m_odometry with the DriveKinematics, Rotation of the swerve, and the robot's position
    m_odometry = new SwerveDriveOdometry(Constants.DriveConstants.kDriveKinematics, swerveSubsystem.getRotation2d(), new Pose2d(0, 0, new Rotation2d()));

    m_field = new Field2d(); 

    //This puts the field into SmartDashboard
    SmartDashboard.putData("Field", m_field); 
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run
    
    //This updates the odometry with the rotation and module states
    m_odometry.update(swerveSubsystem.getRotation2d(), swerveSubsystem.getModuleStates());
    
    //This sets the robot to the correct position on the field.
    m_field.setRobotPose(m_odometry.getPoseMeters());

    //These send the X Position, Y Position, and Rotation to SmartDashboard 
    SmartDashboard.putNumber("Pose X", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Pose Y", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Pose Rotation", m_odometry.getPoseMeters().getRotation().getDegrees());

    //Gets the Chassis Speeds (Combined Speeds) from the module states
    var chassisSpeed = Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(swerveSubsystem.getModuleStates());

    //Assigns the chassisRotationSpeed Value to a variable
    double chassisRotationSpeed = chassisSpeed.omegaRadiansPerSecond;

    // Adds the Chassis Rotation Speed + 0.02 to SimYaw
    simYaw += chassisRotationSpeed * 0.02;

    //
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(navXSim, "Yaw"));
    angle.set(-Units.radiansToDegrees(simYaw));
  }
}