package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;

public class AutonSubsytem extends SubsystemBase{

    private SwerveSubsystem swerveSubsystem;

    private PIDController xController;
    private PIDController yController;
    private ProfiledPIDController rotationController;

    private TrajectoryConfig trajectoryConfig;

    public AutonSubsytem(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;

        //Define PID controllers for tracking trajectory
        xController = new PIDController(AutonConstants.kXTranslationP, 0, 0);
        yController = new PIDController(AutonConstants.kYTranslationP, 0, 0);
        rotationController = new ProfiledPIDController(
            AutonConstants.kRotationP, 0, 0, AutonConstants.kRotationalConstraints);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        //Create trajectory settings
        trajectoryConfig = new TrajectoryConfig(
            AutonConstants.kMaxSpeed, AutonConstants.kMaxAcceleration).
            setKinematics(DriveConstants.kDriveKinematics);
    }

    public Trajectory getTrajectory(){
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(1,0),
                new Translation2d(1,-1)
            ),
            new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
            trajectoryConfig
        );
    }

    public Command getAutonCommand(){
        Trajectory trajectory = getTrajectory();

        //Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            swerveSubsystem::getPose, 
            DriveConstants.kDriveKinematics, 
            xController, 
            yController, 
            rotationController, 
            swerveSubsystem::setModuleStates, 
            swerveSubsystem
        );

        return new SequentialCommandGroup(
            new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> swerveSubsystem.stopModules())
        );
    }
}
