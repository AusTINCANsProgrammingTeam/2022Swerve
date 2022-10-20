package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
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

    public AutonSubsytem(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;

        //Define PID controllers for tracking trajectory
        xController = new PIDController(AutonConstants.kXTranslationP, 0, 0);
        yController = new PIDController(AutonConstants.kYTranslationP, 0, 0);
        rotationController = new ProfiledPIDController(
            AutonConstants.kRotationP, 0, 0, AutonConstants.kRotationalConstraints);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    private Trajectory getTrajectory(String name){
        Trajectory trajectory;
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/" + name + ".wplib.json");
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + name, ex.getStackTrace());
            trajectory = null;
        }
        return trajectory;
    }

    private SwerveControllerCommand followTrajectory(String name){
        return new SwerveControllerCommand(
            getTrajectory(name),
            swerveSubsystem::getPose, 
            DriveConstants.kDriveKinematics, 
            xController, 
            yController, 
            rotationController, 
            swerveSubsystem::setModuleStates, 
            swerveSubsystem
        );
    }

    public Command getAutonCommand(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> swerveSubsystem.resetOdometry(getTrajectory("New Path").getInitialPose())),
            followTrajectory("New Path"),
            new InstantCommand(() -> swerveSubsystem.stopModules())
        );
    }
}
