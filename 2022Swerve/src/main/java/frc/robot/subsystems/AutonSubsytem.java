package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.Objects;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;


public class AutonSubsytem extends SubsystemBase{
    private enum AutonModes{
        FORWARD, // Go forward 2 meters
        BACKWARD, // Wait 3 seconds, go backward 2 meters
        FORWARD180; // Go forward 2 meters and rotate 180 degrees
    }

    private ShuffleboardTab configTab = Shuffleboard.getTab("Config");
    private NetworkTableEntry delayEntry = configTab.add("Auton Delay", 0.0).getEntry();
    private SendableChooser<AutonModes> modeChooser = new SendableChooser<>();

    private DataLog datalog = DataLogManager.getLog();
    private StringLogEntry trajectoryLog = new StringLogEntry(datalog, "/auton/trajectory"); //Logs x translation state output
    private StringLogEntry commandLog = new StringLogEntry(datalog, "/auton/command"); //Logs x translation state output

    private SwerveSubsystem swerveSubsystem;

    private PIDController xController;
    private PIDController yController;
    private ProfiledPIDController rotationController;

    private TrajectoryConfig trajectoryConfig;

    private AutonModes autonMode;

    public AutonSubsytem(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;

        //Add auton modes to chooser
        for(AutonModes mode : AutonModes.values()){
            modeChooser.addOption(mode.toString(), mode);
        }
        configTab.add("Auton mode", modeChooser);

        //Define PID controllers for tracking trajectory
        xController = new PIDController(AutonConstants.kXTranslationP, 0, 0);
        yController = new PIDController(AutonConstants.kYTranslationP, 0, 0);
        rotationController = new ProfiledPIDController(
            AutonConstants.kRotationP, 0, 0, AutonConstants.kRotationalConstraints);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        //Define config for generated trajectories
        trajectoryConfig = new TrajectoryConfig(
            AutonConstants.kMaxSpeed, AutonConstants.kMaxAcceleration).
            setKinematics(DriveConstants.kDriveKinematics);
    }

    private Trajectory getTrajectory(String name) throws IOException{
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/" + name + ".wplib.json");
        return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }

    private Trajectory generateTrajectory(Pose2d initialPose, List<Translation2d> waypoints, Pose2d finalPose){
        return TrajectoryGenerator.generateTrajectory(
            initialPose,
            waypoints,
            finalPose,
            trajectoryConfig
        );
    }

    private SwerveControllerCommand followTrajectory(String name) throws IOException{
        //For use with trajectories generated from pathplanner
        trajectoryLog.append(name);
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

    private SwerveControllerCommand followTrajectory(Trajectory trajectory){
        //For use with trajectories generated from a list of poses
        trajectoryLog.append("Following generated trajectory");
        return new SwerveControllerCommand(
            trajectory,
            swerveSubsystem::getPose, 
            DriveConstants.kDriveKinematics, 
            xController, 
            yController, 
            rotationController, 
            swerveSubsystem::setModuleStates, 
            swerveSubsystem
        );
    }

    private Command resetOdometry(String initialTrajectory) throws IOException{
        //Resets odometry to the initial position of the given trajectory
        Trajectory trajectory = getTrajectory(initialTrajectory);
        Pose2d initialPose = Objects.isNull(trajectory) ? new Pose2d(0, 0, new Rotation2d()) : trajectory.getInitialPose();
        return new InstantCommand(() -> swerveSubsystem.resetOdometry(initialPose));
    }

    private Command delay(double seconds){
        return new WaitCommand(seconds).beforeStarting(() -> commandLog.append("Wait " + seconds + " seconds"));
    }
    
    private Command getAutonSequence(){
        autonMode = modeChooser.getSelected();
        //Sequence of actions to be performed during the autonomous period
        try{
        switch(autonMode){
            case FORWARD:
               return 
                    new SequentialCommandGroup(
                        resetOdometry("Forward"),
                        followTrajectory("Forward")
                    );
            case BACKWARD:
                return
                    new SequentialCommandGroup(
                        resetOdometry("Backward"),
                        delay(3),
                        followTrajectory("Backward")
                    );
            case FORWARD180:
                return
                    new SequentialCommandGroup(
                        resetOdometry("Forward180"),
                        followTrajectory("Forward180")
                    );
            default:
                return null;
        }
        }catch(IOException e){
            DriverStation.reportError("Was unable to access a trajectory", e.getStackTrace());
            return getBackupSequence();
        }
    }

    private Command getBackupSequence(){
        //Backup sequence in case a trajectory fails to load
        return new SequentialCommandGroup(
            followTrajectory(
                generateTrajectory(
                    new Pose2d(0, 0, new Rotation2d(0)),
                    List.of(
                        new Translation2d(1,0),
                        new Translation2d(1,-1)
                    ),
                    new Pose2d(0, 0, Rotation2d.fromDegrees(180))
                )
            )
        );
    }

    private Command getAutonEnd(){
        //Actions to be performed unconditionally after the autonomous sequence has ended (Stop motors)
        return new SequentialCommandGroup(
            new InstantCommand(() -> swerveSubsystem.stopModules())
        );
    }

    public Command getAutonCommand(){
        return getAutonSequence().beforeStarting(delay(delayEntry.getDouble(0.0))).andThen(getAutonEnd());
    }
}
