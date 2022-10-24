package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
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

    private SwerveSubsystem swerveSubsystem;

    private PIDController xController;
    private PIDController yController;
    private ProfiledPIDController rotationController;

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

    private Command resetOdometry(String initialTrajectory){
        //Resets odometry to the initial position of the given trajectory
        return new InstantCommand(() -> swerveSubsystem.resetOdometry(getTrajectory(initialTrajectory).getInitialPose()));
    }
    
    private Command getAutonSequence(){
        autonMode = modeChooser.getSelected();
        //Sequence of actions to be performed during the autonomous period
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
                        new WaitCommand(3),
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
    }

    private Command getAutonEnd(){
        //Actions to be performed unconditionally after the autonomous seqence has ended (Stop motors)
        return new SequentialCommandGroup(
            new InstantCommand(() -> swerveSubsystem.stopModules())
        );
    }

    public Command getAutonCommand(){
        return getAutonSequence().beforeStarting(new WaitCommand(delayEntry.getDouble(0.0))).andThen(getAutonEnd());
    }
}
