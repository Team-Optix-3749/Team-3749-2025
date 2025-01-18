package frc.robot.commands.swerve;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.commands.auto.AutoConstants;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.ToPos;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;

public class OnTheFly extends Command {

    PathPlannerTrajectory trajectory;
    Timer timer = new Timer();
    PPHolonomicDriveController SwerveController = new PPHolonomicDriveController(
        new PIDConstants(AutoConstants.kPDrive, 0, AutoConstants.kDDrive), 
        new PIDConstants(AutoConstants.kPTurn,0,AutoConstants.kDTurn));

    public OnTheFly() {
        
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        PathPlannerPath path = ToPos.generateDynamicPath(Robot.swerve.getPose(),
        Robot.swerve.getPPSetpoint(),
            Robot.swerve.getMaxDriveSpeed(), SwerveConstants.DriveConstants.maxAccelerationMetersPerSecondSquared,
            Robot.swerve.getMaxAngularSpeed(), SwerveConstants.DriveConstants.maxAngularAccelerationRadiansPerSecondSquared,new Translation2d(4.5,4),0.940816);
        try {
            trajectory = path.generateTrajectory(Robot.swerve.getChassisSpeeds(), Robot.swerve.getRotation2d(), 
            RobotConfig.fromGUISettings());
        } catch (IOException | ParseException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void execute() {
        double currentTime = timer.get();
        PathPlannerTrajectoryState goalState = trajectory.sample(currentTime);
        ChassisSpeeds speeds = SwerveController.calculateRobotRelativeSpeeds(Robot.swerve.getPose(), goalState);
        Robot.swerve.setModuleStates(DriveConstants.driveKinematics.toSwerveModuleStates(speeds));
        Robot.swerve.logSetpoints(goalState);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        // Check if the timer has exceeded the trajectory duration
        boolean trajectoryComplete = timer.get() >= trajectory.getTotalTimeSeconds();
    
        // Optional: Add a positional tolerance check for more precise completion
        if (trajectoryComplete) {
            double positionTolerance = 0.05; // meters
            double rotationTolerance = 2.0; // degrees
            
            Translation2d currentPosition = Robot.swerve.getPose().getTranslation();
            Translation2d finalPosition = trajectory.getEndState().pose.getTranslation();
            double positionError = currentPosition.getDistance(finalPosition);
    
            double currentRotation = Robot.swerve.getPose().getRotation().getDegrees();
            double finalRotation = trajectory.getEndState().pose.getRotation().getDegrees();
            double rotationError = Math.abs(currentRotation - finalRotation);
    
            return positionError <= positionTolerance && rotationError <= rotationTolerance;
        }
    
        return false;
    }
    

}
