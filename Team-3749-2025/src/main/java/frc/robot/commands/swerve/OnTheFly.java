package frc.robot.commands.swerve;

import java.io.IOException;
import org.json.simple.parser.ParseException;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.ToPos;

public class OnTheFly extends Command {
    private PathPlannerTrajectory trajectory;
    private final Timer timer = new Timer();
    private static final double positionTolerance = .5; // meters
    private static final double rotationTolerance = 10; // degrees
    private PathPlannerTrajectoryState secondToLastWaypoint = null;
    private boolean hasTriggeredSecondLastAction = false;

    public OnTheFly() {
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        hasTriggeredSecondLastAction = false; // Reset flag for each execution

        ToPos toPos = new ToPos();
        PathPlannerPath path = toPos.generateDynamicPath(
                Robot.swerve.getPose(),
                Robot.swerve.getPPSetpoint().approachPoint,
                Robot.swerve.getPPSetpoint().setpoint,
                Robot.swerve.getMaxDriveSpeed(),
                SwerveConstants.DriveConstants.maxAccelerationMetersPerSecondSquared,
                Robot.swerve.getMaxAngularSpeed(),
                SwerveConstants.DriveConstants.maxAngularAccelerationRadiansPerSecondSquared);

        if (path == null) {
            System.out.println("Error: Failed to generate path. Ending OnTheFly command.");
            Robot.swerve.isOTF = false;
            this.cancel();
            return;
        }

        try {
            trajectory = path.generateTrajectory(
                    Robot.swerve.getChassisSpeeds(),
                    Robot.swerve.getRotation2d(),
                    RobotConfig.fromGUISettings());

            var states = trajectory.getStates();
            if (states.size() >= 2) {
                secondToLastWaypoint = states.get(states.size() - 2);
            }
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            Robot.swerve.isOTF = false;
            this.cancel();
        }
    }

    @Override
    public void execute() {
        if (trajectory == null || !Robot.swerve.isOTF) {
            this.cancel();
            return;
        }

        double currentTime = timer.get();
        PathPlannerTrajectoryState goalState = trajectory.sample(currentTime);

        Robot.swerve.followSample(goalState.pose,
                new Pose2d(
                        goalState.fieldSpeeds.vxMetersPerSecond,
                        goalState.fieldSpeeds.vyMetersPerSecond,
                        new Rotation2d(goalState.fieldSpeeds.omegaRadiansPerSecond)));

        if (secondToLastWaypoint != null && !hasTriggeredSecondLastAction) {
            System.out.println("Checking second-to-last waypoint...");
            if (withinSetpointTolerance(secondToLastWaypoint.pose)) {
                System.out.println("Triggering custom action at second-to-last waypoint!");
                hasTriggeredSecondLastAction = true;
                triggerCustomAction();
            }
        }

        if (isFinished()) {
            this.end(true);
            Robot.swerve.isOTF = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        if (trajectory == null) {
            return true;
        }
        boolean trajectoryComplete = timer.get() >= trajectory.getTotalTimeSeconds();
        if (trajectoryComplete) {
            return withinSetpointTolerance(trajectory.getEndState().pose);
        }
        return false;
    }

    private boolean withinSetpointTolerance(Pose2d setpoint) {
        double xError = Math.abs(setpoint.relativeTo(Robot.swerve.getPose()).getX());
        double yError = Math.abs(setpoint.relativeTo(Robot.swerve.getPose()).getY());
        double thetaError = setpoint.relativeTo(Robot.swerve.getPose()).getRotation().getDegrees();

        return xError < positionTolerance && yError < positionTolerance && thetaError < rotationTolerance;
    }

    private void triggerCustomAction() {
        System.out.println("Reached second-to-last waypoint! Running custom action...");
     
    }
}
