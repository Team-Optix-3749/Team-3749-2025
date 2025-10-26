package frc.robot.commands.swerve;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.ToPos;

/**
 * The `OnTheFly` command dynamically generates and follows a trajectory
 * from the robot's current position to a designated scoring or movement
 * setpoint.
 * It is responsible for real-time path planning and execution using
 * PathPlanner.
 */
public class OnTheFly extends Command {
    private PathPlannerTrajectory trajectory; // The generated trajectory for movement
    private final Timer timer = new Timer(); // Timer to track trajectory progress
    private final ToPos toPos = new ToPos();
    private final Timer debugTimer = new Timer();

    private Pose2d endpoint = new Pose2d();
    private static boolean currentlyThreading = false; // threads make funky async stuff: this normalizes it

    /**
     * Constructs the OnTheFly command.
     * This command does not require any parameters as it dynamically determines the
     * path.
     */
    public OnTheFly() {
        addRequirements(Robot.swerve);
    }

    /**
     * Initializes the trajectory generation process and starts the timer.
     * If no valid path is generated, the command cancels itself.
     */
    @Override
    public void initialize() {
        // System.out.println("epic fail");

        // Robot.elevator.setState(ElevatorStates.STOW);
        Robot.coralArm.setState(CoralArmConstants.ArmStates.STOW);
        currentlyThreading = true;
        endpoint = Robot.swerve.getPPSetpoint().setpoint;

        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            endpoint = FlippingUtil.flipFieldPose(endpoint);
        }

        new Thread(() -> {
            debugTimer.reset();
            debugTimer.start();
            // Create a new dynamic path generator
            PathPlannerPath path = toPos.generateDynamicPath(
                    Robot.swerve.getPose(), // Current robot position
                    Robot.swerve.getPPSetpoint().approachPoint, // Intermediate approach point
                    Robot.swerve.getPPSetpoint().setpoint, // Final target position
                    Robot.swerve.getMaxDriveSpeed(), // Max driving speed
                    SwerveConstants.ControlConstants.maxAcceleration.get(), // Max acceleration
                    Robot.swerve.getMaxAngularSpeed(), // Max angular speed
                    SwerveConstants.ControlConstants.maxAngularAccelerationRadiansPerSecondSquared // Max angular
                                                                                                   // acceleration
            );
            // System.out.println("path generation took: " + debugTimer.get());
            debugTimer.stop();

            // If path generation fails, stop the command
            if (path == null) {
                Robot.swerve.setIsOTF(false);
                currentlyThreading = false;
                this.cancel();
                return;
            }

            try {
                // Generate the trajectory using the planned path and current robot state
                trajectory = path.generateTrajectory(
                        Robot.swerve.getChassisSpeeds(), // Current velocity
                        Robot.swerve.getRotation2d(), // Current rotation
                        RobotConfig.fromGUISettings()); // PathPlanner robot config settings

                var states = trajectory.getStates();
                if (states.size() >= 2) {
                    // Placeholder: Optionally store second-to-last waypoint
                }
            } catch (IOException | ParseException e) {
                // If an error occurs during trajectory generation, stop execution
                e.printStackTrace();
                Robot.swerve.setIsOTF(false);
                currentlyThreading = false;
                this.cancel();
            }
            currentlyThreading = false;
            timer.reset();
            timer.start();
            // System.out.println(debugTimer.get());
        }).start();
    }

    /**
     * Continuously executes the trajectory-following logic.
     * If the trajectory is invalid or the robot is not in OTF mode, the command
     * cancels itself.
     */
    @Override
    public void execute() {
        if (!Robot.swerve.getPPSetpoint().setpoint.equals(endpoint)) {
            initialize();
        }
        if (currentlyThreading) {
            return;
        }

        if ((trajectory == null || !Robot.swerve.getIsOTF())) {
            this.cancel();
            return;
        }

        // Get the current elapsed time in the trajectory
        double currentTime = timer.get();
        PathPlannerTrajectoryState goalState = trajectory.sample(currentTime); // Get the desired state at the current
                                                                               // time

        // Command the robot to follow the sampled trajectory state
        Robot.swerve.followSample(goalState.pose,
                new Pose2d(
                        goalState.fieldSpeeds.vxMetersPerSecond, // X velocity
                        goalState.fieldSpeeds.vyMetersPerSecond, // Y velocity
                        new Rotation2d(goalState.fieldSpeeds.omegaRadiansPerSecond) // Angular velocity
                ));

    }

    /**
     * Stops the trajectory execution when the command ends.
     *
     * @param interrupted Indicates whether the command was interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        currentlyThreading = false;
        timer.stop();
        
        timer.reset();
        Robot.swerve.setIsOTF(false);

    }

    /**
     * Determines if the trajectory is complete.
     * The command finishes if the trajectory time has elapsed and the robot is
     * within an acceptable error margin.
     *
     * @return true if the trajectory is complete and the robot is close enough to
     *         the target.
     */
    @Override
    public boolean isFinished() {
        return !currentlyThreading && (trajectory == null || (!Robot.swerve.getIsOTF()));
    }

    public double getTime() {
        return timer.get();
    }
}
