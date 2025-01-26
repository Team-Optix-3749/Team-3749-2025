package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ToPos {
    private static final ExecutorService executorService = Executors.newSingleThreadExecutor();

    /**
     * Asynchronously generates a dynamic path using PathPlanner.
     *
     * @param initialPose            Starting pose
     * @param finalPose              Target pose
     * @param maxVelocity            Maximum velocity constraint
     * @param maxAcceleration        Maximum acceleration constraint
     * @param maxAngularVelocity     Maximum angular velocity constraint
     * @param maxAngularAcceleration Maximum angular acceleration constraint
     * @param initialSpeed           Initial speed of the robot
     * @param initialHeading         Initial heading of the robot
     */
    public static void generateDynamicPathAsync(
            Pose2d initialPose,
            Pose2d finalPose,
            double maxVelocity,
            double maxAcceleration,
            double maxAngularVelocity,
            double maxAngularAcceleration,
            double initialSpeed,
            double initialHeading) {
        executorService.submit(() -> {
            System.out.println("Starting dynamic path generation...");
            PathPlannerPath path = generateDynamicPath(
                    initialPose, finalPose, maxVelocity, maxAcceleration, maxAngularVelocity, maxAngularAcceleration, initialSpeed, initialHeading);
            if (path != null) {
                System.out.println("Dynamic path generated successfully.");
            } else {
                System.out.println("Dynamic path generation failed.");
            }
        });
    }

    /**
     * Generates a dynamic path avoiding obstacles using PathPlanner.
     *
     * @param initialPose            Starting pose
     * @param finalPose              Target pose
     * @param maxVelocity            Maximum velocity constraint
     * @param maxAcceleration        Maximum acceleration constraint
     * @param maxAngularVelocity     Maximum angular velocity constraint
     * @param maxAngularAcceleration Maximum angular acceleration constraint
     * @param initialSpeed           Initial speed of the robot
     * @param initialHeading         Initial heading of the robot
     * @return A PathPlannerPath if successful, null otherwise
     */
    public static PathPlannerPath generateDynamicPath(
            Pose2d initialPose,
            Pose2d finalPose,
            double maxVelocity,
            double maxAcceleration,
            double maxAngularVelocity,
            double maxAngularAcceleration,
            double initialSpeed,
            double initialHeading) {
        List<Waypoint> waypoints = new ArrayList<>();

        // Define initial waypoint
        Translation2d start = initialPose.getTranslation();
        Translation2d end = finalPose.getTranslation();
        waypoints.add(new Waypoint(start, start, start.plus(new Translation2d(0.5, 0.0))));

        // Add a waypoint 1 meter before the final pose for heading alignment
        Translation2d approachPoint = end.minus(new Translation2d(
                1.0 * Math.cos(finalPose.getRotation().getRadians()),
                1.0 * Math.sin(finalPose.getRotation().getRadians())));
        waypoints.add(new Waypoint(approachPoint, approachPoint, approachPoint.plus(new Translation2d(0.5, 0.0))));

        // Optimize for maximum speed time by directly heading to the final pose
        waypoints.add(new Waypoint(end, end, end.minus(new Translation2d(0.5, 0.0))));

        if (waypoints.size() < 2) {
            System.out.println("Error: Not enough waypoints for path generation.");
            return null;
        }

        // Set path constraints
        PathConstraints constraints = new PathConstraints(maxVelocity, maxAcceleration, maxAngularVelocity, maxAngularAcceleration);

        try {
            GoalEndState goalEndState = new GoalEndState(0.0, finalPose.getRotation());
            PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, goalEndState);
            path.preventFlipping = true;
            return path;
        } catch (Exception e) {
            System.out.println("PathPlanner Error: " + e.getMessage());
            return null;
        }
    }

    /**
     * Checks if a direct path is clear of obstacles.
     *
     * @param start Starting point
     * @param end   Ending point
     * @return True if the path is clear, false otherwise
     */
    private static boolean isPathClear(Translation2d start, Translation2d end) {
        // Placeholder logic for obstacle detection
        // Replace with real sensor or map-based obstacle detection
        return true; // Assume no obstacles for now
    }

    /**
     * Calculates detour points to avoid obstacles between start and end.
     *
     * @param start Starting point
     * @param end   Ending point
     * @return List of detour points
     */
    private static List<Translation2d> calculateDetours(Translation2d start, Translation2d end) {
        List<Translation2d> detours = new ArrayList<>();

        // Example logic to calculate a midpoint detour
        double midX = (start.getX() + end.getX()) / 2;
        double midY = (start.getY() + end.getY()) / 2 + 1.0; // Offset to avoid obstacles
        detours.add(new Translation2d(midX, midY));

        return detours;
    }

    /**
     * Stops the thread for path generation when no longer needed.
     */
    public static void shutdown() {
        executorService.shutdownNow();
        System.out.println("Path generation service stopped.");
    }
}
