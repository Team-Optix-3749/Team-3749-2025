package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ToPos {

    public static PathPlannerPath generateDynamicPath(
            Pose2d initialPose,
            Pose2d finalPose,
            double maxVelocity,
            double maxAcceleration,
            double maxAngularVelocity,
            double maxAngularAcceleration,
            Translation2d obstacleCenter,
            double obstacleRadius
    ) {
        List<Waypoint> waypoints = new ArrayList<>();
        waypoints.add(new Waypoint(initialPose.getTranslation(), initialPose.getTranslation(), initialPose.getTranslation()));

        // Check if the direct path intersects the obstacle
        if (intersectsObstacle(initialPose.getTranslation(), finalPose.getTranslation(), obstacleCenter, obstacleRadius)) {
            System.out.println("Obstacle detected! Calculating detour...");

            // Generate a smooth Bezier detour around the obstacle
            List<Translation2d> bezierControlPoints = calculateBezierControlPoints(
                initialPose.getTranslation(), finalPose.getTranslation(), obstacleCenter, obstacleRadius
            );

            for (Translation2d controlPoint : bezierControlPoints) {
                waypoints.add(new Waypoint(controlPoint, controlPoint, controlPoint));
            }
            System.out.println("Added Bezier control points: " + bezierControlPoints);
        } else {
            System.out.println("No obstacle detected. Using direct path.");
        }

        // Add final waypoint
        waypoints.add(new Waypoint(finalPose.getTranslation(), finalPose.getTranslation(), finalPose.getTranslation()));

        // Generate and return the trajectory
        try {
            PathConstraints constraints = new PathConstraints(maxVelocity, maxAcceleration, maxAngularVelocity, maxAngularAcceleration);
            PathPlannerPath path = new PathPlannerPath(
                    waypoints, constraints, null, new GoalEndState(0.0, finalPose.getRotation()));

            path.preventFlipping = true;
            System.out.println("Path generation complete. Total waypoints: " + waypoints.size());
            return path;
        } catch (Exception e) {
            System.out.println("Error during trajectory generation: " + e.getMessage());
            return null;
        }
    }

    private static boolean intersectsObstacle(Translation2d start, Translation2d end, Translation2d center, double radius) {
        Translation2d direction = end.minus(start).div(start.getDistance(end));
        Translation2d toObstacle = center.minus(start);

        double projection = toObstacle.getX() * direction.getX() + toObstacle.getY() * direction.getY();
        Translation2d closestPoint = start.plus(direction.times(projection));

        double distanceToObstacle = closestPoint.getDistance(center);
        return distanceToObstacle < radius && projection > 0 && projection < start.getDistance(end);
    }

    private static List<Translation2d> calculateBezierControlPoints(Translation2d start, Translation2d end, Translation2d center, double radius) {
        List<Translation2d> controlPoints = new ArrayList<>();

        Translation2d direction = end.minus(start).div(start.getDistance(end));
        Translation2d perpendicular = new Translation2d(-direction.getY(), direction.getX());

        // Generate two detour control points
        Translation2d detour1 = center.plus(perpendicular.times(radius + 0.5));
        Translation2d detour2 = center.minus(perpendicular.times(radius + 0.5));

        controlPoints.add(detour1);
        controlPoints.add(center); // Optional: Add the obstacle center as an intermediate point
        controlPoints.add(detour2);

        return controlPoints;
    }
}
