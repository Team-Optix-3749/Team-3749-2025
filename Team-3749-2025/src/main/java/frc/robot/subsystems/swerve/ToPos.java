package frc.robot.subsystems.swerve;

import java.util.*;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.*;

/**
 * This class generates dynamic paths for a robot to move from one pose to
 * another
 * while avoiding obstacles, specifically a hexagonal area defined by safety
 * margins.
 */
public class ToPos {

    /**
     * Generates a dynamic path for the robot from an initial pose to a final pose.
     *
     * @param initialPose            The starting position and orientation of the
     *                               robot.
     * @param finalPose              The target position and orientation of the
     *                               robot.
     * @param maxVelocity            Maximum velocity of the robot (m/s).
     * @param maxAcceleration        Maximum acceleration of the robot (m/s^2).
     * @param maxAngularVelocity     Maximum angular velocity of the robot (rad/s).
     * @param maxAngularAcceleration Maximum angular acceleration of the robot
     *                               (rad/s^2).
     * @return A PathPlannerPath containing waypoints for the robot to follow.
     */
    public PathPlannerPath generateDynamicPath(Pose2d initialPose, Pose2d approachPoint, Pose2d finalPose,
            double maxVelocity, double maxAcceleration, double maxAngularVelocity, double maxAngularAcceleration) {
        if (initialPose == null) {
            throw new IllegalArgumentException("Initial pose cannot be null!");
        }

        List<Waypoint> waypoints = new ArrayList<>();

        waypoints.add(
                new Waypoint(initialPose.getTranslation(), initialPose.getTranslation(), initialPose.getTranslation()));
        waypoints.addAll(generateDetourWaypoints(initialPose.getTranslation(), approachPoint.getTranslation()));

        waypoints.add(
                new Waypoint(approachPoint.getTranslation(), finalPose.getTranslation(), finalPose.getTranslation()));

        removeRedundantWaypoints(waypoints, initialPose, finalPose);
        removeExtraStartVertex(waypoints);
        removeExtraEndVertex(waypoints);

        return new PathPlannerPath(waypoints,
                new PathConstraints(maxVelocity, maxAcceleration, maxAngularVelocity, maxAngularAcceleration), null,
                new GoalEndState(0.0, finalPose.getRotation()));
    }

    /**
     * Generates detour waypoints to avoid obstacles between two points.
     *
     * @param start The starting point.
     * @param end   The target point.
     * @return A list of detour waypoints.
     */
    private List<Waypoint> generateDetourWaypoints(Translation2d start, Translation2d end) {
        List<Waypoint> detourWaypoints = new ArrayList<>();

        if (!isPathIntersectingObstacle(start, end)) {
            detourWaypoints.add(new Waypoint(start, start, end));
            return detourWaypoints;
        }
        // Use the path direction to determine the best vertices
        int startVertexIndex = findClosestHexagonVertex(start, start, end);
        int endVertexIndex = findClosestHexagonVertex(end, start, end);

        // System.out.println("Start: " + startVertexIndex + "\nEnd: " +
        // endVertexIndex);

        // Calculate clockwise and counterclockwise distances based on # verticies
        // traveled
        int clockwiseDistance = (endVertexIndex - startVertexIndex
                + ToPosConstants.ReefVerticies.HEXAGON_VERTICES.size())
                % ToPosConstants.ReefVerticies.HEXAGON_VERTICES.size();
        int counterclockwiseDistance = (startVertexIndex - endVertexIndex
                + ToPosConstants.ReefVerticies.HEXAGON_VERTICES.size())
                % ToPosConstants.ReefVerticies.HEXAGON_VERTICES.size();

        // Choose the direction based on shorter angular displacement
        boolean goClockwise = clockwiseDistance < counterclockwiseDistance;

        // Adjust to move in the correct global direction
        int currentIndex = startVertexIndex;
        while (true) {
            Translation2d vertex = ToPosConstants.ReefVerticies.HEXAGON_VERTICES.get(currentIndex);
            detourWaypoints.add(new Waypoint(vertex, vertex, vertex));

            // Stop when reaching the end vertex
            if (currentIndex == endVertexIndex) {
                break;
            }

            // Move to the next vertex in the correct direction
            currentIndex = goClockwise
                    ? (currentIndex + 1) % ToPosConstants.ReefVerticies.HEXAGON_VERTICES.size()
                    : (currentIndex - 1 + ToPosConstants.ReefVerticies.HEXAGON_VERTICES.size())
                            % ToPosConstants.ReefVerticies.HEXAGON_VERTICES.size();
        }

        return detourWaypoints;
    }

    /**
     * Checks if a path between two points intersects any obstacle.
     *
     * @param start The starting point.
     * @param end   The target point.
     * @return True if the path intersects an obstacle, false otherwise.
     */
    private boolean isPathIntersectingObstacle(Translation2d start, Translation2d end) {
        for (int i = 0; i < ToPosConstants.ReefVerticies.HEXAGON_VERTICES.size(); i++) {
            Translation2d vertex1 = ToPosConstants.ReefVerticies.HEXAGON_VERTICES.get(i);
            Translation2d vertex2 = ToPosConstants.ReefVerticies.HEXAGON_VERTICES
                    .get((i + 1) % ToPosConstants.ReefVerticies.HEXAGON_VERTICES.size());
            if (doLinesIntersect(start, end, vertex1, vertex2)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Determines if two line segments intersect.
     *
     * @param p1 First point of the first line segment.
     * @param p2 Second point of the first line segment.
     * @param q1 First point of the second line segment.
     * @param q2 Second point of the second line segment.
     * @return True if the lines intersect, false otherwise.
     */
    private boolean doLinesIntersect(Translation2d p1, Translation2d p2, Translation2d q1, Translation2d q2) {
        double det = (q2.getX() - q1.getX()) * (p2.getY() - p1.getY())
                - (q2.getY() - q1.getY()) * (p2.getX() - p1.getX());
        if (det == 0)
            return false;

        double t = ((q1.getY() - p1.getY()) * (q2.getX() - q1.getX())
                - (q1.getX() - p1.getX()) * (q2.getY() - q1.getY())) / det;
        double u = ((q1.getY() - p1.getY()) * (p2.getX() - p1.getX())
                - (q1.getX() - p1.getX()) * (p2.getY() - p1.getY())) / det;

        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    /**
     * Finds the single best vertex of the hexagon for a given point by evaluating
     * the two closest vertices.
     *
     * @param point     The point to evaluate.
     * @param pathStart The start point of the path.
     * @param pathEnd   The end point of the path.
     * @return The index of the best vertex.
     */
    private int findClosestHexagonVertex(Translation2d point, Translation2d pathStart, Translation2d pathEnd) {
        int closestVertex1 = -1, closestVertex2 = -1;
        double minDistance1 = Double.MAX_VALUE, minDistance2 = Double.MAX_VALUE;

        // Step 1: Find the two closest vertices to the given point.
        for (int i = 0; i < ToPosConstants.ReefVerticies.HEXAGON_VERTICES.size() - 1; i++) {
            double distance = point.getDistance(ToPosConstants.ReefVerticies.HEXAGON_VERTICES.get(i));

            if (distance < minDistance1) {
                // Update second closest
                minDistance2 = minDistance1;
                closestVertex2 = closestVertex1;

                // Update closest
                minDistance1 = distance;
                closestVertex1 = i;
            } else if (distance < minDistance2) {
                // Update only the second closest
                minDistance2 = distance;
                closestVertex2 = i;
            }
        }

        // Step 2: Ensure consecutive vertices.
        // Adjust for wrap-around (consecutive vertices should form a hexagon edge).
        if (Math.abs(closestVertex1 - closestVertex2) != 1 &&
                !(closestVertex1 == 0 && closestVertex2 == ToPosConstants.ReefVerticies.HEXAGON_VERTICES.size() - 2) &&
                !(closestVertex2 == 0 && closestVertex1 == ToPosConstants.ReefVerticies.HEXAGON_VERTICES.size() - 2)) {

            System.out.println("MATH MISTAKE: " + closestVertex1 + ", " + closestVertex2);
            closestVertex2 = (closestVertex1 + 1) % ToPosConstants.ReefVerticies.HEXAGON_VERTICES.size();
        }

        // Step 3: Evaluate alignment with the path direction.
        Translation2d vertex1 = ToPosConstants.ReefVerticies.HEXAGON_VERTICES.get(closestVertex1);
        Translation2d vertex2 = ToPosConstants.ReefVerticies.HEXAGON_VERTICES.get(closestVertex2);

        // the vector representing a translation from the start to the end
        Translation2d pathDirection = new Translation2d(pathEnd.getX() - pathStart.getX(),
                pathEnd.getY() - pathStart.getY());

        // Normalize path direction to 0-1
        double pathLength = Math
                .sqrt(pathDirection.getX() * pathDirection.getX() + pathDirection.getY() * pathDirection.getY());
        if (pathLength > 0) {
            pathDirection = new Translation2d(pathDirection.getX() / pathLength, pathDirection.getY() / pathLength);
        }

        // Calculate vectors from the point to the vertices
        Translation2d toVertex1 = new Translation2d(vertex1.getX() - point.getX(), vertex1.getY() - point.getY());
        Translation2d toVertex2 = new Translation2d(vertex2.getX() - point.getX(), vertex2.getY() - point.getY());

        // Dot products to measure alignment
        double alignment1 = (toVertex1.getX() * pathDirection.getX() + toVertex1.getY() * pathDirection.getY());
        double alignment2 = (toVertex2.getX() * pathDirection.getX() + toVertex2.getY() * pathDirection.getY());

        // Step 4: Calculate distances to the path.
        double distanceToPath1 = calculatePointToLineDistance(vertex1, pathStart, pathEnd);
        double distanceToPath2 = calculatePointToLineDistance(vertex2, pathStart, pathEnd);

        // Step 5: Choose the best vertex.
        if (alignment1 < 0 && alignment2 >= 0) {
            if (point.equals(pathEnd)) {
                return closestVertex1;
            }

            return closestVertex2; // Vertex 1 misaligned, choose Vertex 2
        } else if (alignment2 < 0 && alignment1 >= 0) {
            if (point.equals(pathEnd)) {

                return closestVertex2;

            }

            return closestVertex1; // Vertex 2 misaligned, choose Vertex 1
        } else {

            int chosenVertex = distanceToPath1 < distanceToPath2 ? closestVertex1 : closestVertex2;
            // Both vertices are aligned; choose the one closer to the path

            return chosenVertex;
        }
    }

    /**
     * Calculates the perpendicular distance from a point to a line segment.
     *
     * @param point     The point to check.
     * @param lineStart The start of the line segment.
     * @param lineEnd   The end of the line segment.
     * @return The perpendicular distance from the point to the line segment.
     */
    private double calculatePointToLineDistance(Translation2d point, Translation2d lineStart, Translation2d lineEnd) {
        double x0 = point.getX();
        double y0 = point.getY();
        double x1 = lineStart.getX();
        double y1 = lineStart.getY();
        double x2 = lineEnd.getX();
        double y2 = lineEnd.getY();

        // Line segment vector (x2 - x1, y2 - y1)
        double dx = x2 - x1;
        double dy = y2 - y1;

        // Handle case where lineStart and lineEnd are the same point
        if (dx == 0 && dy == 0) {
            return point.getDistance(lineStart);
        }

        // Calculate the projection of (x0 - x1, y0 - y1) onto the line segment vector
        double t = ((x0 - x1) * dx + (y0 - y1) * dy) / (dx * dx + dy * dy);

        // Clamp t to the range [0, 1] to stay within the line segment
        t = Math.max(0, Math.min(1, t));

        // Closest point on the line segment
        double closestX = x1 + t * dx;
        double closestY = y1 + t * dy;

        // Return the distance from the point to the closest point on the line
        return Math.sqrt((x0 - closestX) * (x0 - closestX) + (y0 - closestY) * (y0 - closestY));
    }

    /**
     * Removes redundant waypoints that are too close to each other or move the
     * wrong direction as well as removes all waypoints if distance and heading are
     * close and the same respectively.
     *
     * @param waypoints  The list of waypoints to be cleaned.
     * @param initalPose The pose2d where robot is at
     * @param fianlPose  The list of waypoints to be cleaned.
     */

    private void removeRedundantWaypoints(List<Waypoint> waypoints, Pose2d initialPose, Pose2d finalPose) {
        Translation2d initialTranslation = initialPose.getTranslation();
        Translation2d finalTranslation = finalPose.getTranslation();
        double distance = initialTranslation.getDistance(finalTranslation);
        double headingInit = initialPose.getRotation().getDegrees();
        double headingFinal = finalPose.getRotation().getDegrees();
        System.out.println("all of the things for removing the points");
        System.out.println(distance);
        System.out.println(headingFinal - headingInit);
        // Compute the smallest difference between angles, considering wrap-around at
        // 360 degrees
        double headingDifference = Math.abs((headingFinal - headingInit + 180) % 360 - 180);

        // Check if the heading difference is within ±10 degrees
        if (distance < 1.0 && headingDifference <= 10) {
            System.out.println("called");
            if (waypoints.size() > 2) {
                waypoints.subList(1, waypoints.size() - 1).clear();
            }
        }

        double threshold = 0.05; // Minimum distance between waypoints to consider them unique (5 cm).
        for (int i = 1; i < waypoints.size(); i++) {
            Translation2d current = waypoints.get(i).anchor();
            Translation2d previous = waypoints.get(i - 1).anchor();
            if (current.getDistance(previous) < threshold) {
                waypoints.remove(i);
                i--; // Adjust index after removal.
            }
        }

    }

    /**
     * Removes overshooting wrong start hexagon vertex
     *
     * @param waypoints The list of waypoints to be cleaned.
     */
    private void removeExtraStartVertex(List<Waypoint> waypoints) {
        if (waypoints.size() < 4) {
            return;
        }
        Translation2d start = waypoints.get(0).anchor();
        Translation2d firstVertex = waypoints.get(1).anchor();
        Translation2d secondVertex = waypoints.get(2).anchor();
        Translation2d startToFirstVertexVector = firstVertex.minus(start);
        Translation2d firstVertexToSecondVertexVector = secondVertex.minus(firstVertex);

        double startingAlignment = (startToFirstVertexVector.getX() * firstVertexToSecondVertexVector.getX()
                + startToFirstVertexVector.getY() * firstVertexToSecondVertexVector.getY());

        if (startingAlignment < 0) {

            waypoints.remove(1);
        }
    }

    /**
     * Removes overshooting wrong end hexagon vertex
     *
     * @param waypoints The list of waypoints to be cleaned.
     */
    private void removeExtraEndVertex(List<Waypoint> waypoints) {
        if (waypoints.size() < 4) {
            return;
        }

        Translation2d end = waypoints.get(waypoints.size() - 1).anchor();
        Translation2d finalVertex = waypoints.get(waypoints.size() - 2).anchor();
        Translation2d secondToFinalVertex = waypoints.get(waypoints.size() - 3).anchor();

        Translation2d finalVertexToEndVector = end.minus(finalVertex);
        Translation2d secondToFinalVertexToFinalVertexVector = finalVertex.minus(secondToFinalVertex);
        System.out.println("Vectors");
        System.out.println(finalVertexToEndVector.toString());
        System.out.println(secondToFinalVertexToFinalVertexVector);

        double endingAlignment = (finalVertexToEndVector.getX() * secondToFinalVertexToFinalVertexVector.getX()
                + finalVertexToEndVector.getY() * secondToFinalVertexToFinalVertexVector.getY());
        System.out.println("alignment and size");
        System.out.println(endingAlignment);
        System.out.println(waypoints.size());
        System.out.println("waypoints");
        System.out.println(waypoints.get(waypoints.size() - 1).anchor().toString());
        System.out.println(waypoints.get(waypoints.size() - 2).anchor().toString());
        System.out.println(waypoints.get(waypoints.size() - 3).anchor().toString());
        System.out.println(waypoints.get(waypoints.size() - 4).anchor().toString());
        if (endingAlignment < 0) {

            waypoints.remove(waypoints.size() - 2);
        }
    }

}