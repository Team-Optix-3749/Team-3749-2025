package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.buttons.JoystickIO;
import frc.robot.buttons.ToPosTriggers;
import frc.robot.buttons.ButtonBoard.ScoringMode;

/**
 * This class generates dynamic paths for a robot to move from one pose to
 * another
 * while avoiding obstacles, specifically a hexagonal area defined by safety
 * margins.
 */
public class ToPos {

    private Timer debugTimer = new Timer();
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
        debugTimer.reset();
        debugTimer.start();
        if (initialPose == null || finalPose == null || approachPoint == null) {
            throw new IllegalArgumentException("Pose arguments cannot be null!");
        }

        if (initialPose.equals(finalPose) || initialPose.equals(approachPoint) || 
        (approachPoint.equals(finalPose) && !ToPosTriggers.isCoralSupplier.getAsBoolean())) { //coral station approachpoint==finalpose only
            return null; // Prevents unnecessary movement
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

        // 🚨 New Check: Ensure at least 2 waypoints before creating the path
        if (waypoints.size() < 2) {
            System.out.println("epic waypoint size fail");
            return null;
        }
        // System.out.println("path optimization took: " + debugTimer.get());
        debugTimer.stop();

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

        // Calculate clockwise and counterclockwise distances based on # verticies
        // traveled
        int clockwiseDistance = (endVertexIndex - startVertexIndex
                + ToPosConstants.ReefVerticies.getHexagonVertices().size())
                % ToPosConstants.ReefVerticies.getHexagonVertices().size();
        int counterclockwiseDistance = (startVertexIndex - endVertexIndex
                + ToPosConstants.ReefVerticies.getHexagonVertices().size())
                % ToPosConstants.ReefVerticies.getHexagonVertices().size();

        // Choose the direction based on shorter angular displacement
        boolean goClockwise = clockwiseDistance < counterclockwiseDistance;

        // Adjust to move in the correct global direction
        int currentIndex = startVertexIndex;
        while (true) {
            Translation2d vertex = ToPosConstants.ReefVerticies.getHexagonVertices().get(currentIndex);
            detourWaypoints.add(new Waypoint(vertex, vertex, vertex));

            // Stop when reaching the end vertex
            if (currentIndex == endVertexIndex) {
                break;
            }

            // Move to the next vertex in the correct direction
            currentIndex = goClockwise
                    ? (currentIndex + 1) % ToPosConstants.ReefVerticies.getHexagonVertices().size()
                    : (currentIndex - 1 + ToPosConstants.ReefVerticies.getHexagonVertices().size())
                            % ToPosConstants.ReefVerticies.getHexagonVertices().size();
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
        for (int i = 0; i < ToPosConstants.ReefVerticies.getHexagonVertices().size(); i++) {
            Translation2d vertex1 = ToPosConstants.ReefVerticies.getHexagonVertices().get(i);
            Translation2d vertex2 = ToPosConstants.ReefVerticies.getHexagonVertices()
                    .get((i + 1) % ToPosConstants.ReefVerticies.getHexagonVertices().size());
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
        for (int i = 0; i < ToPosConstants.ReefVerticies.getHexagonVertices().size() - 1; i++) {
            double distance = point.getDistance(ToPosConstants.ReefVerticies.getHexagonVertices().get(i));

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
                !(closestVertex1 == 0 && closestVertex2 == ToPosConstants.ReefVerticies.getHexagonVertices().size() - 2)
                &&
                !(closestVertex2 == 0
                        && closestVertex1 == ToPosConstants.ReefVerticies.getHexagonVertices().size() - 2)) {
            closestVertex2 = (closestVertex1 + 1) % ToPosConstants.ReefVerticies.getHexagonVertices().size();
        }

        // Step 3: Evaluate alignment with the path direction.
        Translation2d vertex1 = ToPosConstants.ReefVerticies.getHexagonVertices().get(closestVertex1);
        Translation2d vertex2 = ToPosConstants.ReefVerticies.getHexagonVertices().get(closestVertex2);

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

        // Fix floating-point precision issue in heading difference
        double headingDifference = Math.abs((headingFinal - headingInit) % 360);
        if (headingDifference > 180) {
            headingDifference = 360 - headingDifference;
        }

        // Check if the heading difference is within ±20 degrees
        if (distance < 1.0 && headingDifference <= 20) {
            if (waypoints.size() > 2) {
                waypoints.subList(1, waypoints.size() - 1).clear();
            }
        }

        double threshold = 0.05; // Minimum distance between waypoints (5 cm).
        for (int i = 1; i < waypoints.size(); i++) {
            Translation2d current = waypoints.get(i).anchor();
            Translation2d previous = waypoints.get(i - 1).anchor();
            if (current.getDistance(previous) < threshold) {
                waypoints.remove(i);
                i--; // Adjust index after removal.
            }
        }
        if (waypoints.size() < 2) {
            waypoints.clear();
            waypoints.add(new Waypoint(initialTranslation, initialTranslation, initialTranslation));
            waypoints.add(new Waypoint(finalTranslation, finalTranslation, finalTranslation));
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

        double endingAlignment = (finalVertexToEndVector.getX() * secondToFinalVertexToFinalVertexVector.getX()
                + finalVertexToEndVector.getY() * secondToFinalVertexToFinalVertexVector.getY());
        if (endingAlignment < 0) {

            waypoints.remove(waypoints.size() - 2);
        }
    }

    /**
     * Sets the setpoint to the closest reef branch, determining whether to move
     * left or right.
     * Adjusts the setpoint for Level 1 scoring if applicable.
     * 
     * @param isLeftBranch If true, selects the left branch; otherwise, selects the
     *                     right branch.
     */
    public static void setSetpointByClosestReefBranch(boolean isLeftBranch) {
        int branchIndex = isLeftBranch ? 0 : 1; // Selects the left (0) or right (1) branch
        if(JoystickIO.getButtonBoard().getScoringMode()==null)
        {
            JoystickIO.getButtonBoard().setScoringMode(ScoringMode.ALGAE);
        }
        if(JoystickIO.getButtonBoard().getScoringMode().equals(ScoringMode.ALGAE))
        {
            branchIndex = 2;
        }
        int shouldOffsetForL1 = 0; // Offset for Level 1 scoring, default is 0

        // Find the closest reef side to the robot's current position
        Pose2d closestSide = Robot.swerve.getPose().nearest(ToPosConstants.Setpoints.reefSides);

        // If scoring at Level 1, adjust the offset accordingly
        if (JoystickIO.getButtonBoard().getScoringMode() == ScoringMode.L1) {
            shouldOffsetForL1 = 1;
        }
        //possible race condition if scoring mode somehow changes that fast

        // Iterate through the reef branch mappings to set the correct setpoint
        for (Pose2d side : ToPosConstants.Setpoints.driveRelativeBranches.keySet()) {
            if (closestSide.equals(side)) {
                Robot.swerve.setPPSetpointIndex(
                        ToPosConstants.Setpoints.driveRelativeBranches.get(side)[branchIndex] + shouldOffsetForL1);
                break; // Exit loop once the correct setpoint is found
            }
        }
    }

}