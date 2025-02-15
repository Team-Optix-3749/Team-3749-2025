package frc.robot.subsystems.swerve;

import java.util.HashMap;
import java.util.List;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation; //dont remove all unusred importas here read line 42
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ToPosConstants {
    /**
     * The Flipper flips all points in this section, its purpose is for allience
     * change usage.
     * 
     **/
    public static final ChoreoAllianceFlipUtil.Flipper flipper = ChoreoAllianceFlipUtil.getFlipper();

    /**
     * given a pose on blue/red, switch it to red/blue
     * 
     * @param pose intakes a position setpoints and inverts it including all things
     *             such as heading rotation ect to act with flipper
     * 
     **/

    public static Pose2d flipPose(Pose2d pose) {
        Translation2d translation = new Translation2d(flipper.flipX(pose.getX()), flipper.flipY(pose.getY()));
        Rotation2d rotation = new Rotation2d(flipper.flipHeading(pose.getRotation().getRadians()));
        return new Pose2d(translation, rotation);
    }

    /**
     * Uses a hexagon Verticies getter ang makes sure that a hexagon with a
     * safemargin is genorated for detor calculations and setpoints.
     * 
     **/

    public static final class ReefVerticies {

        public static final double positionTolerance = .5; // meters
        public static final double rotationTolerance = 10; // degrees

        // MAKE CONSTANTS
        public static final double SAFE_MARGIN = .95; // Safety margin around the robot.
        public static final double xComponent = Math.cos(Math.toRadians(30));
        public static final double yComponent = Math.sin(Math.toRadians(30));

        private static Translation2d flipIfRed(Translation2d translation) {
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
                return new Translation2d(flipper.flipX(translation.getX()), flipper.flipY(translation.getY()));
            }
            return translation;
        }

        /**
         * UPhysically houses the verticies as a getter
         * 
         **/

        public static List<Translation2d> getHexagonVertices() {

            return List.of(
                    flipIfRed(new Translation2d(3.668 - xComponent * SAFE_MARGIN, 3.520 - yComponent * SAFE_MARGIN)),
                    flipIfRed(new Translation2d(4.5, 3.039 - SAFE_MARGIN)),
                    flipIfRed(new Translation2d(5.332 + xComponent * SAFE_MARGIN, 3.520 - yComponent * SAFE_MARGIN)),
                    flipIfRed(new Translation2d(5.332 + xComponent * SAFE_MARGIN, 4.480 + yComponent * SAFE_MARGIN)),
                    flipIfRed(new Translation2d(4.5, 4.961 + SAFE_MARGIN)),
                    flipIfRed(new Translation2d(3.668 - xComponent * SAFE_MARGIN, 4.480 + yComponent * SAFE_MARGIN)),
                    flipIfRed(new Translation2d(3.668 - xComponent * SAFE_MARGIN, 3.520 - yComponent * SAFE_MARGIN)));
        }
    }

    /**
     * This Setpoints class defines various Pose2d targets on the reef, accounting
     * for offsets (e.g., robot size, orientation). It includes utility methods
     * (like reefTrig, adjustPose, and createApproachPoint) to calculate these
     * positions and generate approach points for different scenarios.
     */
    public static final class Setpoints {

        public static final double approachPointDistance = 0.6;

        /**
         * Assumes a forward vector from the robot front to calculate
         * left/right/backward positions
         */
        public static enum TrigDirection {
            LEFT,
            RIGHT,
            BACKWARD
        }

        /**
         * @param reefPose  the center point of the side of the reef
         * @param direction translate the position left, right, or backward
         */
        public static Pose2d reefTrig(Pose2d reefPose, TrigDirection direction) {

            double angleOffset = 90; // 90 for perpindicular
            double distance = 6.5; // distance between branches, this variable as a whole is essentially the
                                   // "hypotenuse" of the total shift
            double intialOffset = 6.25; // account for the distance from the arm
            double intialAngleOffset = 90; // accounting for the first left and right shfit
            switch (direction) {
                case LEFT:
                    angleOffset = 90;
                    break;

                case RIGHT:
                    angleOffset = -90;
                    break;
                case BACKWARD:
                    distance = 3.5; // instead of moving 6.5 between pipes, move 3 inches away from the reef
                    intialOffset = -6.5; // this takes in the l234 setpoint , so we can return to the center
                    angleOffset = -90;
                    intialAngleOffset = 180;
                    break;
            }

            // Apply an initial 6.25-inch offset based on the robot's orientation
            double xSetup = reefPose.getX()
                    + Math.cos(Math.toRadians(reefPose.getRotation().getDegrees() + intialAngleOffset))
                            * Units.inchesToMeters(intialOffset);
            double ySetup = reefPose.getY()
                    + Math.sin(Math.toRadians(reefPose.getRotation().getDegrees() + intialAngleOffset))
                            * Units.inchesToMeters(intialOffset);
            // Adjust position based on movement direction (left, right, or backward)
            double newX = xSetup
                    + Math.cos(Math.toRadians(reefPose.getRotation().getDegrees() + (angleOffset)))
                            * Units.inchesToMeters(distance);
            double newY = ySetup
                    + Math.sin(Math.toRadians(reefPose.getRotation().getDegrees() + (angleOffset)))
                            * Units.inchesToMeters(distance);

            // Return the updated position with the same rotation
            return new Pose2d(newX, newY, reefPose.getRotation());

        };

        /**
         * Adjusts a given Pose2d based on the robot's length and width to align the
         * front edge with the target.
         *
         * @param x              The x-coordinate of the original position.
         * @param y              The y-coordinate of the original position.
         * @param heading        The robot's heading in radians.
         * @param isCoralStation True if the position is at a coral station (adjusts
         *                       differently).
         * @return The adjusted Pose2d with the new position and same heading.
         */
        public static Pose2d adjustPose(double x, double y, double heading, boolean isCoralStation) {
            // Calculate offsets based on half the robot's dimensions
            double offsetX = (ROBOT_LENGTH / 2) * Math.cos(heading);
            double offsetY = (ROBOT_WIDTH / 2) * Math.sin(heading);

            // If it's a coral station, adjust the pose forward; otherwise, adjust backward
            if (isCoralStation) {
                return new Pose2d(x + offsetX, y + offsetY, new Rotation2d(heading));
            }
            return new Pose2d(x - offsetX, y - offsetY, new Rotation2d(heading));
        }

        /**
         * Creates an approach point slightly behind the given pose to allow smoother
         * movement.
         *
         * @param pose The original pose where the robot is approaching.
         * @return A new Pose2d shifted backward by the approachPointDistance.
         */
        private static Pose2d createApproachPoint(Pose2d pose) {
            Translation2d position = pose.getTranslation();
            Rotation2d heading = pose.getRotation();

            // Calculate an offset distance backward in the direction of the heading
            Translation2d offset = new Translation2d(
                    -approachPointDistance * Math.cos(heading.getRadians()),
                    -approachPointDistance * Math.sin(heading.getRadians()));

            return new Pose2d(position.plus(offset), heading);
        }

        /**
         * Rotates a given Pose2d by a specified number of degrees while keeping its
         * position the same.
         *
         * @param pose    The original pose.
         * @param degrees The amount to rotate the pose by.
         * @return A new Pose2d with the same x and y but rotated by the specified
         *         degrees.
         */
        private static Pose2d rotatePose(Pose2d pose, double degrees) {
            return new Pose2d(
                    pose.getX(),
                    pose.getY(),
                    new Rotation2d(Math.toRadians(pose.getRotation().getDegrees() + degrees)));
        }

        // ======= Robot Dimensions (in meters) =======
        public static final double ROBOT_LENGTH = Units.inchesToMeters(37); // Robot length
        public static final double ROBOT_WIDTH = Units.inchesToMeters(30); // Robot width

        // ======= Coral Station Setpoints =======
        // Positions for placing game elements at the coral stations
        public static Pose2d coralLeft = adjustPose(0.851154, 7.39648, Math.toRadians(-55), true); // Left side of coral
                                                                                                   // station
        public static Pose2d coralRight = adjustPose(0.851154, 0.65532, Math.toRadians(55), true); // Right side of
                                                                                                   // coral station
        public static Pose2d processor = adjustPose(5.987542, -0.00381, Math.toRadians(-90), false); // Processor
                                                                                                     // location

        // ======= Reef Positions =======
        // Positions marking different reef scoring locations
        public static Pose2d reefClose = adjustPose(3.65, 4, Math.toRadians(0), false); // Center front of the reef
        public static Pose2d reefCloseRight = adjustPose(4.07, 3.25, Math.toRadians(60), false); // Right front reef
        public static Pose2d reefFarRight = adjustPose(4.94, 3.25, Math.toRadians(120), false); // Right back reef
        public static Pose2d reefFar = adjustPose(5.35, 4, Math.toRadians(180), false); // Center back of the reef
        public static Pose2d reefFarLeft = adjustPose(4.94, 4.74, Math.toRadians(-120), false); // Left back reef
        public static Pose2d reefCloseLeft = adjustPose(4.07, 4.74, Math.toRadians(-60), false);// Left front reef

        // ======= Reef Scoring Setpoints (Main Scoring Positions) =======
        // Standard scoring positions at the reef
        public static Pose2d aSetpoint = reefTrig(reefClose, TrigDirection.LEFT); // Left of front center reef
        public static Pose2d bSetpoint = reefTrig(reefClose, TrigDirection.RIGHT); // Right of front center reef
        public static Pose2d cSetpoint = reefTrig(reefCloseRight, TrigDirection.LEFT); // Left of front right reef
        public static Pose2d dSetpoint = reefTrig(reefCloseRight, TrigDirection.RIGHT); // Right of front right reef
        public static Pose2d eSetpoint = reefTrig(reefFarRight, TrigDirection.LEFT); // Left of back right reef
        public static Pose2d fSetpoint = reefTrig(reefFarRight, TrigDirection.RIGHT); // Right of back right reef
        public static Pose2d gSetpoint = reefTrig(reefFar, TrigDirection.LEFT); // Left of back center reef
        public static Pose2d hSetpoint = reefTrig(reefFar, TrigDirection.RIGHT); // Right of back center reef
        public static Pose2d iSetpoint = reefTrig(reefFarLeft, TrigDirection.LEFT); // Left of back left reef
        public static Pose2d jSetpoint = reefTrig(reefFarLeft, TrigDirection.RIGHT); // Right of back left reef
        public static Pose2d kSetpoint = reefTrig(reefCloseLeft, TrigDirection.LEFT); // Left of front left reef
        public static Pose2d lSetpoint = reefTrig(reefCloseLeft, TrigDirection.RIGHT); // Right of front left reef

        // ======= Level 1 Scoring Positions (L1) =======
        // Adjusted positions for Level 1 arm scoring at each reef location
        public static Pose2d aL1 = reefTrig(rotatePose(aSetpoint, -90), TrigDirection.BACKWARD); // Level 1 scoring at A
        public static Pose2d bL1 = reefTrig(rotatePose(bSetpoint, -90), TrigDirection.BACKWARD); // Level 1 scoring at B
        public static Pose2d cL1 = reefTrig(rotatePose(cSetpoint, -90), TrigDirection.BACKWARD); // Level 1 scoring at C
        public static Pose2d dL1 = reefTrig(rotatePose(dSetpoint, -90), TrigDirection.BACKWARD); // Level 1 scoring at D
        public static Pose2d eL1 = reefTrig(rotatePose(eSetpoint, -90), TrigDirection.BACKWARD); // Level 1 scoring at E
        public static Pose2d fL1 = reefTrig(rotatePose(fSetpoint, -90), TrigDirection.BACKWARD); // Level 1 scoring at F
        public static Pose2d gL1 = reefTrig(rotatePose(gSetpoint, -90), TrigDirection.BACKWARD); // Level 1 scoring at G
        public static Pose2d hL1 = reefTrig(rotatePose(hSetpoint, -90), TrigDirection.BACKWARD); // Level 1 scoring at H
        public static Pose2d iL1 = reefTrig(rotatePose(iSetpoint, -90), TrigDirection.BACKWARD); // Level 1 scoring at I
        public static Pose2d jL1 = reefTrig(rotatePose(jSetpoint, -90), TrigDirection.BACKWARD); // Level 1 scoring at J
        public static Pose2d kL1 = reefTrig(rotatePose(kSetpoint, -90), TrigDirection.BACKWARD); // Level 1 scoring at K
        public static Pose2d lL1 = reefTrig(rotatePose(lSetpoint, -90), TrigDirection.BACKWARD); // Level 1 scoring at L

        // ======= Reef Sides List =======
        // A list of the central points of each reef side
        public static List<Pose2d> reefSides = List.of(
                reefClose, // Center front of the reef
                reefCloseRight, // Right front reef
                reefCloseLeft, // Left front reef
                reefFar, // Center back of the reef
                reefFarLeft, // Left back reef
                reefFarRight // Right back reef
        );

        /**
         * HashMap mapping each reef side to the possible drive-relative branches.
         * Used for determining the closest reef position when pressing left or right
         * bumpers.
         * Key: Pose2d reef position
         * Value: Integer array representing the closest left (L) and right (R) branch
         * indexes.
         */
        public static HashMap<Pose2d, int[]> driveRelativeBranches = new HashMap<Pose2d, int[]>() {
            {
                put(reefClose, new int[] { 2, 4, 26 }); // Left and right options from reefClose
                put(reefCloseLeft, new int[] { 22, 24, 27 }); // Left and right options from reefCloseLeft
                put(reefCloseRight, new int[] { 6, 8, 28 }); // Left and right options from reefCloseRight
                put(reefFarRight, new int[] { 12, 10,31 }); // Left and right options from reefFarRight
                put(reefFar, new int[] { 16, 14,29 }); // Left and right options from reefFar
                put(reefFarLeft, new int[] { 20, 18,30 }); // Left and right options from reefFarLeft
            }
        };

        // ======= Approach Setpoints =======
        // Approach points for smoother movement before reaching each reef scoring
        // position
        public static Pose2d aApproach = createApproachPoint(aSetpoint); // Approach point for A
        public static Pose2d bApproach = createApproachPoint(bSetpoint); // Approach point for B
        public static Pose2d cApproach = createApproachPoint(cSetpoint); // Approach point for C
        public static Pose2d dApproach = createApproachPoint(dSetpoint); // Approach point for D
        public static Pose2d eApproach = createApproachPoint(eSetpoint); // Approach point for E
        public static Pose2d fApproach = createApproachPoint(fSetpoint); // Approach point for F
        public static Pose2d gApproach = createApproachPoint(gSetpoint); // Approach point for G
        public static Pose2d hApproach = createApproachPoint(hSetpoint); // Approach point for H
        public static Pose2d iApproach = createApproachPoint(iSetpoint); // Approach point for I
        public static Pose2d jApproach = createApproachPoint(jSetpoint); // Approach point for J
        public static Pose2d kApproach = createApproachPoint(kSetpoint); // Approach point for K
        public static Pose2d lApproach = createApproachPoint(lSetpoint); // Approach point for L

        /**
         * Enum representing all preset path planner setpoints for reef and coral
         * station positions.
         * Each setpoint has an associated approach point, which serves as a stopping
         * position before reaching the final setpoint.
         */
        public enum PPSetpoints {
            // ======= Coral Station Setpoints =======
            CORALLEFT(coralLeft, coralLeft), // Coral station (left side)
            CORALRIGHT(coralRight, coralRight), // Coral station (right side)

            // ======= Reef Scoring Setpoints =======
            // Each setpoint represents the final scoring position at the reef
            // The approach point is a temporary stop before reaching the final position
            A(aSetpoint, aApproach), // Final scoring position at A, approach before stopping
            AL1(aL1, aApproach), // Level 1 arm scoring at A, approach before stopping

            B(bSetpoint, bApproach), // Final scoring position at B
            BL1(bL1, bApproach), // Level 1 arm scoring at B

            C(cSetpoint, cApproach), // Final scoring position at C
            CL1(cL1, cApproach), // Level 1 arm scoring at C

            D(dSetpoint, dApproach), // Final scoring position at D
            DL1(dL1, dApproach), // Level 1 arm scoring at D

            E(eSetpoint, eApproach), // Final scoring position at E
            EL1(eL1, eApproach), // Level 1 arm scoring at E

            F(fSetpoint, fApproach), // Final scoring position at F
            FL1(fL1, fApproach), // Level 1 arm scoring at F

            G(gSetpoint, gApproach), // Final scoring position at G
            GL1(gL1, gApproach), // Level 1 arm scoring at G

            H(hSetpoint, hApproach), // Final scoring position at H
            HL1(hL1, hApproach), // Level 1 arm scoring at H

            I(iSetpoint, iApproach), // Final scoring position at I
            IL1(iL1, iApproach), // Level 1 arm scoring at I

            J(jSetpoint, jApproach), // Final scoring position at J
            JL1(jL1, jApproach), // Level 1 arm scoring at J

            K(kSetpoint, kApproach), // Final scoring position at K
            KL1(kL1, kApproach), // Level 1 arm scoring at K

            L(lSetpoint, lApproach), // Final scoring position at L
            LL1(lL1, lApproach), // Level 1 arm scoring at L

            // ======= Nearest Reef Side Setpoints =======
            // These are used to allow the robot to move toward the nearest reef side
            // based on driver input (e.g., moving left or right)
            REEFCLOSE(rotatePose(reefClose, 180), createApproachPoint(reefClose)), // Move to center front reef
            REEFCLOSELEFT(rotatePose(reefCloseLeft, 180), createApproachPoint(reefCloseLeft)), // Move to closest left
                                                                                              // reef
            REEFCLOSERIGHT(rotatePose(reefCloseRight, 180), createApproachPoint(reefCloseRight)), // Move to closest
                                                                                                 // right reef

            REEFFAR(rotatePose(reefFar, 180), createApproachPoint(reefFar)), // Move to center back reef
            REEFFARLEFT(rotatePose(reefFarLeft, 180), createApproachPoint(reefFarLeft)), // Move to closest left back
                                                                                        // reef
            REEFFARRIGHT(rotatePose(reefFarRight, 180), createApproachPoint(reefFarRight)); // Move to closest right back
                                                                                           // reef

            // ======= Variables for Each Setpoint =======
            public Pose2d setpoint; // The final scoring position or movement target
            public Pose2d approachPoint; // The stopping position before the final setpoint for smooth movement

            /**
             * Constructor for PPSetpoints, setting up default setpoints and their approach
             * points.
             * The approach point is used to slow down and ensure a controlled stop before
             * reaching the final position.
             * If the robot is on the red alliance, setpoints are flipped accordingly.
             *
             * @param setpoint      The final target position for scoring or movement.
             * @param approachPoint The position before the final target, ensuring smooth
             *                      movement.
             */
            private PPSetpoints(Pose2d setpoint, Pose2d approachPoint) {
                this.setpoint = setpoint;
                this.approachPoint = approachPoint;

                // Flip setpoints if on the red alliance
                if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
                    this.setpoint = flipPose(setpoint);
                    this.approachPoint = flipPose(approachPoint);
                }
            }
        }

    }
}