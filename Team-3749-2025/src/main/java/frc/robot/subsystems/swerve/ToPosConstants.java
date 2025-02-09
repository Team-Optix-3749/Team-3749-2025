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

    // private final double hexOffset = SwerveConstants.DriveConstants.trackWidth /
    // 2.0;
    // private final double hexOffset = SwerveConstants.DriveConstants.trackWidth /
    // 2.0;

    //alliance flipping the coordinates (im worried about this a little to be honest regarding what happens on the real robot)
    public static final ChoreoAllianceFlipUtil.Flipper flipper = ChoreoAllianceFlipUtil.getFlipper();

    //given a pose on blue/red, switch it to red/blue
    public static Pose2d flipPose(Pose2d pose) {
        Translation2d translation = new Translation2d(flipper.flipX(pose.getX()), flipper.flipY(pose.getY()));
        Rotation2d rotation = new Rotation2d(flipper.flipHeading(pose.getRotation().getRadians()));
        return new Pose2d(translation, rotation);
    }

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
        
        
        // Vertices of the hexagon, adjusted for safety margins.

        public static List<Translation2d> getHexagonVertices() {
         
            return List.of(
                flipIfRed(new Translation2d(3.668 - xComponent * SAFE_MARGIN, 3.520 - yComponent * SAFE_MARGIN)),
                flipIfRed(new Translation2d(4.5, 3.039 - SAFE_MARGIN)),
                flipIfRed(new Translation2d(5.332 + xComponent * SAFE_MARGIN, 3.520 - yComponent * SAFE_MARGIN)),
                flipIfRed(new Translation2d(5.332 + xComponent * SAFE_MARGIN, 4.480 + yComponent * SAFE_MARGIN)),
                flipIfRed(new Translation2d(4.5, 4.961 + SAFE_MARGIN)),
                flipIfRed(new Translation2d(3.668 - xComponent * SAFE_MARGIN, 4.480 + yComponent * SAFE_MARGIN)),
                flipIfRed(new Translation2d(3.668 - xComponent * SAFE_MARGIN, 3.520 - yComponent * SAFE_MARGIN))
            );
        }
        }
        
    public static final class PathPlannerConstants {
        public static final double maxVelocity = 0;
        public static final double maxAcceleration = 0;
        public static final double maxAngularVelocity = 0;
        public static final double maxAngularAcceleration = 0;
    }

    public static final class Setpoints {

        public static final double approachPointDistance = 0.6;

        //on the reef,
        //assume a vector that extends out of the front of the robot,
        //to calc where it'd be if it drove "left, right, or backward" relative to what that vector considers forward,
        //trig
        public static enum TrigDirection {
            LEFT,
            RIGHT,
            BACKWARD
        }

        /**
         * @param reefPose the center point of the side of the reef
         * @param direction translate the position left, right, or backward
         */
        public static Pose2d reefTrig(Pose2d reefPose, TrigDirection direction) {
            double offsetMultiplier = 1; // 90*this value equal 土90 relative to og rotation

            double isBackwardOffset = 1;// if we're going backward the dimensions of the right triangle are negative because it's backward

            double angleOffset = 90; // 90 for perpindicular
            double distance = 6.5; //distance between branches, this variable as a whole is essentially the "hypotenuse" of the total shift
            switch (direction) {
                case LEFT:
                    offsetMultiplier = 1; 
                    break;

                case RIGHT:
                    offsetMultiplier = -1; 
                    break;
                case BACKWARD:
                    angleOffset = 0; 
                    distance=3; // instead of moving 6.5 between pipes, move 3 inches away from the reef
                    isBackwardOffset=-1; 
                break;
            }

            //fundamental idea:
            //cos(x)hypotenuse = the amount of x to move / the x component of a right triangle
            //sin(x)hypotenuse = amount of y to move

            /*
             * therefore,
             * the change in x when moving left and right is 
             * x + cos(currentrotation 土 90 based on left or right)(hypotenuse==6.5inches)
             *  = how long x wise a 6.5inch line with respect totheta is
             */

            double xSetup = reefPose.getX() + Math
                    .cos(Math.toRadians(reefPose.getRotation().getDegrees() + (90))) * Units.inchesToMeters(6.25);
            double ySetup = reefPose.getY() + Math
                    .sin(Math.toRadians(reefPose.getRotation().getDegrees() + (90))) * Units.inchesToMeters(6.25);
            //6.25 is how far off our elevator thingymajig that drops coral off is, this offset happens first regardless
            

            double newX = xSetup + Math
                    .cos(Math.toRadians(reefPose.getRotation().getDegrees() + (angleOffset * offsetMultiplier)))
                    * Units.inchesToMeters(distance) * isBackwardOffset;
            //do whatever based on left right or backward using the same fundamental idea above
            

            double newY = ySetup + Math
                    .sin(Math.toRadians(reefPose.getRotation().getDegrees() + (angleOffset * offsetMultiplier)))
                    * Units.inchesToMeters(distance) * isBackwardOffset;

            return new Pose2d(newX, newY, reefPose.getRotation());
        };

        // Helper method to adjust Pose2d
        public static Pose2d adjustPose(double x, double y, double heading, boolean isCoralStation) {
            // Calculate offsets based only on robot length
            double offsetX = (ROBOT_LENGTH / 2) * Math.cos(heading);
            double offsetY = (ROBOT_WIDTH / 2) * Math.sin(heading);

            // Adjust coordinates to align the robot’s front edge with the target
            if (isCoralStation) {
                return new Pose2d(x + offsetX, y + offsetY, new Rotation2d(heading));

            }

            return new Pose2d(x - offsetX, y - offsetY, new Rotation2d(heading));
        };

        private static Pose2d createApproachPoint(Pose2d pose) {
            Translation2d position = pose.getTranslation();
            Rotation2d heading = pose.getRotation();
            Translation2d offset = new Translation2d(-approachPointDistance * Math.cos(heading.getRadians()),
                    -approachPointDistance * Math.sin(heading.getRadians()));
            return new Pose2d(position.plus(offset), heading);
        }

        private static Pose2d rotatePose(Pose2d pose, double degrees) {
            return new Pose2d(pose.getX(), pose.getY(),
                    new Rotation2d(Math.toRadians(pose.getRotation().getDegrees() + degrees)));
        }

        public static final double ROBOT_LENGTH = Units.inchesToMeters(37); // Length of the robot in meters
        public static final double ROBOT_WIDTH = Units.inchesToMeters(30);; // Width of the robot in meters

        // Adjusted setpoints
        public static Pose2d coralLeft = adjustPose(0.851154, 7.39648, Math.toRadians(-55), true);
        public static Pose2d coralRight = adjustPose(0.851154, 0.65532, Math.toRadians(55), true);
        public static Pose2d processor = adjustPose(5.987542, -0.00381, Math.toRadians(-90), false);

        public static Pose2d reefClose = adjustPose(3.65, 4, Math.toRadians(0), false);
        public static Pose2d reefCloseRight = adjustPose(4.07, 3.25, Math.toRadians(60), false);
        public static Pose2d reefFarRight = adjustPose(4.94, 3.25, Math.toRadians(120), false);
        public static Pose2d reefFar = adjustPose(5.35, 4, Math.toRadians(180), false);
        public static Pose2d reefFarLeft = adjustPose(4.94, 4.74, Math.toRadians(-120), false);
        public static Pose2d reefCloseLeft = adjustPose(4.07, 4.74, Math.toRadians(-60), false);

        // Please refer to:
        // https://firstfrc.blob.core.windows.net/frc2025/Manual/Sections/2025GameManual-05ARENA.pdf
        // to see what each letter setpoint refers to

        public static Pose2d aSetpoint = reefTrig(reefClose, TrigDirection.LEFT);
        public static Pose2d aL1 = reefTrig(rotatePose(aSetpoint, 90),TrigDirection.BACKWARD);

        public static Pose2d bSetpoint = reefTrig(reefClose, TrigDirection.RIGHT);
        public static Pose2d bL1 = reefTrig(rotatePose(bSetpoint, 90),TrigDirection.BACKWARD);

        public static Pose2d cSetpoint = reefTrig(reefCloseRight, TrigDirection.LEFT);
        public static Pose2d cL1 = reefTrig(rotatePose(cSetpoint, 90),TrigDirection.BACKWARD);

        public static Pose2d dSetpoint = reefTrig(reefCloseRight, TrigDirection.RIGHT);
        public static Pose2d dL1 = reefTrig(rotatePose(dSetpoint, 90),TrigDirection.BACKWARD);

        public static Pose2d eSetpoint = reefTrig(reefFarRight, TrigDirection.LEFT);
        public static Pose2d eL1 = reefTrig(rotatePose(eSetpoint, 90),TrigDirection.BACKWARD);

        public static Pose2d fSetpoint = reefTrig(reefFarRight, TrigDirection.RIGHT);
        public static Pose2d fL1 = reefTrig(rotatePose(fSetpoint, 90),TrigDirection.BACKWARD);

        public static Pose2d gSetpoint = reefTrig(reefFar, TrigDirection.LEFT);
        public static Pose2d gL1 = reefTrig(rotatePose(gSetpoint, 90),TrigDirection.BACKWARD);

        public static Pose2d hSetpoint = reefTrig(reefFar, TrigDirection.RIGHT);
        public static Pose2d hL1 = reefTrig(rotatePose(hSetpoint, 90),TrigDirection.BACKWARD);

        public static Pose2d iSetpoint = reefTrig(reefFarLeft, TrigDirection.LEFT);
        public static Pose2d iL1 = reefTrig(rotatePose(iSetpoint, 90),TrigDirection.BACKWARD);

        public static Pose2d jSetpoint = reefTrig(reefFarLeft, TrigDirection.RIGHT);
        public static Pose2d jL1 = reefTrig(rotatePose(jSetpoint, 90),TrigDirection.BACKWARD);

        public static Pose2d kSetpoint = reefTrig(reefCloseLeft, TrigDirection.LEFT);
        public static Pose2d kL1 = reefTrig(rotatePose(kSetpoint, 90),TrigDirection.BACKWARD);

        public static Pose2d lSetpoint = reefTrig(reefCloseLeft, TrigDirection.RIGHT);
        public static Pose2d lL1 = reefTrig(rotatePose(lSetpoint, 90),TrigDirection.BACKWARD);

        public static List<Pose2d> reefSides = List.of( //a list of the center points of each side
                reefClose,
                reefCloseRight,
                reefCloseLeft,
                reefFar,
                reefFarLeft,
                reefFarRight);


        //drive relative branches == when i press left or right bumper go to the closest one based on what the driver considers left or right
        public static HashMap<Pose2d, int[]> driveRelativeBranches = new HashMap<Pose2d, int[]>() {
            {
                put(reefClose, new int[] { 2, 4 }); // L R
                put(reefCloseLeft, new int[] { 22, 24 });
                put(reefCloseRight, new int[] { 7, 8 });
                put(reefFarRight, new int[] { 12, 10 });
                put(reefFar, new int[] { 16, 14 });
                put(reefFarLeft, new int[] { 20, 18 });
            }
        };

        public static Pose2d aApproach = createApproachPoint(aSetpoint);
        public static Pose2d bApproach = createApproachPoint(bSetpoint);
        public static Pose2d cApproach = createApproachPoint(cSetpoint);
        public static Pose2d dApproach = createApproachPoint(dSetpoint);
        public static Pose2d eApproach = createApproachPoint(eSetpoint);
        public static Pose2d fApproach = createApproachPoint(fSetpoint);
        public static Pose2d gApproach = createApproachPoint(gSetpoint);
        public static Pose2d hApproach = createApproachPoint(hSetpoint);
        public static Pose2d iApproach = createApproachPoint(iSetpoint);
        public static Pose2d jApproach = createApproachPoint(jSetpoint);
        public static Pose2d kApproach = createApproachPoint(kSetpoint);
        public static Pose2d lApproach = createApproachPoint(lSetpoint);

        public enum PPSetpoints {
            CORALLEFT(coralLeft, coralLeft), //ppsetpoint index 0
            CORALRIGHT(coralRight, coralRight), //1

            A(aSetpoint, aApproach), //2
            AL1(aL1, aApproach), //3

            B(bSetpoint, bApproach), //4
            BL1(bL1, bApproach), //ok you can count but notice how the index of BL2 is always an odd number and is B's index+1

            C(cSetpoint, cApproach),
            CL1(cL1, cApproach),

            D(dSetpoint, dApproach),
            DL1(dL1, dApproach),

            E(eSetpoint, eApproach),
            EL1(eL1, eApproach),

            F(fSetpoint, fApproach),
            FL1(fL1, fApproach),

            G(gSetpoint, gApproach),
            GL1(gL1, gApproach),

            H(hSetpoint, hApproach),
            HL1(hL1, hApproach),

            I(iSetpoint, iApproach),
            IL1(iL1, iApproach),

            J(jSetpoint, jApproach),
            JL1(jL1, jApproach),

            K(kSetpoint, kApproach),
            KL1(kL1, kApproach),

            L(lSetpoint, lApproach),
            LL1(lL1, lApproach),

            REEFCLOSE(rotatePose(reefClose,90), createApproachPoint(reefClose)), //26
            REEFCLOSELEFT(rotatePose(reefCloseLeft,90), createApproachPoint(reefCloseLeft)),
            REEFCLOSERIGHT(rotatePose(reefCloseRight,90), createApproachPoint(reefCloseRight)),

            REEFFAR(rotatePose(reefFar,90), createApproachPoint(reefFar)),
            REEFFARLEFT(rotatePose(reefFarLeft,90), createApproachPoint(reefFarLeft)),
            REEFFARRIGHT(rotatePose(reefFarRight,90), createApproachPoint(reefFarRight));

            public Pose2d setpoint;
            public Pose2d approachPoint;

            private PPSetpoints(Pose2d setpoint, Pose2d approachPoint) {
                this.setpoint = setpoint;
                this.approachPoint = approachPoint;
                if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
                    this.setpoint = flipPose(setpoint);
                    this.approachPoint = flipPose(approachPoint);
                }

        }

    }

}
}