package frc.robot.subsystems.swerve;

import java.util.List;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class ToPosConstants {

    // private final double hexOffset = SwerveConstants.DriveConstants.trackWidth /
    // 2.0;
    // private final double hexOffset = SwerveConstants.DriveConstants.trackWidth /
    // 2.0;

    public static final ChoreoAllianceFlipUtil.Flipper flipper = ChoreoAllianceFlipUtil.getFlipper();

    public static Pose2d flipPose(Pose2d pose) {
        Translation2d translation = new Translation2d(flipper.flipX(pose.getX()), flipper.flipY(pose.getY()));
        Rotation2d rotation = new Rotation2d(flipper.flipHeading(pose.getRotation().getRadians()));
        return new Pose2d(translation, rotation);
    }

    public static final class ReefVerticies {

        // MAKE CONSTANTS
        private static final double SAFE_MARGIN = .95; // Safety margin around the robot.
        private static final double xComponenet = Math.cos(Math.toRadians(30));
        private static final double yComponenet = Math.sin(Math.toRadians(30));

        private static Translation2d flipAccount(Translation2d translation) {
            if (DriverStationSim.getAllianceStationId().equals(AllianceStationID.Red1)) {
                return new Translation2d(flipper.flipX(translation.getX()), flipper.flipY(translation.getY()));
            }
            return translation;
        }
        // Vertices of the hexagon, adjusted for safety margins.

        public static List<Translation2d> HEXAGON_VERTICES = List.of(
                flipAccount(new Translation2d(3.668 - xComponenet * SAFE_MARGIN, 3.520 - yComponenet * SAFE_MARGIN)), // close
                                                                                                                      // right
                flipAccount(new Translation2d(4.5, 3.039 - SAFE_MARGIN)), // middle right
                flipAccount(new Translation2d(5.332 + xComponenet * SAFE_MARGIN, 3.520 - yComponenet * SAFE_MARGIN)), // far
                                                                                                                      // right
                flipAccount(new Translation2d(5.332 + xComponenet * SAFE_MARGIN, 4.480 + yComponenet * SAFE_MARGIN)), // far
                                                                                                                      // left
                flipAccount(new Translation2d(4.5, 4.961 + SAFE_MARGIN)), // middle left
                flipAccount(new Translation2d(3.668 - xComponenet * SAFE_MARGIN, 4.480 + yComponenet * SAFE_MARGIN)), // close
                                                                                                                      // left
                flipAccount(new Translation2d(3.668 - xComponenet * SAFE_MARGIN, 3.520 - yComponenet * SAFE_MARGIN))); // close
                                                                                                                       // right
    }

    public static final class PathPlannerConstants {
        public static final double maxVelocity = 0;
        public static final double maxAcceleration = 0;
        public static final double maxAngularVelocity = 0;
        public static final double maxAngularAcceleration = 0;
    }

    public static final class Setpoints {

        public static final double approachPointDistance = 0.6;

        public static enum TrigDirection {
            LEFT,
            RIGHT,
            FORWARD,

        }

        // public static Pose2d buttonBoardSetpointMap(int index) {
        // return ppSetpoints[index];
        // }

        // public static Pose2d buttonBoardApproachSetpointMap(int index) {
        // return ppApproachSetpoints[index];
        // }

        public static Pose2d reefTrig(Pose2d reefPose, TrigDirection direction) {
            double offsetMultiplier = 1;
            double forwardMagnitudeMultiplier = 1;
            double angleOffset = 90; // 90 for perpindicular
            switch (direction) {
                case LEFT:
                    offsetMultiplier = 1;
                    break;

                case RIGHT:
                    offsetMultiplier = -1;
                    break;

                case FORWARD:
                    angleOffset = 0;
                    forwardMagnitudeMultiplier = 1.5;
                    break;

                default:
                    System.out.println("did not specify trig direction");
                    break;
            }

            double xOffset = Math
                    .cos(Math.toRadians(reefPose.getRotation().getDegrees() + (angleOffset * offsetMultiplier)))
                    / (4 * forwardMagnitudeMultiplier);
            double yOffset = Math
                    .sin(Math.toRadians(reefPose.getRotation().getDegrees() + (angleOffset * offsetMultiplier)))
                    / (4 * forwardMagnitudeMultiplier);
            return new Pose2d(reefPose.getX() + xOffset, reefPose.getY() + yOffset, reefPose.getRotation());
        };

        // Helper method to adjust Pose2d
        public static Pose2d adjustPose(double x, double y, double heading, boolean isCoralStation) {
            // Calculate offsets based only on robot length
            double offsetX = (ROBOT_LENGTH / 2) * Math.cos(heading);
            double offsetY = (ROBOT_LENGTH / 2) * Math.sin(heading);

            // Adjust coordinates to align the robot’s front edge with the target
            if (isCoralStation) {
                // if(DriverStationSim.getAllianceStationId().equals(AllianceStationID.Red1))
                // {
                // return flipPose(new Pose2d(x + offsetX, y + offsetY, new
                // Rotation2d(heading)));
                // }
                return new Pose2d(x + offsetX, y + offsetY, new Rotation2d(heading));

            }
            // if(DriverStationSim.getAllianceStationId().equals(AllianceStationID.Red1))
            // {
            // return flipPose(new Pose2d(x - offsetX, y - offsetY, new
            // Rotation2d(heading)));
            // }
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
        public static final double ROBOT_WIDTH = Units.inchesToMeters(31); // Width of the robot in meters

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
        public static Pose2d aRotatedSetpoint = rotatePose(aSetpoint, 90);

        public static Pose2d bSetpoint = reefTrig(reefClose, TrigDirection.RIGHT);
        public static Pose2d bRotatedSetpoint = rotatePose(bSetpoint, 90);

        public static Pose2d cSetpoint = reefTrig(reefCloseRight, TrigDirection.LEFT);
        public static Pose2d cRotatedSetpoint = rotatePose(cSetpoint, 90);

        public static Pose2d dSetpoint = reefTrig(reefCloseRight, TrigDirection.RIGHT);
        public static Pose2d dRotatedSetpoint = rotatePose(dSetpoint, 90);

        public static Pose2d eSetpoint = reefTrig(reefFarRight, TrigDirection.LEFT);
        public static Pose2d eRotatedSetpoint = rotatePose(eSetpoint, 90);

        public static Pose2d fSetpoint = reefTrig(reefFarRight, TrigDirection.RIGHT);
        public static Pose2d fRotatedSetpoint = rotatePose(fSetpoint, 90);

        public static Pose2d gSetpoint = reefTrig(reefFar, TrigDirection.LEFT);
        public static Pose2d gRotatedSetpoint = rotatePose(gSetpoint, 90);

        public static Pose2d hSetpoint = reefTrig(reefFar, TrigDirection.RIGHT);
        public static Pose2d hRotatedSetpoint = rotatePose(hSetpoint, 90);

        public static Pose2d iSetpoint = reefTrig(reefFarLeft, TrigDirection.LEFT);
        public static Pose2d iRotatedSetpoint = rotatePose(iSetpoint, 90);

        public static Pose2d jSetpoint = reefTrig(reefFarLeft, TrigDirection.RIGHT);
        public static Pose2d jRotatedSetpoint = rotatePose(jSetpoint, 90);

        public static Pose2d kSetpoint = reefTrig(reefCloseLeft, TrigDirection.LEFT);
        public static Pose2d kRotatedSetpoint = rotatePose(kSetpoint, 90);

        public static Pose2d lSetpoint = reefTrig(reefCloseLeft, TrigDirection.RIGHT);
        public static Pose2d lRotatedSetpoint = rotatePose(lSetpoint, 90);

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

        // for(int i=0;i<PPSetpoints.values().length;i++)
        // {
        // Pose2d approachPoint = PPSetpoints.values()[i].approachPoint;
        // Pose2d setpoint = PPSetpoints.values()[i].setpoint;
        // PPSetpoints.values()[i].approachPoint = flipPose(approachPoint);
        // PPSetpoints.values()[i].setpoint = flipPose(setpoint);
        // }

        public enum PPSetpoints {

            CORALLEFT(coralLeft, coralLeft, new PrintCommand("coral left")),
            CORALRIGHT(coralRight, coralRight, new PrintCommand("coral right")),

            A(aSetpoint, aApproach, new PrintCommand("a setpoint")),
            AROTATE(aRotatedSetpoint, aApproach, new PrintCommand("a rotated setpoint")),

            B(bSetpoint, bApproach, new PrintCommand("b setpoint")),
            BROTATE(bRotatedSetpoint, bApproach, new PrintCommand("b rotated setpoint")),

            C(cSetpoint, cApproach, new PrintCommand("c setpoint")),
            CROTATE(cRotatedSetpoint, cApproach, new PrintCommand("c rotated setpoint")),

            D(dSetpoint, dApproach, new PrintCommand("d setpoint")),
            DROTATE(dRotatedSetpoint, dApproach, new PrintCommand("d rotated setpoint")),

            E(eSetpoint, eApproach, new PrintCommand("e setpoint")),
            EROTATE(eRotatedSetpoint, eApproach, new PrintCommand("e rotated setpoint")),

            F(fSetpoint, fApproach, new PrintCommand("f setpoint")),
            FROTATE(fRotatedSetpoint, fApproach, new PrintCommand("f rotated setpoint")),

            G(gSetpoint, gApproach, new PrintCommand("g setpoint")),
            GROTATE(gRotatedSetpoint, gApproach, new PrintCommand("g rotated setpoint")),

            H(hSetpoint, hApproach, new PrintCommand("h setpoint")),
            HROTATE(hRotatedSetpoint, hApproach, new PrintCommand("h rotated setpoint")),

            I(iSetpoint, iApproach, new PrintCommand("i setpoint")),
            IROTATE(iRotatedSetpoint, iApproach, new PrintCommand("i rotated setpoint")),

            J(jSetpoint, jApproach, new PrintCommand("j setpoint")),
            JROTATE(jRotatedSetpoint, jApproach, new PrintCommand("j rotated setpoint")),

            K(kSetpoint, kApproach, new PrintCommand("k setpoint")),
            KROTATE(kRotatedSetpoint, kApproach, new PrintCommand("k rotated setpoint")),

            L(lSetpoint, lApproach, new PrintCommand("l setpoint")),
            LROTATE(lRotatedSetpoint, lApproach, new PrintCommand("l rotated setpoint")),

            REEFCLOSE(reefClose, createApproachPoint(reefClose), new PrintCommand("reef close")),
            REEFCLOSELEFT(reefCloseLeft, createApproachPoint(reefCloseLeft), new PrintCommand("reef close left")),
            REEFCLOSERIGHT(reefCloseRight, createApproachPoint(reefCloseRight), new PrintCommand("reef close right")),

            REEFFAR(reefFar, createApproachPoint(reefFar), new PrintCommand("reef far")),
            REEFFARLEFT(reefFarLeft, createApproachPoint(reefFarLeft), new PrintCommand("reef far left")),
            REEFFARRIGHT(reefFarRight, createApproachPoint(reefFarRight), new PrintCommand("reef far right"));

            public Pose2d setpoint;
            public Pose2d approachPoint;
            public Command onReachCommand;

            private PPSetpoints(Pose2d setpoint, Pose2d approachPoint, Command onReachCommand) {
                this.setpoint = setpoint;
                this.approachPoint = approachPoint;
                this.onReachCommand = onReachCommand;
                if (DriverStationSim.getAllianceStationId().equals(AllianceStationID.Red1)) {
                    this.setpoint = flipPose(setpoint);
                    this.approachPoint = flipPose(approachPoint);
                    this.onReachCommand = onReachCommand;
                }
            }

        }

        // public static final Pose2d a = new Pose2d(3.76, 3.49, new Rotation2d(0));
        // public static final Pose2d b = new Pose2d(4.39, 4.93, new Rotation2d(0));
        // public static final Pose2d c = new Pose2d( 5.22, 4.57, new Rotation2d(0));
        // public static final Pose2d c = new Pose2d( 5.22, 4.57, new Rotation2d(0));
        // public static final Pose2d d = new Pose2d(3.76, 3.49, new Rotation2d(0));
        // public static final Pose2d e = new Pose2d(5.32, 3.67, new Rotation2d(0));
        // public static final Pose2d f = new Pose2d(4.59, 3.13, new Rotation2d(0));
        // these aren't reef letter id's just verticies of the hexagon

    }

    public static final class Obstacles {

    }

}