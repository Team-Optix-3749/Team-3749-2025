package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.LoggedTunableNumber;

/**
 * All constants for the swerve subsystem and swerve modules
 * 
 * @author Noah Simon
 * @author Neel Adem
 * @author Rohin Sood
 * @author Raadwan Masum
 * 
 */
public class SwerveConstants {

        public static final class ControlConstants {
                public static final double[] turnPID = new double[] {
                                3.75, 0, 0 };
                public static final double[] drivePID = new double[] {
                                0.27, 0, 0 };
                // our FF values
                public static LoggedTunableNumber kSDriving = new LoggedTunableNumber("subsystems/swerve/kSDriving",
                                0.26);
                public static LoggedTunableNumber kVDriving = new LoggedTunableNumber("subsystems/swerve/kVDriving",
                                2.765);
                public static LoggedTunableNumber kADriving = new LoggedTunableNumber("subsystems/swerve/kADriving",
                                0.0);

                public static LoggedTunableNumber maxVelocity = new LoggedTunableNumber("subsystems/swerve/maxVelocity",
                                4.3);
                public static LoggedTunableNumber maxAcceleration = new LoggedTunableNumber(
                                "subsystems/swerve/maxAcceleration", 3.3);

                // teleop speed
                public static final double teleopMaxSpeedReduction = 0; // If we can drive a little faster in telop
                // we may as well
                public static final double teleopMaxSpeedMetersPerSecond = maxVelocity.get()
                                * (1 - teleopMaxSpeedReduction);

                // auto speed
                public static final double autoMaxSpeedReduction = 0;
                public static final double autoMaxSpeedMetersPerSecond = maxVelocity.get()
                                * (1 - autoMaxSpeedReduction);

                public static final double maxAngularSpeedRadiansPerSecond = 12.162;
                public static final double maxAngularAccelerationRadiansPerSecondSquared = 15.543;
                // teleop angluar speed
                public static final double teleopMaxAngularSpeedReduction = 0.4;
                public static final double teleopMaxAngularSpeedRadPerSecond = maxAngularSpeedRadiansPerSecond
                                * (1 - teleopMaxAngularSpeedReduction);

                // auto angular speed
                public static final double autoMaxAngularSpeedReduction = 0;
                public static final double autoMaxAngularSpeedRadPerSecond = maxAngularSpeedRadiansPerSecond
                                * (1 - autoMaxAngularSpeedReduction);

        }

        public static final class MotorConstants {
                public static final double driveMotorGearRatio = 6.75;
                public static final double turnMotorGearRatio = 12.8;
                // Module Settings: order is FL, FR, BL, BR
                public static final int[] driveMotorIds = { 3, 5, 7, 9 };
                public static final int[] turnMotorIds = { 4, 6, 8, 10 };
                public static final int[] absoluteEncoderIds = { 11, 12, 13, 14 };

                public static final boolean[] driveMotorInverted = {
                                true,
                                false,
                                true,
                                true
                };
                public static final boolean[] turningMotorInverted = {
                                false,
                                false,
                                false,
                                false
                };
                public static final boolean[] driveAbsoluteEncoderInverted = {
                                false,
                                false,
                                false,
                                false
                };
                public static final double[] absoluteEncoderOffsetDeg = {
                                49.733, 67.208, 276.509, 21.715
                };

        }

        public static final class DrivetrainConstants {
                public static final double wheelDiameterMeters = Units.inchesToMeters(4);
                // Distance between right and left wheels
                public static final double trackWidth = Units.inchesToMeters(19.5);
                // Distance between front and back wheels
                public static final double wheelBase = Units.inchesToMeters(19.5);
                public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
                                new Translation2d(wheelBase / 2, trackWidth / 2), // front left
                                new Translation2d(wheelBase / 2, -trackWidth / 2), // front right
                                new Translation2d(-wheelBase / 2, trackWidth / 2), // back left
                                new Translation2d(-wheelBase / 2, -trackWidth / 2)); // back right

        }

}
