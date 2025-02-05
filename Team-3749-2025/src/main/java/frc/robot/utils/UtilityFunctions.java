package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

/**
 * Methods that are helpful throughout the code base
 * 
 * @author Noah Simon
 */
public class UtilityFunctions {

    public static boolean isRedAlliance() {
        boolean isRed = false;

        if (DriverStation.getAlliance().isEmpty()) {
            return isRed;
        } else {
            isRed = DriverStation.getAlliance().get() == Alliance.Red;
        }
        return isRed;
    }

    /***
     * 
     * @param margin how close the values need to be to return true. Use a positive
     *               number
     * @param a      the first number
     * @param b      the second number
     * @return true if it is within the margin, false if not
     */
    public static boolean withinMargin(double margin, double a, double b) {
        if (a + margin >= b && a - margin <= b) {
            return true;
        }
        return false;
    }

    /**
     * @param velocity
     * @return whether or not the velocity is below 0.01, which we consider to be
     *         stopped
     */
    public static boolean isStopped(double velocity) {
        return withinMargin(0.01, velocity, 0);
    }

    /**
     * @param velocity
     * @param minSpeed
     * @return whether or not the velocity is below the minimum speed to be
     *         considered stopped
     */
    public static boolean isStopped(double velocity, double minSpeed) {
        return withinMargin(minSpeed, velocity, 0);
    }

    // This is a custom trigger for AutoUtils
    public static Trigger CheckPose(Pose2d targetPose, double toleranceMeters, double toleranceRadians) {
        return new Trigger(() -> {
            Pose2d currentPose = Robot.swerve.getPose();
            double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
            double rotationDifference = Math.abs(currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians());
            return distance < toleranceMeters && rotationDifference < toleranceRadians;
        });
    }

}
