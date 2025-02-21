package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
        }
        return DriverStation.getAlliance().get() == Alliance.Red;
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
        // if (a + margin >= b && a - margin <= b) {
        // return true;
        // }
        // return false;

        return (a + margin >= b && a - margin <= b);
    }

    public static boolean withinMargin(double translationMargin, Translation2d translationA,
            Translation2d translationB) {

        double Dist = translationA.getDistance(translationB);

        return Dist < translationMargin;
    }

    /**
     * 
     * @param poseMargin
     * @param poseA
     * @param poseB
     * @return
     */
    public static boolean withinMargin(Pose2d poseMargin, Pose2d poseA, Pose2d poseB) {

        Pose2d relativeMargin = poseA.relativeTo(poseB);

        return Math.abs(relativeMargin.getX()) < poseMargin.getX() &&
                Math.abs(relativeMargin.getY()) < poseMargin.getY() &&
                Math.abs(relativeMargin.getRotation().getDegrees()) < poseMargin.getRotation().getDegrees();
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

    /**
     * Ensures safe initialization of Rotation2d. Falls back to a default rotation
     * if invalid.
     *
     * @param rotation The rotation to validate.
     * @return A valid Rotation2d object.
     */
    public static Rotation2d safeRotation(double rads) {
        if (Math.abs(Math.cos(rads)) < 1e-6 && Math.abs(Math.sin(rads)) < 1e-6) {
            // System.out.println("Warning: Invalid Rotation2d detected. Falling back to
            // neutral rotation.");
            return new Rotation2d(0); // Neutral rotation
        }
        return new Rotation2d(rads);
    }

    public static Rotation2d safeRotation(double x, double y) {
        double rads = Math.atan(y / x);
        if (x == 0) {
            rads = Math.copySign(Math.PI / 2, y);
        }

        if (Math.abs(Math.cos(rads)) < 1e-6 && Math.abs(Math.sin(rads)) < 1e-6) {
            // System.out.println("Warning: Invalid Rotation2d detected. Falling back to
            // neutral rotation.");
            return new Rotation2d(0); // Neutral rotation
        }
        return new Rotation2d(rads);
    }

    // This is a custom trigger for AutoUtils
    public static Trigger CheckPose(Pose2d targetPose, double toleranceMeters, double toleranceRadians) {
        return new Trigger(() -> {
            Pose2d currentPose = Robot.swerve.getPose();
            double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
            double rotationDifference = Math
                    .abs(currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians());
            return distance < toleranceMeters && rotationDifference < toleranceRadians;
        });
    }

    public static double applyDeadband(double num, double deadband) {
        if (deadband > Math.abs(num)) {
            return 0;
        }
        return num;

    }

}
