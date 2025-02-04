package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.utils.UtilityFunctions;
import frc.robot.utils.MiscConstants.*;
import java.util.function.Supplier;

/***
 * Default command to control the swerve subsystem with joysticks
 * 
 * @author Noah Simon
 * @author Raadwan Masum
 * @author Rohin Sood
 */

public class SwerveDefaultCommand extends Command {

  private final Supplier<Double> xSpdFunction, ySpdFunction, xTurningSpdFunction;

  public SwerveDefaultCommand(
      Supplier<Double> xSpdFunction,
      Supplier<Double> ySpdFunction,
      Supplier<Double> xTurningSpdFunction) {
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.xTurningSpdFunction = xTurningSpdFunction;

    super.addRequirements(Robot.swerve);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // controllers are weird in what's positive, so we flip these
    double xMagnitude = xSpdFunction.get();
    double yMagnitude = ySpdFunction.get();
    double turningMagnitude = -xTurningSpdFunction.get();

    // one combined magnitutde
    double linearMagnitude = Math.hypot(xMagnitude, yMagnitude);
    // to make a 0,0 rotation 2d not throw an error

    // one combined direction
    Rotation2d linearDirection = UtilityFunctions.safeRotation(xMagnitude, yMagnitude);

    // deadbands
    // is always postive
    linearMagnitude = MathUtil.applyDeadband(linearMagnitude, ControllerConstants.deadband);
    // can be negative
    turningMagnitude = Math.abs(turningMagnitude) > ControllerConstants.deadband
        ? turningMagnitude
        : 0.0;

    // squaring the inputs for smoother driving at low speeds
    linearMagnitude = Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);
    turningMagnitude = Math.copySign(turningMagnitude * turningMagnitude, turningMagnitude);

    if (Robot.swerve.isOTF) {
      if ((Math.abs(linearMagnitude) > 0.2 || Math.abs(turningMagnitude) > 0.2)) {
        Robot.swerve.isOTF = false;
      } else {
        return;
      }
    }

    double driveSpeedMPS = linearMagnitude * Robot.swerve.getMaxDriveSpeed();
    double turningSpeedRadPerSecond = turningMagnitude * Robot.swerve.getMaxAngularSpeed();

    // Calcaulate new linear components
    double xSpeed = driveSpeedMPS * Math.cos(linearDirection.getRadians());
    double ySpeed = driveSpeedMPS * Math.sin(linearDirection.getRadians());
    ChassisSpeeds chassisSpeeds;

    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        UtilityFunctions.isRedAlliance() ? ySpeed : -ySpeed,
        UtilityFunctions.isRedAlliance() ? xSpeed : -xSpeed,
        turningSpeedRadPerSecond,
        Robot.swerve.getRotation2d());

    // set chassis speeds

    Robot.swerve.setChassisSpeeds(chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
