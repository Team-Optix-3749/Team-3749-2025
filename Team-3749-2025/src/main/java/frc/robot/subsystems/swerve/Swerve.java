// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;

import choreo.util.ChoreoAllianceFlipUtil.Flipper;

import org.littletonrobotics.junction.Logger;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.commands.auto.AutoConstants;
import frc.robot.commands.auto.AutoUtils;
import frc.robot.subsystems.swerve.GyroIO.GyroData;
import frc.robot.subsystems.swerve.real.PigeonGyro;
import frc.robot.subsystems.swerve.sim.GyroSim;
import frc.robot.subsystems.swerve.sim.SwerveModuleSim;
import frc.robot.utils.MotorData;
import frc.robot.utils.SysIdTuner;
import frc.robot.utils.UtilityFunctions;
import frc.robot.subsystems.swerve.SwerveConstants.ControlConstants;
import frc.robot.subsystems.swerve.SwerveConstants.DrivetrainConstants;
import frc.robot.subsystems.swerve.real.*;
import frc.robot.subsystems.vision.VisionConstants;

/***
 * Subsystem class for swerve drive, used to manage four swerve
 * modules and set their states. Also includes a pose estimator,
 * gyro, and logging information
 * 
 * Rotation standard: everything is relative to blue alliance. 0 is
 * from blue alliance wall, CCP
 * 
 * @author Noah Simon
 * @author Neel Adem
 * @author Rohin Sood
 * @author Raadwan Masum
 * 
 * 
 */
public class Swerve extends SubsystemBase {

  private SwerveModule[] modules = new SwerveModule[4];

  private GyroIO gyro;
  private GyroData gyroData = new GyroData();

  // equivilant to a odometer, but also intakes vision
  private SwerveDrivePoseEstimator swerveDrivePoseEstimator;

  private PIDController xController = new PIDController(AutoConstants.kPDrive, 0, AutoConstants.kDDrive);
  private PIDController yController = new PIDController(AutoConstants.kPDrive, 0, AutoConstants.kDDrive);
  private PIDController turnController = new PIDController(AutoConstants.kPTurn, 0, AutoConstants.kDTurn);

  private boolean utilizeVision = true;
  private double velocity = 0;
  private double yaw;


  private SysIdTuner driveSysIdTuner;
  private SysIdTuner turningSysIdTuner;
  private SysIdTuner rotationalSysIdTuner;

  private Map<String, MotorData> driveMotorData;
  private Map<String, MotorData> turningMotorData;
  private Map<String, MotorData> rotationalMotorData;


  SysIdRoutine.Config config = new SysIdRoutine.Config(
      Volts.per(Seconds).of(1.2), // Voltage ramp rate
      Volts.of(12), // Max voltage
      Seconds.of(10) // Test duration
  );

  public Swerve() {

    // if simulation
    if (Robot.isSimulation()) {
      gyro = new GyroSim();
      for (int i = 0; i < 4; i++) {
        modules[i] = new SwerveModule(i, new SwerveModuleSim());
      }
    }
    // if real
    else {
      // gyro = new NavX2Gyro();
      gyro = new PigeonGyro();
      for (int i = 0; i < 4; i++) {
        
        modules[i] = new SwerveModule(i, new SwerveModuleSpark(i));
      }
    }
    // pose estimator
    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
        DrivetrainConstants.driveKinematics,
        new Rotation2d(0),
        new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        },
        new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
        VecBuilder.fill(0.045, 0.045, 0.0004), // 6328's 2024 numbers with factors of 1.5x, 1.5x, 2x
        VecBuilder.fill(VisionConstants.StandardDeviations.PreMatch.xy,
            VisionConstants.StandardDeviations.PreMatch.xy,
            VisionConstants.StandardDeviations.PreMatch.thetaRads));

    driveMotorData = Map.of("drive_left",
        new MotorData(
            modules[0].getModuleData().driveAppliedVolts,
            modules[0].getModuleData().drivePositionM,
            modules[0].getModuleData().driveVelocityMPerSec,
            modules[0].getModuleData().driveAccelerationMPerSecSquared));
    
    turningMotorData = Map.of("turning_left",
    new MotorData(
        modules[0].getModuleData().turnAppliedVolts,
        modules[0].getModuleData().turnPositionRad,
        modules[0].getModuleData().turnVelocityRadPerSec,
        0));
    
    rotationalMotorData = Map.of("drive_left",
    new MotorData(
        modules[0].getModuleData().driveAppliedVolts,
        modules[0].getModuleData().drivePositionM,
        modules[0].getModuleData().driveVelocityMPerSec,
        modules[0].getModuleData().driveAccelerationMPerSecSquared));

    driveSysIdTuner = new SysIdTuner("drive", config, this, (volts) -> {
      for (int i = 0; i < 4; i++) {
        modules[i].setDriveVoltage(volts);
      }}, driveMotorData);
    
    turningSysIdTuner = new SysIdTuner("turn", config, this, (volts) -> {
      for (int i = 0; i < 4; i++) {
        modules[i].setTurnVoltage(volts);
      }}, turningMotorData);

    rotationalSysIdTuner = new SysIdTuner("rotate", config, this, (volts) -> {
      for (int i = 0; i < 4; i++) {
        modules[i].setDriveVoltage(volts);
      }}, rotationalMotorData);
    
    turnController.enableContinuousInput(-Math.PI, Math.PI);

    // put us on the field with a default orientation
    resetGyro();
    setOdometry(new Pose2d(1.33,5.53, new Rotation2d(0)));
    logSetpoints(1.33, 0, 0, 5.53, 0, 0, 0, 0, 0);

  }

  public SysIdTuner getDriveSysIdTuner() {
    return driveSysIdTuner;
  }

  public SysIdTuner getTurningSysIdTuner() {
    return turningSysIdTuner;
  }

  public SysIdTuner getRotationalSysIdTuner(){
    return rotationalSysIdTuner;
  }

  public boolean getRotated(){
    return UtilityFunctions.withinMargin(0.01, modules[0].getModuleData().turnPositionRad, Rotation2d.fromDegrees(45).getRadians())
    &&  UtilityFunctions.withinMargin(0.01, modules[1].getModuleData().turnPositionRad, Rotation2d.fromDegrees(135).getRadians())
    &&  UtilityFunctions.withinMargin(0.01, modules[2].getModuleData().turnPositionRad, Rotation2d.fromDegrees(225).getRadians())
    &&  UtilityFunctions.withinMargin(0.01, modules[3].getModuleData().turnPositionRad, Rotation2d.fromDegrees(315).getRadians());

  }

  /**
   * Returns the coordinate and angular velocity of the robot
   * 
   * @return chassisSpeeds - ChasssisSpeeds object with a x, y, and rotaitonal
   *         velocity
   */
  public ChassisSpeeds getChassisSpeeds() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        DrivetrainConstants.driveKinematics.toChassisSpeeds(states),
        getRotation2d());
    return speeds;
  }

  /**
   * @return Returns direction the robot is facing as a Rotation2d object
   */
  public Rotation2d getRotation2d() {
    Rotation2d rotation = swerveDrivePoseEstimator
        .getEstimatedPosition()
        .getRotation();
    // return rotation;
    double heading = rotation.getDegrees();

    while (heading < 0) {
      heading += 360;
    }
    return new Rotation2d(heading / 180 * Math.PI);
  }

  /**
   * @return Returns coordinates and head the robot as a Pose2d object
   */
  public Pose2d getPose() {
    Pose2d estimatedPose = swerveDrivePoseEstimator.getEstimatedPosition();
    return new Pose2d(estimatedPose.getTranslation(), getRotation2d());
    // return new Pose2d(new Translation2d(2, 4.9), new Rotation2d(Math.PI/2));
  }

  /**
   * @return Returns max speed to be achieved by the robot based on telop or auto
   */
  public double getMaxDriveSpeed() {
    return DriverStation.isTeleopEnabled() ? ControlConstants.teleopMaxSpeedMetersPerSecond
        : ControlConstants.autoMaxSpeedMetersPerSecond;
  }

  /**
   * @return Returns max angular speed to be achieved by the robot based on telop
   *         or auto
   */
  public double getMaxAngularSpeed() {
    return DriverStation.isTeleopEnabled() ? ControlConstants.teleopMaxAngularSpeedRadPerSecond
        : ControlConstants.autoMaxAngularSpeedRadPerSecond;
  }

  public SwerveDrivePoseEstimator getPoseEstimator() {
    return swerveDrivePoseEstimator;
  }

  /**
   * Turns a set of coordinate and angular velocities into module states, then
   * sets all modules to those states
   * 
   * @param chassisSpeeds - ChasssisSpeeds object with a x, y, and rotaitonal
   *                      velocity
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    // Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DrivetrainConstants.driveKinematics.toSwerveModuleStates(
        chassisSpeeds);
    setModuleStates(moduleStates);

  }

  /**
   * takes a set of module states and sets individual modules to those states,
   * capping speeds to the maxmimum of the wheels
   * 
   * @param desiredStates - the individual module states coupled. FLD, FLT, FRD,
   *                      FRT, BLD, BLT, BRD, BRT
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates,
        getMaxDriveSpeed());

    modules[0].setDesiredState(desiredStates[0]);
    modules[1].setDesiredState(desiredStates[1]);
    modules[2].setDesiredState(desiredStates[2]);
    modules[3].setDesiredState(desiredStates[3]);

  }

  /**
   * 
   * @param curPose the current pose of the robot in meters, used for measuring
   *                error
   * @param sample  the setpoint sample with position, velocity, acceleration, and
   *                forces
   * 
   * @see https://choreo.autos/choreolib/getting-started/#setting-up-the-drive-subsystem
   * 
   * @note verticle flipping relies on choreo detecting rotational symetry on the
   *       field
   */

  public void followSample(SwerveSample sample, boolean isFlipped) {

    Flipper flipper = AutoUtils.flipper;

    // ternaries are for x-axis flipping

    double xPos = sample.x;

    double xVel = sample.vx;
    double xAcc = sample.ax;
    double yPos = isFlipped ? flipper.flipY(sample.y) : sample.y;

    double yVel = isFlipped ? -sample.vy : sample.vy;
    double yAcc = isFlipped ? -sample.ay : sample.ay;

    double heading = isFlipped ? new Rotation2d(Math.PI - sample.heading).rotateBy(new Rotation2d(Math.PI)).getRadians()
        : sample.heading;

    double omega = isFlipped ? -sample.omega : sample.omega;
    double alpha = isFlipped ? -sample.alpha : sample.alpha;

    Robot.swerve.logSetpoints(xPos, xVel, xAcc, yPos, yVel, yAcc, heading, omega, alpha);

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        new ChassisSpeeds(
            xController.calculate(getPose().getX(), xPos) + xVel,
            yController.calculate(getPose().getY(), yPos) + yVel,
            turnController.calculate(getPose().getRotation().getRadians(), heading) + omega),
        getPose().getRotation());

    Robot.swerve.setChassisSpeeds(speeds);
  }

  public void setBreakMode(boolean enable) {
    for (int i = 0; i < 4; i++) {
      modules[i].setBreakMode(enable);
    }
  }

  /**
   * Toggles whether or not vision updates odometry
   * 
   * @param utilize - whether or not to use vision updates on odometery, true is
   *                yes
   */
  public void setUtilizeVision(boolean utilize) {
    utilizeVision = utilize;
  }

  /**
   * Manually sets our odometry position
   * 
   * @param pose - Pose2d object of what to set our position to
   */
  public void setOdometry(Pose2d pose) {
    System.out.println("Set Odometry: " + pose.getX() + ", " + pose.getY() + ", " + pose.getRotation().getDegrees());
    Rotation2d gyroHeading = Rotation2d.fromDegrees(gyroData.yawDeg);

    swerveDrivePoseEstimator.resetPosition(
        gyroHeading,
        new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        },
        pose);
  }

  public void setRotation(){
    modules[0].setTurnPosition(45*(Math.PI/180));
    modules[1].setTurnPosition(135*Math.PI/180);

    modules[2].setTurnPosition(225*Math.PI/180);
    modules[3].setTurnPosition(315*Math.PI/180);

  }

  /**
   * Updates our odometry position based on module encoders. Ran in Periodic
   */
  public void updateOdometry() {
    // convert to -pi to pi

    swerveDrivePoseEstimator.update(
        Rotation2d.fromDegrees(gyroData.yawDeg),
        new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        });
  }

  /**
   * Updates our odometry position based on vision. Called by vision subsystem
   */
  public void visionUpdateOdometry(Pose2d pose, double timestamp) {
    if (utilizeVision) {
      swerveDrivePoseEstimator.addVisionMeasurement(pose,
          timestamp);
    }
  }

  /**
   * Sets voltage to all swerve motors to 0
   */
  public void stopModules() {
    for (SwerveModule module : modules) {
      module.stop();
    }
  }

  /**
   * When called, makes the robot's current direction "forward"
   */
  public void resetGyro() {
    gyro.resetGyro();
    if (UtilityFunctions.isRedAlliance()) {
      swerveDrivePoseEstimator.resetPosition(new Rotation2d(), new SwerveModulePosition[] {
          modules[0].getPosition(),
          modules[1].getPosition(),
          modules[2].getPosition(),
          modules[3].getPosition()
      }, new Pose2d(swerveDrivePoseEstimator.getEstimatedPosition().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      swerveDrivePoseEstimator.resetPosition(new Rotation2d(), new SwerveModulePosition[] {
          modules[0].getPosition(),
          modules[1].getPosition(),
          modules[2].getPosition(),
          modules[3].getPosition()
      }, new Pose2d(swerveDrivePoseEstimator.getEstimatedPosition().getTranslation(), new Rotation2d()));
    }
  }

  /**
   * logs all setpoints for the swerve subsystem in autonomous functions
   * 
   * @param the swerve sample of setpoints
   */
  public void logSetpoints(double posX, double velX, double accX, double posY, double velY, double accY, double heading,
      double omega, double alpha) {
    // setpoint logging for automated driving
    Double[] positions = new Double[] { posX, posY, heading };
    Logger.recordOutput("/subsystems/swerve/swerve position x", positions[0]);
    Logger.recordOutput("/subsystems/swerve/swerve position y", positions[1]);
    Logger.recordOutput("/subsystems/swerve/swerve position heading", positions[2]);

    Double[] velocities = new Double[] { velX, velY, omega };
    double velocity = 0;
    for (int i = 0; i < 2; i++) {
      velocity += Math.pow(velocities[i], 2);
    }
    velocity = Math.sqrt(velocity);
    Logger.recordOutput("/subsystems/swerve/setpoint velocity", velocity);
    Logger.recordOutput("/subsystems/swerve/velocity", velocities[2]);
    velocity = velocities[2];

    Double[] accelerations = new Double[] { accX, accY, alpha };
    double acceleration = 0;
    for (int i = 0; i < 2; i++) {
      acceleration += Math.pow(accelerations[i], 2);
    }
    acceleration = Math.sqrt(acceleration);
    Logger.recordOutput("/subsystems/swerve/setpoint acceleration", acceleration);
    Logger.recordOutput("/subsystems/swerve/setpoint rotational acceleration", accelerations[2]);

  }

  /**
   * log all serve data
   */
  private void logData() {
    // logging of our module states
    Double[] realStates = {
        modules[0].getState().angle.getRadians(),
        modules[0].getState().speedMetersPerSecond,
        modules[1].getState().angle.getRadians(),
        modules[1].getState().speedMetersPerSecond,
        modules[2].getState().angle.getRadians(),
        modules[2].getState().speedMetersPerSecond,
        modules[3].getState().angle.getRadians(),
        modules[3].getState().speedMetersPerSecond
    };

    // SmartDashboard.puTarr("Swerve: Real States",realSwerveModuleStates);
    Double[] desiredStates = {
        modules[0].getDesiredState().angle.getRadians(),
        modules[0].getDesiredState().speedMetersPerSecond,
        modules[1].getDesiredState().angle.getRadians(),
        modules[1].getDesiredState().speedMetersPerSecond,
        modules[2].getDesiredState().angle.getRadians(),
        modules[2].getDesiredState().speedMetersPerSecond,
        modules[3].getDesiredState().angle.getRadians(),
        modules[3].getDesiredState().speedMetersPerSecond
    };

    Logger.recordOutput("/subsystems/swerve/real states/module 1 angle", realStates[0]);
    Logger.recordOutput("/subsystems/swerve/real states/module 2 position", realStates[1]);
    Logger.recordOutput("/subsystems/swerve/real states/module 2 angle", realStates[2]);
    Logger.recordOutput("/subsystems/swerve/real states/module 2 position", realStates[3]);
    Logger.recordOutput("/subsystems/swerve/real states/module 3 angle", realStates[4]);
    Logger.recordOutput("/subsystems/swerve/real states/module 3 position", realStates[5]);
    Logger.recordOutput("/subsystems/swerve/real states/module 4 angle", realStates[6]);
    Logger.recordOutput("/subsystems/swerve/real states/module 4 position", realStates[7]);
    Logger.recordOutput("/subsystems/swerve/desired states/module 1 angle", desiredStates[0]);
    Logger.recordOutput("/subsystems/swerve/desired states/module 2 position", desiredStates[1]);
    Logger.recordOutput("/subsystems/swerve/desired states/module 2 angle", desiredStates[2]);
    Logger.recordOutput("/subsystems/swerve/desired states/module 2 position", desiredStates[3]);
    Logger.recordOutput("/subsystems/swerve/desired states/module 3 angle", desiredStates[4]);
    Logger.recordOutput("/subsystems/swerve/desired states/module 3 position", desiredStates[5]);
    Logger.recordOutput("/subsystems/swerve/desired states/module 4 angle", desiredStates[6]);
    Logger.recordOutput("/subsystems/swerve/desired states/module 4 position", desiredStates[7]);

    // odometry logging
    Logger.recordOutput("/subsystesm/swerve/pose x", getPose().getX());
    Logger.recordOutput("/subsystesm/swerve/pose y", getPose().getY());
    Logger.recordOutput("/subsystesm/swerve/rotation", getPose().getRotation().getRadians());
    Logger.recordOutput("/subsystems/swerve/utilize vision", utilizeVision);

    // gyro logging
    Logger.recordOutput("/subsystems/swerve/rotational velocity", (gyroData.yawDeg - yaw) / 0.02);
    Logger.recordOutput("/subsystems/swerve/gyro yaw", gyroData.yawDeg);
    yaw = gyroData.yawDeg;
    Logger.recordOutput("/subsystems/swerve/gyro pitch", gyroData.pitchDeg);
    Logger.recordOutput("/subsystems/swerve/gyro roll", gyroData.rollDeg);
    Logger.recordOutput("/subsystems/swerve/is gyro connected", gyroData.isConnected);
    Logger.recordOutput("/subsystems/swerve/gyro rotation", getRotation2d().getDegrees());

    // velocity and acceleration logging
    double robotVelocity = Math.hypot(getChassisSpeeds().vxMetersPerSecond,
        getChassisSpeeds().vyMetersPerSecond);
    Logger.recordOutput("/subsystems/swerve/gyro rotation", (robotVelocity - velocity) / .02);
    Logger.recordOutput("/subsystems/swerve/gyro rotation", robotVelocity);
    Logger.recordOutput("/subsystems/swerve/gyro rotation", this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
  }

  @Override
  public void periodic() {
    gyro.updateData(gyroData);
    updateOdometry();

    // periodic method for individual modules
    for (int i = 0; i < 4; i++) {
      modules[i].periodic();
    }

    logData();

    driveMotorData.get("drive_left").appliedVolts = modules[0].getModuleData().driveAppliedVolts;
    driveMotorData.get("drive_left").position = modules[0].getModuleData().drivePositionM;
    driveMotorData.get("drive_left").velocity = modules[0].getModuleData().driveVelocityMPerSec;
    driveMotorData.get("drive_left").acceleration = modules[0].getModuleData().driveAccelerationMPerSecSquared;

    turningMotorData.get("turning_left").appliedVolts = modules[0].getModuleData().turnAppliedVolts;
    turningMotorData.get("turning_left").position = modules[0].getModuleData().turnPositionRad;
    turningMotorData.get("turning_left").velocity = modules[0].getModuleData().turnVelocityRadPerSec;
    turningMotorData.get("turning_left").acceleration = 0;

    rotationalMotorData.get("drive_left").appliedVolts = modules[0].getModuleData().driveAppliedVolts;
    rotationalMotorData.get("drive_left").position = modules[0].getModuleData().drivePositionM;
    rotationalMotorData.get("drive_left").velocity = modules[0].getModuleData().driveVelocityMPerSec / 0.533;
    rotationalMotorData.get("drive_left").acceleration = modules[0].getModuleData().driveAccelerationMPerSecSquared / 0.533;
  }
}
