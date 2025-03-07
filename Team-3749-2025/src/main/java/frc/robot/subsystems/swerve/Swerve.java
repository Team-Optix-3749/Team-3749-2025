// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.buttons.JoystickIO;
import frc.robot.buttons.ButtonBoard.ScoringMode;
import frc.robot.commands.auto.AutoConstants;
import frc.robot.commands.auto.AutoUtils;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.swerve.GyroIO.GyroData;
import frc.robot.subsystems.swerve.ToPosConstants.Setpoints.PPSetpoints;
import frc.robot.subsystems.swerve.sim.GyroSim;
import frc.robot.subsystems.swerve.sim.SwerveModuleSim;
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

  private PIDController xController = new PIDController(AutoConstants.kPDrive, AutoConstants.kIDrive,
      AutoConstants.kDDrive);
  private PIDController yController = new PIDController(AutoConstants.kPDrive, AutoConstants.kIDrive,
      AutoConstants.kDDrive);
  private PIDController turnController = new PIDController(AutoConstants.kPTurn, AutoConstants.kITurn,
      AutoConstants.kDTurn);

  private Pose2d positionSetpoint = new Pose2d();
  private Pose2d velocitySetpoint = new Pose2d();

  private boolean utilizeVision = true;
  private double velocity = 0;

  private int currentPPSetpointIndex = 0; // what "index" do we currently want to go to for OTF
  private int currentPPApproachSetpointIndex = 0;

  private boolean isOTF = false; // are we OTF driving rn

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
      // gyro = new GyroSim();
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
        new Pose2d(new Translation2d(5.773, 3.963), Rotation2d.fromDegrees(180)),
        VecBuilder.fill(0.045, 0.045, 0.004), // 6328's 2024 numbers with factors of 1.5x, 1.5x, 2x
        VecBuilder.fill(VisionConstants.StandardDeviations.PreMatch.xy,
            VisionConstants.StandardDeviations.PreMatch.xy,
            VisionConstants.StandardDeviations.PreMatch.thetaRads));

    // auto PID settings
    turnController.enableContinuousInput(-Math.PI, Math.PI);
    turnController.setIZone(AutoConstants.turnIZone);
    turnController.setTolerance(AutoConstants.turnToleranceRad / 2);
    xController.setTolerance(AutoConstants.driveToleranceMeters / 3);
    yController.setTolerance(AutoConstants.driveToleranceMeters / 3);
    xController.setIZone(AutoConstants.driveIZone);
    yController.setIZone(AutoConstants.driveIZone);

    // put us on the field with a default orientation
    resetGyro();
    setOdometry(new Pose2d(3, 3, new Rotation2d(0)));
    logSetpoints(1.33, 0, 0, 5.53, 0, 0, 0, 0, 0);

  }

  public int getPPSetpointIndex() {
    return currentPPApproachSetpointIndex;
  }

  public int getApproachSetpointIndex() {
    return currentPPApproachSetpointIndex;
  }

  public boolean getIsOTF() {
    return isOTF;
  }

  public boolean getRotated() {
    return UtilityFunctions.withinMargin(0.01, modules[0].getModuleData().turnPositionRad,
        Rotation2d.fromDegrees(45).getRadians())
        && UtilityFunctions.withinMargin(0.01, modules[1].getModuleData().turnPositionRad,
            Rotation2d.fromDegrees(135).getRadians())
        && UtilityFunctions.withinMargin(0.01, modules[2].getModuleData().turnPositionRad,
            Rotation2d.fromDegrees(225).getRadians())
        && UtilityFunctions.withinMargin(0.01, modules[3].getModuleData().turnPositionRad,
            Rotation2d.fromDegrees(315).getRadians());

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

  // called by OTF and Choreo, given the position and velocity at the points
  // generated in the
  // dynamic path:
  // calc the speeds and throw them into the speed
  public void followSample(Pose2d positions, Pose2d velocities) {
    positionSetpoint = positions;
    velocitySetpoint = velocities;

    double xPID = xController.calculate(getPose().getX(), positions.getX());
    xPID = MathUtil.clamp(xPID, -1, 1);
    if (xController.atSetpoint()) {
      xPID = 0;
    }

    double yPID = yController.calculate(getPose().getY(), positions.getY());
    yPID = MathUtil.clamp(yPID, -1, 1);

    if (yController.atSetpoint()) {
      yPID = 0;
    }

    double turnPID = turnController.calculate(getPose().getRotation().getRadians(),
        positions.getRotation().getRadians());
    // turnPID = MathUtil.clamp(turnPID, -Math.PI/2, Math.PI/2);

    if (turnController.atSetpoint()) {
      turnPID = 0;
    }
    Logger.recordOutput("Swerve/auto/turn PID", turnPID);
    Logger.recordOutput("Swerve/auto/x PID", xPID);
    Logger.recordOutput("Swerve/auto/y PID", yPID);

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        new ChassisSpeeds(
            xPID + velocities.getX(),
            yPID + velocities.getY(),
            turnPID
                + velocities.getRotation().getRadians()),
        getPose().getRotation());
    logSetpoints(positions, velocities);
    Logger.recordOutput("Swerve/auto/velocity hypt", Math.sqrt(
        speeds.vxMetersPerSecond * speeds.vxMetersPerSecond + speeds.vyMetersPerSecond * speeds.vyMetersPerSecond));
    setChassisSpeeds(speeds);
  }

  public void followSample(SwerveSample sample, boolean isFlipped) {
    System.out.println("folowing sample");

    // ternaries are for x-axis flipping

    double xPos = sample.x;
    double xVel = sample.vx;
    double xAcc = sample.ax;

    double yPos = isFlipped ? AutoUtils.flipper.flipY(sample.y) : sample.y;
    double yVel = isFlipped ? -sample.vy : sample.vy;
    double yAcc = isFlipped ? -sample.ay : sample.ay;

    double heading = isFlipped ? new Rotation2d(Math.PI - sample.heading).rotateBy(new Rotation2d(Math.PI)).getRadians()
        : sample.heading;
    double omega = isFlipped ? -sample.omega : sample.omega;
    double alpha = isFlipped ? -sample.alpha : sample.alpha;

    positionSetpoint = new Pose2d(xPos, yPos, new Rotation2d(heading));
    velocitySetpoint = new Pose2d(xVel, yVel, new Rotation2d(omega));

    followSample(positionSetpoint, velocitySetpoint);
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

  public void setApproachSetpointIndex(int index) {
    currentPPApproachSetpointIndex = index;
  }

  public void setIsOTF(boolean otf) {
    isOTF = otf;
  }

  public void setPPSetpointIndex(int index) {
    currentPPSetpointIndex = index;
  }

  // this is only used for testing, pressing B to cycle through all of the stuff
  public void cyclePPSetpoint() {
    isOTF = false;
    currentPPSetpointIndex++;
    if (JoystickIO.getButtonBoard().getScoringMode() == ScoringMode.ALGAE &&
        currentPPSetpointIndex >= 2 && currentPPSetpointIndex <= 25) {
    }
    if (JoystickIO.getButtonBoard().getScoringMode() == ScoringMode.L1 &&
        currentPPSetpointIndex >= 2 && currentPPSetpointIndex <= 24 && currentPPSetpointIndex % 2 == 0) {
      currentPPSetpointIndex++;
    }
    if (JoystickIO.getButtonBoard().getScoringMode() != ScoringMode.L1 &&
        currentPPSetpointIndex >= 3 && currentPPSetpointIndex <= 25 && currentPPSetpointIndex % 2 != 0) {
      currentPPSetpointIndex++;
    }

    if (currentPPSetpointIndex >= ToPosConstants.Setpoints.PPSetpoints.values().length) {
      currentPPSetpointIndex = 0;
    }
    showOTFEndPoint();
    showOTFApproachPoint();
  }

  public PPSetpoints getPPSetpoint() {
    return PPSetpoints.values()[currentPPSetpointIndex];
  }

  public Pose2d getPositionSetpoint() {
    return positionSetpoint;
  }

  public Pose2d getVelocitySetpoint() {
    return velocitySetpoint;
  }

  public boolean reachedSwerveSetpoint(Pose2d pose) {
    // Pose2d setpoint = getPositionSetpoint();
    // double xMargin = (Math.sin(setpoint.getRotation().getRadians())
    // * AutoConstants.driveToleranceMeters) + 0.015;

    // double yMargin = (Math.cos(setpoint.getRotation().getRadians())
    // * AutoConstants.driveToleranceMeters) + 0.015;

    // Logger.recordOutput("Swerve/auto/x margin", xMargin);
    // Logger.recordOutput("Swerve/auto/y margin", yMargin);

    // boolean withinPositionMargin = UtilityFunctions.withinMargin(
    // new Pose2d(xMargin,
    // yMargin,
    // new Rotation2d(AutoConstants.turnToleranceRad)),
    // getPose(), setpoint);

    // Pose2d velocities = new Pose2d(getChassisSpeeds().vxMetersPerSecond,
    // getChassisSpeeds().vyMetersPerSecond,
    // new Rotation2d(getChassisSpeeds().omegaRadiansPerSecond));

    // boolean withinVelocityMargin = UtilityFunctions.withinMargin(
    // new Pose2d(0.1, 0.1, new Rotation2d(Units.degreesToRadians(3))),
    // getVelocitySetpoint(),
    // velocities);

    // return withinPositionMargin && withinVelocityMargin;

    return UtilityFunctions.withinMargin(
        new Pose2d(AutoConstants.driveToleranceMeters,
            AutoConstants.driveToleranceMeters,
            new Rotation2d(AutoConstants.turnToleranceRad)),
        getPose(), pose);

  }

  // called when the button board is pressed with the (ppsetpoint)"index" the
  // button is associated w to drive to

  public void startOnTheFly(int setpointIndex) {
    setIsOTF(false);
    Robot.elevator.setState(ElevatorStates.STOW);
    currentPPSetpointIndex = setpointIndex;

    if (JoystickIO.getButtonBoard().getScoringMode() == ScoringMode.ALGAE &&
        currentPPSetpointIndex >= 2 && currentPPSetpointIndex <= 25) {
    }

    if (JoystickIO.getButtonBoard().getScoringMode() == ScoringMode.L1 &&
        currentPPSetpointIndex >= 2 && currentPPSetpointIndex <= 24 && currentPPSetpointIndex % 2 == 0) {
      currentPPSetpointIndex++;
    }

    if (JoystickIO.getButtonBoard().getScoringMode() != ScoringMode.L1 &&
        currentPPSetpointIndex >= 3 && currentPPSetpointIndex <= 25 && currentPPSetpointIndex % 2 != 0) {
      currentPPSetpointIndex++;
    }
    showOTFEndPoint();
    showOTFApproachPoint();
    isOTF = true;
  }

  /**
   * Manually sets our odometry position
   * 
   * @param pose - Pose2d object of what to set our position to
   */
  public void setOdometry(Pose2d pose) {
    // System.out.println("Set Odometry: " + pose.getX() + ", " + pose.getY() + ", "
    // + pose.getRotation().getDegrees());
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

  public void setRotation() {
    modules[0].setTurnPosition(45 * (Math.PI / 180));
    modules[1].setTurnPosition(135 * Math.PI / 180);

    modules[2].setTurnPosition(225 * Math.PI / 180);
    modules[3].setTurnPosition(315 * Math.PI / 180);
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
    System.out.println("rest gyro");
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

  public void logSetpoints(Pose2d position, Pose2d velocity) {
    logSetpoints(position.getX(), velocity.getX(), 0, position.getY(), velocity.getY(), 0,
        position.getRotation().getRadians(), velocity.getRotation().getRadians(), 0);

  }

  /**
   * logs all setpoints for the swerve subsystem in autonomous functions
   * 
   * @param the swerve sample of setpoints
   */
  public void logSetpoints(double posX, double velX, double accX, double posY, double velY, double accY, double heading,
      double omega, double alpha) {
    // setpoint logging for automated driving
    double[] positions = new double[] { posX, posY, heading };
    Logger.recordOutput("Swerve/auto/position setpoint", positions);
    Transform2d poseDiff = new Pose2d(posX, posY, new Rotation2d(heading)).minus(getPose());
    Logger.recordOutput("Swerve/auto/position error", poseDiff);

    Double[] velocities = new Double[] { velX, velY, omega };
    double velocity = 0;
    velocity += Math.pow(velocities[0], 2);
    velocity += Math.pow(velocities[1], 2);

    velocity = Math.sqrt(velocity);
    Logger.recordOutput("Swerve/auto/setpoint velocity", velocity);
    Logger.recordOutput("Swerve/auto/setpoint rotational velocity", velocities[2]);
    velocity = velocities[2];
    Logger.recordOutput("Swerve/auto/velocity hypt", Math.sqrt(
        velX * velX + velY * velY));

    Double[] accelerations = new Double[] { accX, accY, alpha };
    double acceleration = 0;
    acceleration += Math.pow(accelerations[0], 2);
    acceleration += Math.pow(accelerations[1], 2);

    acceleration = Math.sqrt(acceleration);
    Logger.recordOutput("Swerve/auto/setpoint acceleration", acceleration);
    Logger.recordOutput("Swerve/auto/setpoint rotational acceleration", accelerations[2]);

    Logger.recordOutput("at setpoints", reachedSwerveSetpoint(positionSetpoint));
  }

  // this is only really relevant for testing purposes: as this is logged as
  // endgoal position or smth like that
  // shows what the end position will be like in advantage scope
  public void showOTFEndPoint() {
    Logger.recordOutput("Swerve/auto/otf end goal",
        new double[] { getPPSetpoint().setpoint.getX(), getPPSetpoint().setpoint.getY(),
            getPPSetpoint().setpoint.getRotation().getRadians() });
  }

  public void showOTFApproachPoint() {
    Logger.recordOutput("Swerve/auto/otf approach point",
        new double[] { getPPSetpoint().approachPoint.getX(), getPPSetpoint().approachPoint.getY(),
            getPPSetpoint().approachPoint.getRotation().getRadians() });
  }

  /**
   * log all serve data
   */
  private void logData() {
    // logging of our module states
    double[] realStates = {
        modules[0].getState().angle.getRadians(),
        modules[0].getState().speedMetersPerSecond,
        modules[1].getState().angle.getRadians(),
        modules[1].getState().speedMetersPerSecond,
        modules[2].getState().angle.getRadians(),
        modules[2].getState().speedMetersPerSecond,
        modules[3].getState().angle.getRadians(),
        modules[3].getState().speedMetersPerSecond
    };

    double[] desiredStates = {
        modules[0].getDesiredState().angle.getRadians(),
        modules[0].getDesiredState().speedMetersPerSecond,
        modules[1].getDesiredState().angle.getRadians(),
        modules[1].getDesiredState().speedMetersPerSecond,
        modules[2].getDesiredState().angle.getRadians(),
        modules[2].getDesiredState().speedMetersPerSecond,
        modules[3].getDesiredState().angle.getRadians(),
        modules[3].getDesiredState().speedMetersPerSecond
    };

    Logger.recordOutput("Swerve/real states", realStates);
    Logger.recordOutput("Swerve/desired states", desiredStates);
    Logger.recordOutput("Swerve/auto/isOTF", isOTF);

    double[] odometry = {
        getPose().getX(),
        getPose().getY(),
        getPose().getRotation().getRadians() };
    Pose3d pose = new Pose3d(getPose());

    Logger.recordOutput("Swerve/odometry", odometry);
    Logger.recordOutput("Swerve/odometry pose3d", pose);
    Logger.recordOutput("Swerve/utilizeVision", utilizeVision);

    // gyro logging
    Logger.recordOutput("Swerve/gyro/Yaw", gyroData.yawDeg);
    // yaw = gyroData.yawDeg;
    Logger.recordOutput("Swerve/gyro/Pitch", gyroData.pitchDeg);
    Logger.recordOutput("Swerve/gyro/Roll", gyroData.rollDeg);
    Logger.recordOutput("Swerve/gyro/isConnected", gyroData.isConnected);
    Logger.recordOutput("Swerve/heading", getRotation2d().getDegrees());

    // velocity and acceleration logging
    double robotVelocity = Math.hypot(getChassisSpeeds().vxMetersPerSecond,
        getChassisSpeeds().vyMetersPerSecond);

    Logger.recordOutput("Swerve/robot velocity", robotVelocity);
    Logger.recordOutput("Swerve/robot rotational velocity", getChassisSpeeds().omegaRadiansPerSecond);

    Logger.recordOutput("Swerve/robot acceleration", (robotVelocity -
        velocity) / .02);

    velocity = robotVelocity;

    String currentCommand = this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName();
    Logger.recordOutput("Swerve/currentCommand", currentCommand);

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

  }

}