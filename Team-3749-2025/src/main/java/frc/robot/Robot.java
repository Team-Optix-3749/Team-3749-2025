// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.arm.climb.ClimbArm;
import frc.robot.subsystems.arm.coral.CoralArm;

import frc.robot.subsystems.roller.implementations.AlgaeRoller;
import frc.robot.subsystems.roller.implementations.CoralRoller;
import frc.robot.subsystems.roller.implementations.ScoringRoller;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.ToPosConstants;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.MiscConstants;

public class Robot extends LoggedRobot {
private Field2d field2d = new Field2d();
  private Command m_autonomousCommand;

  public static Swerve swerve = new Swerve();
  public static AlgaeRoller algaeRoller = new AlgaeRoller();
  public static CoralRoller coralRoller = new CoralRoller();
  public static ScoringRoller scoringRoller = new ScoringRoller();

  public static Elevator elevator = new Elevator();

  public static CoralArm coralArm = new CoralArm();
  public static ClimbArm climbArm = new ClimbArm();
  public static LoggedTunableNumber subsystemVoltageSetter = new LoggedTunableNumber("/subsystems/setVoltage", 1);

  private RobotContainer m_robotContainer;

  public static Subsystem[] getAllSuperStructureSubsystems() {
    return new Subsystem[] {algaeRoller, coralRoller, scoringRoller, elevator, coralArm, climbArm};
  }

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (MiscConstants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // /Publish hexagon points to NetworkTables
    List<Pose2d> hexagonPoses = new ArrayList<>();
    for (Translation2d vertex : ToPosConstants.ReefVerticies.getHexagonVertices()) {
      hexagonPoses.add(new Pose2d(vertex, new Rotation2d()));
    }

    // Add hexagon points as "Object" on the field
    field2d.getObject("Hexagon").setPoses(hexagonPoses);
  }

  @Override
  public void disabledInit() {
    swerve.setBreakMode(false);
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
    swerve.setBreakMode(true);
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationInit() {
}
}