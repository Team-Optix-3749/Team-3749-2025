// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystems.arm.algae.AlgaeArm;
import frc.robot.subsystems.arm.climb.ClimbArm;
import frc.robot.subsystems.arm.coral.CoralArm;
import frc.robot.subsystems.elevator.Elevator;

import frc.robot.subsystems.example.ExampleSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.ShuffleData;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  public static Swerve swerve = new Swerve();
  public static ExampleSubsystem subsystem = new ExampleSubsystem();

  public static AlgaeArm algaeArm = new AlgaeArm();
  public static CoralArm coralArm = new CoralArm();
  public static ClimbArm climbArm = new ClimbArm();
  public static Elevator elevator = new Elevator();

  private ShuffleData<Double> batteryVoltageLog = new ShuffleData<Double>("DS", "battery voltage", 0.0);
  private ShuffleData<Boolean> isBrownedOutLog = new ShuffleData<Boolean>("DS", "brownout", false);
  private ShuffleData<Double> cpuTempLog = new ShuffleData<Double>("DS", "cpu temp", 0.0);
  private ShuffleData<Double> CANUtilizationLog = new ShuffleData<Double>("DS", "CAN utilizaition", 0.0);
  private ShuffleData<String> radioStatusLog = new ShuffleData<String>("DS", "radio status", "kOff");
  private ShuffleData<String> allianceLog = new ShuffleData<String>("DS", "alliance", "Red");
  private ShuffleData<Boolean> FMSLog = new ShuffleData<Boolean>("DS", "FMS connected", false);
  private RobotContainer m_robotContainer;
  PowerDistribution pdp = new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging

  public Robot() {
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    batteryVoltageLog.set(RobotController.getBatteryVoltage());
    cpuTempLog.set(RobotController.getCPUTemp());
    CANUtilizationLog.set(RobotController.getCANStatus().percentBusUtilization);
    radioStatusLog.set(RobotController.getRadioLEDState().name());
    isBrownedOutLog.set(RobotController.isBrownedOut());
    allianceLog.set(DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get().name() : "None");
    FMSLog.set(DriverStation.isFMSAttached());

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
    pdp.close();
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
  public void simulationInit(){
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
  }
}
