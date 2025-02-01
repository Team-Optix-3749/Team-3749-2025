// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.arm.climb.ClimbArm;
import frc.robot.subsystems.arm.coral.CoralArm;
import frc.robot.subsystems.example.ExampleSubsystem;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.implementations.AlgaeRoller;
import frc.robot.subsystems.roller.implementations.CoralRoller;
import frc.robot.subsystems.roller.implementations.ScoringRoller;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.ShuffleData;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  public static Swerve swerve = new Swerve();
  public static Roller algaeRoller = new AlgaeRoller();
  public static Roller coralRoller = new CoralRoller();
  public static Roller scoringRoller = new ScoringRoller();
  
  public static ExampleSubsystem subsystem = new ExampleSubsystem();
  public static Elevator elevator = new Elevator();

  public static CoralArm coralArm = new CoralArm();
  public static ClimbArm climbArm = new ClimbArm();

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
		switch (Constants.currentMode) {
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
