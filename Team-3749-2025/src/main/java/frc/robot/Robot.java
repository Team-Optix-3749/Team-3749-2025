// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.arm.climb.ClimbArm;
import frc.robot.subsystems.arm.coral.CoralArm;

import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.implementations.AlgaeRoller;
import frc.robot.subsystems.roller.implementations.CoralRoller;
import frc.robot.subsystems.roller.implementations.ScoringRoller;
import frc.robot.subsystems.swerve.Swerve;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static Swerve swerve = new Swerve();
  public static Roller algaeRoller = new AlgaeRoller();
  public static Roller coralRoller = new CoralRoller();
  public static Roller scoringRoller = new ScoringRoller();

  public static Elevator elevator = new Elevator();

  public static CoralArm coralArm = new CoralArm();
  public static ClimbArm climbArm = new ClimbArm();

  public static LEDs leDs = new LEDs(1);
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
 

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
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
  }
}
