// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.AutoUtils;
import frc.robot.commands.auto.Autos;
import frc.robot.utils.JoystickIO;

public class RobotContainer {
  
  // private final setstate stop = new setstate(ElevatorStates.STOP);
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    DriverStation.removeRefreshedDataEventHandle(44000);

    // When to go into brownout protection
    RobotController.setBrownoutVoltage(7.0);

    // robot setup
    JoystickIO.getButtonBindings();
    AutoUtils.initAuto();
    
  }

  public Command getAutonomousCommand() {
    return Autos.getSelectedCommand();
  }
}
