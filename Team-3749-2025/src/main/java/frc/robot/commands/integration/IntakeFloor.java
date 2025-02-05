package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;
import frc.robot.subsystems.arm.coral.CoralConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
/*
 * IntakeFloor command for intaking coral from floor
 */
public class IntakeFloor extends Command {
    public static Command activeIntakeFloorCommand = null; 

    public IntakeFloor() {
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }

    @Override
    public void initialize() {
        activeIntakeFloorCommand = this; 
        Robot.coralArm.setState(CoralConstants.ArmStates.CORAL_PICKUP);
        Robot.elevator.setState(ElevatorStates.STOW);
        Robot.coralRoller.setState(RollerConstants.RollerStates.RUN);
        Robot.scoringRoller.setState(RollerStates.STOP);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        activeIntakeFloorCommand = null; 
        Robot.coralArm.setState(CoralConstants.ArmStates.STOWED);
        Robot.coralRoller.setState(RollerConstants.RollerStates.MAINTAIN);
    }

    @Override
    public boolean isFinished() {
        return Robot.coralRoller.hasPiece() && this.isScheduled();
    }
}
