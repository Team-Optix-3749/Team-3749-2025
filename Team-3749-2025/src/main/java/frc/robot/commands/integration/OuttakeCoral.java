package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.arm.coral.CoralArmConstants;

/*
 * OuttakeCoral command for shooting coral from coral arm
 */
public class OuttakeCoral extends Command {

    public OuttakeCoral() {
        // ensures other commands do not infere while this is active
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }

    @Override
    public void initialize() {
        Robot.coralArm.setState(CoralArmConstants.ArmStates.CORAL_PICKUP);
        Robot.coralRoller.setState(RollerConstants.RollerStates.STOP);
        Robot.elevator.setState(ElevatorStates.STOW);
    }

    @Override
    public void execute() {
        if (Robot.coralArm.getState() == CoralArmConstants.ArmStates.CORAL_PICKUP && Robot.coralArm.getIsStableState()) {
            Robot.coralRoller.setState(RollerConstants.RollerStates.SCORE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.elevator.setState(ElevatorStates.STOW);
        Robot.coralRoller.setState(RollerConstants.RollerStates.STOP);
    }

    /**
     * Command finishes when coralRoller does not have coral
     */
    @Override
    public boolean isFinished() {
        return !Robot.coralRoller.hasPiece();
    }
}


