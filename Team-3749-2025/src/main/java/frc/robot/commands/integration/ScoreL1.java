package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.coral.CoralConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;
/*
 * ScoreL1 command for scoring coral on L1 using coral arm
 */
public class ScoreL1 extends Command {

    public ScoreL1() {
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }

    @Override
    public void initialize() {
        Robot.elevator.setState(ElevatorConstants.ElevatorStates.STOW);
        Robot.coralArm.setState(CoralConstants.ArmStates.L1);
        Robot.coralRoller.setState(RollerConstants.RollerStates.MAINTAIN);
        Robot.chuteRoller.setState(RollerConstants.RollerStates.STOP);
    }

    @Override
    public void execute() {
        if (Robot.coralArm.getState() == CoralConstants.ArmStates.L1 && Robot.coralArm.getIsStableState()) {
            Robot.coralRoller.setState(RollerConstants.RollerStates.SCORE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.coralArm.setState(CoralConstants.ArmStates.STOWED);
        Robot.coralRoller.setState(RollerStates.STOP);
    }

    @Override
    public boolean isFinished() {
        return !Robot.coralRoller.hasPiece();
    }
}
