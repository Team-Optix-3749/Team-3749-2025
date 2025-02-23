package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;

/**
 * CoralIntakeSource command for intaking coral from source
 */
public class IntakeSource extends Command {

    public IntakeSource() {
        // ensures other commands do not infere while this is active
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }

    @Override
    public void initialize() {
        Robot.coralArm.setState(CoralArmConstants.ArmStates.STOWED);
        Robot.elevator.setState(ElevatorStates.SOURCE);
        Robot.scoringRoller.setState(RollerStates.STOP);
        Robot.coralRoller.setState(RollerStates.STOP);
    }    

    @Override
    public void execute() {
        if (Robot.elevator.getIsStableState()) {
            Robot.scoringRoller.setState(RollerStates.INTAKE);
        }
        if (Robot.scoringRoller.hasPiece() && Robot.scoringRoller.getState() != RollerStates.MAINTAIN) {
            Robot.scoringRoller.setState(RollerStates.MAINTAIN);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.elevator.setState(ElevatorStates.STOW);
    }

    /**
     * Command finishes when coralRoller has coral and command is being scheduled
     */
    @Override
    public boolean isFinished() {
        return Robot.coralRoller.hasPiece() && this.isScheduled();
    }
}
