package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.RollerConstants;

/*
 * KnockAlgae command for knocking algae off reef
 */
public class KnockAlgae extends Command {
    private final ElevatorStates elevatorState;

    /**
     * 
     * @param elevatorState 
     */
    public KnockAlgae(ElevatorStates elevatorState) {
        this.elevatorState = elevatorState;
        // ensures other commands do not infere while this is active
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }

    @Override
    public void initialize() {
        Robot.coralArm.setState(CoralArmConstants.ArmStates.STOWED);
        Robot.algaeRoller.setState(RollerConstants.RollerStates.RUN);
        Robot.elevator.setState(elevatorState);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        Robot.elevator.setState(ElevatorStates.STOW);
        Robot.algaeRoller.setState(RollerConstants.RollerStates.STOP);
    }

    // command finishes when elevator reaches desired state and command is being scheduled
    @Override
    public boolean isFinished() {
        return Robot.elevator.getIsStableState() && this.isScheduled(); 
    }
}
