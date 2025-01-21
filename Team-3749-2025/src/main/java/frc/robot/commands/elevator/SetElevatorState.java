package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;

/**
 * Elevator states command
 * 
 * @author Dhyan Soni
 * @author Andrew Ge
 */
public class SetElevatorState extends Command {
    ElevatorStates state;
    public SetElevatorState(ElevatorStates state) {
        this.state = state;
        super.addRequirements(Robot.elevator);
    }

    @Override
    public void initialize() {
        Robot.elevator.setState(state);
    }

    // if further state change are needed over time, put them here with conditionals
    @Override
    public void execute() {
        /*if (Robot.elevator.getIsStableState()) {
            Robot.elevator.setState(ElevatorStates.STOP);
        }*/
    }

    @Override
    public void end(boolean interupted) {
        Robot.elevator.setState(ElevatorStates.STOW);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
