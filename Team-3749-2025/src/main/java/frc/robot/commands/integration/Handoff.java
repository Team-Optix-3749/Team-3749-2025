package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.coral.CoralArm;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
import frc.robot.subsystems.roller.Roller;

/*
 * Handoff command for coral intake to chute
 * 
 * @author Dhyan Soni
 */

public class Handoff extends Command {
    private final Chute chute;
    private final CoralArm coralArm;
    private final Elevator elevator;
    private final Roller coralRoller;

    public Handoff(Chute chute, CoralArm coralArm, Elevator elevator, Roller coralRoller) {
        this.chute = chute;
        this.coralArm = coralArm;
        this.elevator = elevator;
        this.coralRoller = coralRoller;
    }

    @Override
    public void initialize() {
        coralArm.setState(CoralArmConstants.ArmStates.HAND_OFF);
        coralRoller.setState(RollerConstants.RollerStates.MAINTAIN);
    }

    @Override
    public void execute() {
        if (coralArm.getState() == CoralArmConstants.ArmStates.HAND_OFF && coralArm.getIsStableState()){
            elevator.setState(ElevatorConstants.ElevatorStates.STOW);
        }
        if (elevator.getState() == ElevatorStates.STOW){ // might need to edit elevator.getIsStableState()
            coralRoller.setState(RollerConstants.RollerStates.SCORE); // reverse spinning
        }
    }

    @Override
    public void end(boolean interrupted) {
        coralArm.setState(CoralArmConstants.ArmStates.STOPPED);
        coralRoller.setState(RollerConstants.RollerStates.STOP);
    }

    @Override
    public boolean isFinished() {
        return chute.hasPiece();
    }
}
