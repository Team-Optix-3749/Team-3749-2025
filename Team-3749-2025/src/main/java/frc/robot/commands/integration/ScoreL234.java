package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;
import frc.robot.subsystems.arm.coral.CoralConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
/*
 * ScoreL234 command for scoring coral on L2, L3, L4
 */
public class ScoreL234 extends Command {
    private final ElevatorStates elevatorState;
    private boolean handoffComplete = false;

    public ScoreL234(ElevatorStates elevatorState) {
        this.elevatorState = elevatorState;
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }

    @Override
    public void initialize() {
        if (Robot.chuteRoller.hasPiece()) { 
            Robot.elevator.setState(elevatorState);
            Robot.coralArm.setState(CoralConstants.ArmStates.STOWED);
            Robot.chuteRoller.setState(RollerConstants.RollerStates.MAINTAIN);
            Robot.coralRoller.setState(RollerConstants.RollerStates.STOP);
            System.out.println("scoreL234 start");
        } else if (Robot.coralRoller.hasPiece()) {
            Robot.elevator.setState(ElevatorStates.STOW);
            Robot.coralArm.setState(CoralConstants.ArmStates.HAND_OFF);
            Robot.coralRoller.setState(RollerConstants.RollerStates.MAINTAIN); 
            Robot.chuteRoller.setState(RollerConstants.RollerStates.INTAKE);
        } else {
            this.cancel();
        }
    }

    @Override
    public void execute() {
        if (Robot.coralArm.getState() == CoralConstants.ArmStates.HAND_OFF && Robot.coralArm.getIsStableState() 
                && Robot.elevator.getState() == ElevatorStates.STOW && Robot.elevator.getIsStableState()) { 
            Robot.coralRoller.setState(RollerConstants.RollerStates.SCORE); 
            handoffComplete = true;
        }

        if (handoffComplete && !Robot.coralRoller.hasPiece() && Robot.chuteRoller.hasPiece()) { 
            Robot.chuteRoller.setState(RollerStates.MAINTAIN);
            Robot.elevator.setState(elevatorState);
        }
        if (Robot.elevator.getState() == elevatorState && Robot.elevator.getIsStableState()) {
            Robot.chuteRoller.setState(RollerConstants.RollerStates.SCORE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.coralArm.setState(CoralConstants.ArmStates.STOWED);
        Robot.elevator.setState(ElevatorStates.STOW);
        Robot.chuteRoller.setState(RollerConstants.RollerStates.STOP);
        Robot.coralRoller.setState(RollerConstants.RollerStates.STOP);
        System.out.println("scoreL234 end");
    }

    @Override
    public boolean isFinished() {
        System.out.println("scoreL234 finishing: " + Robot.chuteRoller.hasPiece());
        return !Robot.chuteRoller.hasPiece();
    }
}
