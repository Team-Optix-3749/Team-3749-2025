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
    private final ElevatorStates state;
    private boolean handoffComplete = false;
    public static Command activeScoreCommand = null;



    public ScoreL234(ElevatorStates state) {
        this.state = state;
        addRequirements(Robot.getAllSuperStructureSubsystems());
        System.out.println("ScoreL234 Constructor: Command Created with state = " + state);
    }

    @Override
    public void initialize() {
        activeScoreCommand = this;

        if (Robot.scoringRoller.hasPiece()) {
            Robot.elevator.setState(state);
            Robot.scoringRoller.setState(RollerConstants.RollerStates.MAINTAIN);
            Robot.coralArm.setState(CoralConstants.ArmStates.STOWED);
        } else if (Robot.coralRoller.hasPiece()) {
            Robot.coralArm.setState(CoralConstants.ArmStates.HAND_OFF);
            Robot.elevator.setState(ElevatorStates.STOW);
            Robot.coralRoller.setState(RollerConstants.RollerStates.MAINTAIN);
        } else {
            this.cancel();
        }
    }

    @Override
    public void execute() {

        if ((Robot.coralArm.getState() == CoralConstants.ArmStates.HAND_OFF) && Robot.coralArm.getIsStableState() &&
            (Robot.elevator.getState() == ElevatorStates.STOW) && Robot.elevator.getIsStableState()) {
            Robot.coralRoller.setState(RollerConstants.RollerStates.SCORE);
            Robot.scoringRoller.setState(RollerConstants.RollerStates.RUN);
            handoffComplete = true;
        }

        if (handoffComplete && !Robot.coralRoller.hasPiece() && Robot.scoringRoller.hasPiece()) {
            Robot.scoringRoller.setState(RollerStates.MAINTAIN);
            Robot.elevator.setState(state);
        }

        if (Robot.elevator.getState() == state && Robot.elevator.getIsStableState()) {
            Robot.scoringRoller.setState(RollerConstants.RollerStates.SCORE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        activeScoreCommand = null;
        Robot.coralArm.setState(CoralConstants.ArmStates.STOWED);
        Robot.elevator.setState(ElevatorStates.STOW);
        Robot.scoringRoller.setState(RollerConstants.RollerStates.STOP);
        Robot.coralRoller.setState(RollerConstants.RollerStates.STOP);
    }

    @Override
    public boolean isFinished() {
        boolean finished = !Robot.scoringRoller.hasPiece();

        return finished && this.isScheduled();
    }
}
