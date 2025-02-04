package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.coral.CoralConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;
/**
 * CoralIntakeSource command for intaking coral from source
 */
public class CoralIntakeSource extends Command {
    public CoralIntakeSource() {
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }

    @Override
    public void initialize() {
        Robot.coralArm.setState(CoralConstants.ArmStates.SOURCE);
        Robot.elevator.setState(ElevatorStates.STOW);
        Robot.scoringRoller.setState(RollerStates.STOP);
        Robot.coralRoller.setState(RollerStates.RUN);
    }    

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        Robot.coralArm.setState(CoralConstants.ArmStates.HAND_OFF);
        Robot.coralRoller.setState(RollerStates.MAINTAIN);
    }

    @Override
    public boolean isFinished() {
        return Robot.coralRoller.hasPiece();
    }
}
