package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;
import frc.robot.subsystems.arm.coral.CoralConstants;

/*
 * Handoff command for coral intake to chute
 * 
 * @author Dhyan Soni
 */

public class Handoff extends Command {

    public Handoff() {
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }

    @Override
    public void initialize() {
        Robot.elevator.setState(ElevatorStates.STOW);
        Robot.coralArm.setState(CoralConstants.ArmStates.HAND_OFF);
        Robot.coralRoller.setState(RollerConstants.RollerStates.MAINTAIN);
        Robot.chuteRoller.setState(RollerConstants.RollerStates.INTAKE);
    }

    @Override
    public void execute() {
        if (Robot.coralArm.getState() == CoralConstants.ArmStates.HAND_OFF && Robot.coralArm.getIsStableState()
                && Robot.elevator.getState() == ElevatorStates.STOW && Robot.elevator.getIsStableState()) { 
            Robot.coralRoller.setState(RollerConstants.RollerStates.SCORE); 
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.coralArm.setState(CoralConstants.ArmStates.STOWED);
        Robot.coralRoller.setState(RollerConstants.RollerStates.STOP);
        Robot.chuteRoller.setState(RollerStates.MAINTAIN);
        System.out.println("end");
    }

    @Override
    public boolean isFinished() {
        System.out.println("handoff hasPiece: " + Robot.chuteRoller.hasPiece());
        return Robot.chuteRoller.hasPiece();
    }
}
