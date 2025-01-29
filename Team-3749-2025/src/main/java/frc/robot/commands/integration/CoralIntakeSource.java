package frc.robot.commands.integration;

import org.opencv.photo.Photo;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
// import frc.robot.subsystems.arm.coral.CoralArm;
import frc.robot.subsystems.arm.coral.CoralConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
// import frc.robot.subsystems.roller.PhotoelectricIO;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;

public class CoralIntakeSource extends Command {
    public CoralIntakeSource() {
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }

    @Override
    public void initialize() {
        Robot.coralArm.setState(CoralConstants.ArmStates.SOURCE);
        Robot.elevator.setState(ElevatorStates.STOW);
        Robot.scoringRoller.setState(RollerStates.STOP);
        Robot.coralRoller.setState(RollerStates.STOP);
    }

    @Override
    public void execute() {
        if (Robot.coralArm.getState() == CoralConstants.ArmStates.SOURCE && Robot.coralArm.getIsStableState()) {
            Robot.coralRoller.setState(RollerStates.RUN);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.coralRoller.setState(RollerStates.MAINTAIN);
        System.out.println("DEBUG : intake command end");
    }

    @Override
    public boolean isFinished() {
        System.out.println("coralintakesource: " + Robot.coralArm.hasPiece());
        return Robot.coralArm.hasPiece();
    }
}
