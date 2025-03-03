package frc.robot.commands.roller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;

public class MaintainCommand extends Command {
    private static RollerStates targetState = RollerStates.MAINTAIN;
    private Roller roller;

    public MaintainCommand(Roller roller) {
        this.roller = roller;
        addRequirements(roller);
    }

    @Override
    public void initialize() {
        roller.setState(targetState);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        Robot.coralRoller.setState(RollerStates.STOP);
        Robot.scoringRoller.setState(RollerStates.STOP);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}