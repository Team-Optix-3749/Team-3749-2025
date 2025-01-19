package frc.robot.commands.roller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;

public class RunCommand extends Command {
    private static RollerStates targetState = RollerStates.RUN;

    public RunCommand() {
    }

    @Override
    public void initialize() {
        Robot.algaeRoller.setState(targetState);
        Robot.coralRoller.setState(targetState);
        Robot.scoringRoller.setState(targetState);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        Robot.algaeRoller.setState(RollerStates.STOP);
        Robot.coralRoller.setState(RollerStates.STOP);
        Robot.scoringRoller.setState(RollerStates.STOP);        
    }
    
    @Override
    public boolean isFinished() {
        return false; 
    }
}
