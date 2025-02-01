package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.coral.CoralConstants.ArmStates;
import frc.robot.subsystems.arm.coral.CoralArm;

/**
 * Sets the selected state for a given arm
 */
public class SetCoralArmState extends Command {

    private final CoralArm arm;
    private final ArmStates state;

    public SetCoralArmState(CoralArm arm, ArmStates state, double setPoint) {
        this.arm = arm;
        this.state = state;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setState(state);
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }

    @Override
    public boolean isFinished() {
        
        return arm.getIsStableState();
        
    }
}
