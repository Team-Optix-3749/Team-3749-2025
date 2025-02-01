package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.climb.ClimbConstants.ArmStates;
import frc.robot.subsystems.arm.climb.ClimbArm;

/**
 * Sets the selected state for a given arm
 */
public class SetClimbArmState extends Command {

    private final ClimbArm arm;
    private final ArmStates state;

    public SetClimbArmState(ClimbArm arm, ArmStates state, double setPoint) {
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
