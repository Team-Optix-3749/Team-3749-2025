package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.coral.CoralArm;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
import frc.robot.subsystems.arm.coral.CoralArmConstants.ArmStates;
import frc.robot.subsystems.led.LEDConstants.LEDPattern;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;

public class ScoreL1 extends Command {
    private final CoralArm coralArm;
    private final Roller coralRoller;
    private final ArmStates state;

    public ScoreL1(CoralArm coralArm, Roller coralRoller, ArmStates state) {
        this.coralArm = coralArm;
        this.coralRoller = coralRoller;
        this.state = state;
    }

    @Override
    public void initialize() {
        coralArm.setState(CoralArmConstants.ArmStates.L1);
        coralRoller.setState(RollerConstants.RollerStates.MAINTAIN);
    }

    @Override
    public void execute() {
        if (coralArm.getState() == CoralArmConstants.ArmStates.L1 && coralArm.getIsStableState()) {
            coralRoller.setState(RollerConstants.RollerStates.SCORE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        coralArm.setState(CoralArmConstants.ArmStates.STOWED);
        coralRoller.setState(RollerStates.STOP);
        Robot.led.setLEDPattern(LEDPattern.WHITE);
    }

    @Override
    public boolean isFinished() {
        if (coralArm.getState() == state && coralArm.getIsStableState()) {
            Robot.led.setLEDPattern(LEDPattern.BLUE);
            return true;
        }
        return false;
    }
}
