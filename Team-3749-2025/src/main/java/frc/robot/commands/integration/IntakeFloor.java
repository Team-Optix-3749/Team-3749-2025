package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.coral.CoralArm;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
import frc.robot.subsystems.led.LEDConstants.LEDPattern;
import frc.robot.subsystems.roller.Roller;

public class IntakeFloor extends Command {
    private final CoralArm coralArm;
    private final Roller coralRoller;

    public IntakeFloor(CoralArm coralArm, Roller coralRoller) {
        this.coralArm = coralArm;
        this.coralRoller = coralRoller;
    }

    @Override
    public void initialize() {
        coralArm.setState(CoralArmConstants.ArmStates.CORAL_PICKUP);
    }

    @Override
    public void execute() {
        coralRoller.setState(RollerConstants.RollerStates.RUN);
        
        if (coralArm.getState() == CoralArmConstants.ArmStates.CORAL_PICKUP && coralArm.getIsStableState()) {
            Robot.led.setLEDPattern(LEDPattern.BLUE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        coralArm.setState(CoralArmConstants.ArmStates.STOPPED);
        coralRoller.setState(RollerConstants.RollerStates.MAINTAIN);
    }

    @Override
    public boolean isFinished() {
        if (coralArm.hasPiece()){
            Robot.led.setLEDPattern(LEDPattern.GREEN);
            return true;
        }
        return false;
    }
}
