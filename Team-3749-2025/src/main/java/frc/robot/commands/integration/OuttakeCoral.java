package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.coral.CoralArm;
import frc.robot.subsystems.led.LEDConstants.LEDPattern;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.Roller;

public class OuttakeCoral extends Command {
    private final CoralArm coralArm;
    private final Roller coralRoller;

    public OuttakeCoral(CoralArm coralArm, Roller coralRoller) {
        this.coralArm = coralArm;
        this.coralRoller = coralRoller;
    }

    @Override
    public void initialize() {
        coralRoller.setState(RollerConstants.RollerStates.SCORE);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        coralRoller.setState(RollerConstants.RollerStates.STOP);
        Robot.led.setLEDPattern(LEDPattern.WHITE);
    }

    @Override
    public boolean isFinished() {
        return !coralArm.hasPiece();
    }
}
