package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.coral.CoralArm;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
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
    }

    @Override
    public boolean isFinished() {
        return !coralArm.hasPiece();
    }
}
