package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.coral.CoralArm;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.led.LEDConstants.LEDPattern;

//works for l2-4 change name
public class ScoreL234 extends Command {
    private final Chute chute;
    private final CoralArm coralArm;
    private final Elevator elevator;
    private final Roller coralRoller;
    private final Roller scoringRoller;
    private final ElevatorStates state;
    private boolean handoffComplete = false;

    public ScoreL234(Chute chute, CoralArm coralArm, Elevator elevator, Roller coralRoller, Roller scoringRoller,
            ElevatorStates state) {
        this.chute = chute;
        this.coralArm = coralArm;
        this.elevator = elevator;
        this.coralRoller = coralRoller;
        this.scoringRoller = scoringRoller;
        this.state = state;
    }

    @Override
    public void initialize() {
        if (chute.hasPiece()) {
            elevator.setState(state);
            scoringRoller.setState(RollerConstants.RollerStates.MAINTAIN);
        } else {
            coralArm.setState(CoralArmConstants.ArmStates.HAND_OFF);
            elevator.setState(ElevatorStates.STOW);
            coralRoller.setState(RollerConstants.RollerStates.MAINTAIN);
        }
        if (chute.hasPiece() && coralArm.getState() == CoralArmConstants.ArmStates.HAND_OFF) {
            elevator.setState(ElevatorStates.STOW);
        }
    }

    @Override
    public void execute() {
        if (coralArm.getState() == CoralArmConstants.ArmStates.HAND_OFF && coralArm.getIsStableState()
                && elevator.getState() == ElevatorStates.STOW) { // add working elevator isStableState later
            coralRoller.setState(RollerConstants.RollerStates.SCORE);
            handoffComplete = true;
        }
        if (handoffComplete && !coralArm.hasPiece() && chute.hasPiece()) { // should work once hasPiece logic is made
            elevator.setState(state);
        }

        if (elevator.getState() == state && elevator.getIsStableState()) {
            scoringRoller.setState(RollerConstants.RollerStates.SCORE);
            Robot.led.setLEDPattern(LEDPattern.BLUE);
        }
        if (!Robot.elevator.getIsStableState()) {
            Robot.led.setLEDPattern(LEDPattern.YELLOW);
        }
        if (!Robot.coralArm.getIsStableState()) {
            Robot.led.setLEDPattern(LEDPattern.YELLOW);
        }

    }

    @Override
    public void end(boolean interrupted) {
        coralArm.setState(CoralArmConstants.ArmStates.STOPPED);
        elevator.setState(ElevatorStates.STOP);
        scoringRoller.setState(RollerConstants.RollerStates.STOP);
        coralRoller.setState(RollerConstants.RollerStates.STOP);
        Robot.led.setLEDPattern(LEDPattern.WHITE);
    }

    @Override
    public boolean isFinished() {
        return elevator.getState() == state && !chute.hasPiece(); // add working elevator isStableState later
    }
}
