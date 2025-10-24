package frc.robot.commands.integration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.commands.auto.AutoConstants;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;
import frc.robot.subsystems.swerve.ToPosConstants;
import frc.robot.utils.UtilityFunctions;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
import frc.robot.subsystems.arm.coral.CoralArmConstants.ArmStates;

/*
 * ScoreL234 command for scoring coral on L2, L3, L4
 */
@SuppressWarnings("unused")
public class ScoreL234Manual extends Command {
    private final ElevatorStates elevatorState;
    private boolean handoffComplete = false;
    private boolean pieceRecognized = false;
    private double scoreTimestamp = Double.MAX_VALUE;

    private double elevTimestamp = Double.MAX_VALUE;

    /**
     * 
     * @param elevatorState
     */
    public ScoreL234Manual(ElevatorStates elevatorState) {
        this.elevatorState = elevatorState;
        // ensures other commands do not infere while this is active
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }

    @Override
    public void initialize() {
        Robot.elevator.setState(elevatorState);
        Robot.scoringRoller.setState(RollerStates.STOP);
        Robot.coralArm.setState(ArmStates.STOW);

        elevTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        Robot.coralArm.setState(ArmStates.STOW);
        Robot.scoringRoller.setState(RollerStates.STOP);
        Robot.coralRoller.setState(RollerStates.STOP);
        pieceRecognized = false;
        scoreTimestamp = Double.MAX_VALUE;
        elevTimestamp = Double.MAX_VALUE;
    }

    /**
     * Command finishes when scoringRoller does not have coral and command is being
     * scheduled
     */
    @Override
    public boolean isFinished() {
        // return !Robot.scoringRoller.hasPiece() && pieceRecognized && Timer.getFPGATimestamp() - scoreTimestamp > 0.6
        //         && this.isScheduled();
        return false;
        // return false;
    }

}
