
package frc.robot.buttons;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.subsystems.leds.LEDConstants.LEDColor;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;
import frc.robot.buttons.ButtonBoard.ScoringMode;

import frc.robot.commands.integration.Climb;
import frc.robot.commands.integration.CoralIntakeSource;
import frc.robot.commands.integration.Handoff;
import frc.robot.commands.integration.IntakeFloor;
import frc.robot.commands.integration.Handoff;
import frc.robot.commands.integration.IntakeFloor;
import frc.robot.commands.integration.IntakeSource;
import frc.robot.commands.integration.KnockAlgae;
import frc.robot.commands.integration.ScoreL1;
import frc.robot.commands.integration.ScoreL234;

import frc.robot.commands.swerve.OnTheFly;
import frc.robot.commands.swerve.SwerveDefaultCommand;
import frc.robot.subsystems.arm.climb.ClimbArmConstants;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.leds.LEDConstants;
import frc.robot.subsystems.swerve.ToPos;
import frc.robot.subsystems.swerve.ToPosConstants;
import frc.robot.commands.integration.PrepareClimb;
import frc.robot.commands.integration.Reset;
import frc.robot.utils.MiscConstants.ControllerConstants;

/**
 * Util class for button bindings
 * 
 * @author Rohin Sood
 * @author Noah Simon
 */
@SuppressWarnings("unused")
public class JoystickIO {

        private static final CommandXboxController pilot = new CommandXboxController(0);
        private static final CommandXboxController operator = new CommandXboxController(1);
        private static final OnTheFly onTheFly = new OnTheFly();

        private static ButtonBoard buttonBoard = new ButtonBoard();

        public JoystickIO() {
        }

        /**
         * Activates the rumble motor per method call (requires continuous calls)
         */
        public static void rumblePilot() {
                pilot.setRumble(RumbleType.kBothRumble, 0.1); // 0 to 100% (0 to 1 argument)
        }

        public static void getButtonBindings() {

                if (Robot.isSimulation()) {
                        // will show not connected if on
                        pilotAndOperatorBindings();
                        // simBindings();
                } else {
                        pilotAndOperatorBindings();

                }

                setDefaultCommands();
        }

        public static void bindButtonBoard() {
                buttonBoard.buttonLeftSource
                                .onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(0)));
                buttonBoard.buttonRightSource
                                .onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(1)));
                buttonBoard.buttonReefZoneLeft1
                                .onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(2)));
                buttonBoard.buttonReefZoneRight1
                                .onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(4)));
                buttonBoard.buttonReefZoneRight2
                                .onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(6)));
                buttonBoard.buttonReefZoneRight3
                                .onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(8)));
                buttonBoard.buttonReefZoneRight4
                                .onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(10)));
                buttonBoard.buttonReefZoneRight5
                                .onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(12)));
                buttonBoard.buttonReefZoneRight6
                                .onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(14)));
                buttonBoard.buttonReefZoneLeft6
                                .onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(16)));
                buttonBoard.buttonReefZoneLeft5
                                .onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(18)));
                buttonBoard.buttonReefZoneLeft4
                                .onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(20)));
                buttonBoard.buttonReefZoneLeft3
                                .onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(22)));
                buttonBoard.buttonReefZoneLeft2
                                .onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(24)));
                buttonBoard.buttonl1.onTrue(Commands.runOnce(() -> buttonBoard.setScoringMode(ScoringMode.L1)));
                buttonBoard.buttonl2.onTrue(Commands.runOnce(() -> buttonBoard.setScoringMode(ScoringMode.L2)));
                buttonBoard.buttonl3.onTrue(Commands.runOnce(() -> buttonBoard.setScoringMode(ScoringMode.L3)));
                buttonBoard.buttonl4.onTrue(Commands.runOnce(() -> buttonBoard.setScoringMode(ScoringMode.L4)));
                buttonBoard.buttonAlgaeKnockoff
                                .onTrue(Commands.runOnce(() -> buttonBoard.setScoringMode(ScoringMode.ALGAE)));

                buttonBoard.buttonReset.onTrue(new Reset());
                buttonBoard.buttonUtilityA.onTrue(Commands.runOnce(
                                () -> Robot.coralRoller.setHasPiece(!Robot.coralRoller.hasPiece())));
                ;
        }

        /**
         * If both controllers are plugged in (pi and op)
         */
        public static void pilotAndOperatorBindings() {
                // gyro reset
                pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));

                // intake floor
                pilot.leftTrigger().onTrue(new IntakeFloor()).onFalse(
                                Commands.runOnce(() -> System.out.println("interupt ground intake"), Robot.coralArm));
                // intake source w arm
                pilot.rightTrigger().onTrue(new CoralIntakeSource());
                // outtake arm
                pilot.leftBumper().onTrue(new ScoreL1());
                // intake source w elevator
                pilot.rightBumper().onTrue(new IntakeSource());
                // handoff
                pilot.a().onTrue(new Handoff());
                // Climb - Reset to cancel
                pilot.y().onTrue(new PrepareClimb()).onFalse(new Climb());
                // reset
                pilot.povDown().onTrue(new Reset());
                // toggle hasPiece
                pilot.povUp().onTrue(Commands
                                .runOnce(() -> Robot.coralRoller.setHasPiece(!Robot.coralRoller.hasPiece())));

                // scoring
                operator.a().onTrue(Commands.runOnce(() -> buttonBoard.setScoringMode(ScoringMode.L1)));
                operator.x().onTrue(Commands.runOnce(() -> buttonBoard.setScoringMode(ScoringMode.L2)));
                operator.b().onTrue(Commands.runOnce(() -> buttonBoard.setScoringMode(ScoringMode.L3)));
                operator.y().onTrue(Commands.runOnce(() -> buttonBoard.setScoringMode(ScoringMode.L4)));
                // Algae
                operator.leftTrigger().onTrue(new KnockAlgae(ElevatorStates.ALGAE_LOW));
                operator.rightTrigger().onTrue(new KnockAlgae(ElevatorStates.ALGAE_HIGH));
                // Reset
                operator.povDown().onTrue(new Reset());

                // OTF Binding
                new Trigger(() -> Robot.swerve.getIsOTF()).onTrue(onTheFly);

                // OTF Cancel
                new Trigger(() -> {
                        if (Math.abs(pilot.getLeftX()) > ControllerConstants.deadband
                                        || Math.abs(pilot.getLeftY()) > ControllerConstants.deadband
                                        || Math.abs(pilot.getRightX()) > ControllerConstants.deadband) {
                                return true;
                        }
                        return false;
                }).onTrue(Commands.runOnce(() -> Robot.swerve.setIsOTF(false)));

                // OTF Triggers
                ToPosTriggers.createOTFTriggers();

                // Button board
                bindButtonBoard();

        }

        public static void testBindings() {
                pilotAndOperatorBindings();

                pilot.povRight().whileTrue(Commands.run(() -> rumblePilot()));

                // OTF by controller - Closest apriltag
                pilot.x().onTrue(Commands.runOnce(() -> {
                        ToPos.setSetpointByClosestReefBranch(true);
                        Robot.swerve.setIsOTF(true);
                }));
                pilot.b().onTrue(Commands.runOnce(() -> {
                        ToPos.setSetpointByClosestReefBranch(false);
                        Robot.swerve.setIsOTF(true);
                }));

                pilot.povRight().onTrue(Commands.runOnce(
                                () -> Robot.climbArm.setState(ClimbArmConstants.ArmStates.STOWED)))
                                .onFalse(Commands.runOnce(
                                                () -> Robot.climbArm.setState(ClimbArmConstants.ArmStates.STOPPED)));

                operator.povLeft().onTrue(Commands.runOnce(() -> Robot.swerve.cyclePPSetpoint()));

        }

        public static void pilotBindings() {
                // gyro reset
                pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));

        }

        public static void simBindings() {
                pilotBindings();
        }

        /**
         * Sets the default commands
         */
        public static void setDefaultCommands() {
                if (Robot.isSimulation()) {
                        setSimDefaultCommands();
                } else {
                        setRealDefaultCommands();
                }
        }

        private static void setRealDefaultCommands() {
                Robot.swerve.setDefaultCommand(
                                new SwerveDefaultCommand(
                                                () -> pilot.getLeftX(),
                                                () -> pilot.getLeftY(),
                                                () -> pilot.getRightX()));
        }

        private static void setSimDefaultCommands() {
                Robot.swerve
                                .setDefaultCommand(
                                                new SwerveDefaultCommand(
                                                                () -> pilot.getLeftX(),
                                                                () -> pilot.getLeftY(),
                                                                () -> pilot.getRightX()));
        }

        public static ButtonBoard getButtonBoard() {
                return buttonBoard;
        }
}
