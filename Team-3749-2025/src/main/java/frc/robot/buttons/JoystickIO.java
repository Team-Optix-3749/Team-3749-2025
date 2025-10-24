
package frc.robot.buttons;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.subsystems.leds.LEDConstants.LEDColor;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;
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
import frc.robot.RobotContainer;
import frc.robot.buttons.ButtonBoard.ScoringMode;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.integration.Climb;
import frc.robot.commands.integration.CoralIntakeSource;
import frc.robot.commands.integration.Handoff;
import frc.robot.commands.integration.IntakeFloor;
import frc.robot.commands.integration.IntakeFloorEnd;
import frc.robot.commands.integration.Handoff;
import frc.robot.commands.integration.IntakeFloor;
import frc.robot.commands.integration.IntakeSource;
import frc.robot.commands.integration.KnockAlgae;
import frc.robot.commands.integration.ScoreL1;
import frc.robot.commands.integration.ScoreL234;
import frc.robot.commands.integration.ScoreL234Manual;
import frc.robot.commands.integration.ScoringModeConditionalHandoff;
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
                        // pilotAndOperatorBindings();
                        pilotAndOperatorBindings();
                } else {
                        pilotAndOperatorBindings();
                        // testBindings();

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
                                .onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(14)));
                buttonBoard.buttonReefZoneLeft6
                                .onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(8)));
                buttonBoard.buttonReefZoneRight4
                                .onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(10)));
                buttonBoard.buttonReefZoneRight5
                                .onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(12)));
                buttonBoard.buttonReefZoneRight6
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
                pilot.back().whileTrue(Commands.run(Robot.swerve::lockModules, Robot.swerve));

                // gyro reset
                pilot.start().onTrue(Commands.runOnce(Robot.swerve::resetGyro));
                // outtake arm
                pilot.leftBumper().onTrue(new ScoreL1());
                pilot.x().onTrue(new ScoreL234Manual(ElevatorStates.L2));
                pilot.b().onTrue(new ScoreL234Manual(ElevatorStates.L3));
                pilot.y().onTrue(new ScoreL234Manual(ElevatorStates.L4));
                pilot.povUp().onTrue(Commands.runOnce(() -> Robot.scoringRoller.setState(RollerStates.SCORE)))
                                .onFalse(Commands.runOnce(() -> Robot.scoringRoller.setState(RollerStates.STOP)));
                pilot.a().onTrue(new Handoff())
                                .onFalse(Commands.runOnce(() -> System.out.println("interupt ground intake"),
                                                Robot.coralRoller, Robot.coralArm));

                operator.x().onTrue(new ScoreL234Manual(ElevatorStates.L2));
                operator.b().onTrue(new ScoreL234Manual(ElevatorStates.L3));
                operator.y().onTrue(new ScoreL234Manual(ElevatorStates.L4));
                operator.a().onTrue(Commands.runOnce(() -> Robot.scoringRoller.setState(RollerStates.SCORE)))
                                .onFalse(Commands.runOnce(() -> Robot.scoringRoller.setState(RollerStates.STOP)));

                pilot.leftTrigger().onTrue(new IntakeFloor()).onFalse(
                                new IntakeFloorEnd());
                pilot.rightTrigger().onTrue(new IntakeSource())
                                .onFalse(Commands.runOnce(() -> System.out.println("interupt ground intake"),
                                                Robot.coralRoller, Robot.coralArm));

                pilot.povLeft().onTrue(Commands.runOnce(() -> Robot.swerve.goToNearestBranch(true)));
                pilot.povRight().onTrue(Commands.runOnce(() -> Robot.swerve.goToNearestBranch(false)));
                
                operator.povLeft().onTrue(Commands.runOnce(() -> Robot.swerve.goToNearestBranch(true)));
                operator.povRight().onTrue(Commands.runOnce(() -> Robot.swerve.goToNearestBranch(false)));

                // reset
                pilot.povDown().onTrue(new Reset());

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
                }).onTrue(Commands.runOnce(() -> {
                        System.out.println("otf cancelled");
                        Robot.swerve.setIsOTF(false);
                }));
        }

        public static void testBindings() {
                // pilotAndOperatorBindings();

                // operator.povLeft().onTrue(new ScoreL234(ElevatorStates.L2));
                // operator.povRight().onTrue(new ScoreL234(ElevatorStates.L3));

                // pilot.povRight().whileTrue(Commands.run(() -> rumblePilot()));

                // // OTF by controller - Closest apriltag
                // pilot.x().onTrue(Commands.runOnce(() -> {
                // ToPos.setSetpointByClosestReefBranch(true);
                // Robot.swerve.setIsOTF(true);
                // }));
                // pilot.b().onTrue(Commands.runOnce(() -> {
                // ToPos.setSetpointByClosestReefBranch(false);
                // Robot.swerve.setIsOTF(true);
                // }));

                // pilot.povRight().onTrue(Commands.runOnce(
                // () -> Robot.climbArm.setState(ClimbArmConstants.ArmStates.STOWED)))
                // .onFalse(Commands.runOnce(
                // () -> Robot.climbArm.setState(ClimbArmConstants.ArmStates.STOPPED)));

                // operator.povLeft().onTrue(Commands.runOnce(() ->
                // Robot.swerve.cyclePPSetpoint()));

                // operator.a().onTrue(Robot.swerve.startOnTheFly(0);)

                pilot.a().onTrue(Commands.runOnce(() -> Robot.scoringRoller.setHasPiece(false)));
                pilot.b().onTrue(Commands.runOnce(() -> Robot.scoringRoller.setHasPiece(true)));

                pilot.x().onTrue(Commands.runOnce(() -> Robot.coralRoller.setHasPiece(false)));
                pilot.y().onTrue(Commands.runOnce(() -> Robot.coralRoller.setHasPiece(true)));
        }

        public static void pilotBindings() {
                // gyro reset
                pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));

        }

        public static void simBindings() {
                // pilotBindings();
                // pilot.a().onTrue(Commands.runOnce(() ->
                // Robot.scoringRoller.setHasPiece(false)));
                // pilot.b().onTrue(Commands.runOnce(() ->
                // Robot.scoringRoller.setHasPiece(true)));

                // pilot.x().onTrue(Autos.run3Piece());

                pilot.a().onTrue(Commands.runOnce(() -> Robot.coralRoller.setHasPiece(true)));
                pilot.a().onTrue(Commands.runOnce(() -> Robot.coralRoller.setHasPiece(false)));
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
