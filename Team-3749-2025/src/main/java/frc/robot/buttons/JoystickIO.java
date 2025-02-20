package frc.robot.buttons;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;

import frc.robot.commands.arm.SetClimbArmState;
import frc.robot.commands.arm.SetCoralArmState;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.commands.integration.Handoff;
import frc.robot.commands.integration.IntakeFloor;
import frc.robot.commands.integration.IntakeSource;
import frc.robot.commands.integration.ScoreL234;
import frc.robot.commands.roller.MaintainCommand;
import frc.robot.commands.roller.OuttakeRoller;
import frc.robot.commands.roller.RunRoller;
import frc.robot.commands.swerve.SwerveDefaultCommand;
import frc.robot.subsystems.arm.climb.ClimbArmConstants;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;

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

    private static final Command climbStow = new SetClimbArmState(Robot.climbArm, ClimbArmConstants.ArmStates.STOWED);
    private static final Command climb = new SetClimbArmState(Robot.climbArm, ClimbArmConstants.ArmStates.CLIMB);
    private static final Command coralHandOff = new SetCoralArmState(Robot.coralArm,
            CoralArmConstants.ArmStates.HAND_OFF);
    private static final Command coralPickUp = new SetCoralArmState(Robot.coralArm,
            CoralArmConstants.ArmStates.CORAL_PICKUP);
    private static final Command coralL1 = new SetCoralArmState(Robot.coralArm,
            CoralArmConstants.ArmStates.L1);

    private static final SetElevatorState l1 = new SetElevatorState(ElevatorStates.L1);
    private static final SetElevatorState l2 = new SetElevatorState(ElevatorStates.L2);
    private static final SetElevatorState l3 = new SetElevatorState(ElevatorStates.L3);
    private static final SetElevatorState l4 = new SetElevatorState(ElevatorStates.L4);

    private static final RunRoller algaeRun = new RunRoller(Robot.algaeRoller);
    private static final RunRoller coralRunIntake = new RunRoller(Robot.coralRoller);
    private static final OuttakeRoller coralRunOuttake = new OuttakeRoller(Robot.coralRoller);
    private static final RunRoller scoringRun = new RunRoller(Robot.scoringRoller);

    private static final MaintainCommand algaeMaintain = new MaintainCommand(Robot.algaeRoller);
    private static final MaintainCommand coralMaintain = new MaintainCommand(Robot.coralRoller);
    private static final MaintainCommand scoringMaintain = new MaintainCommand(Robot.scoringRoller);

    private static final IntakeSource intakeSource = new IntakeSource();
    private static final ScoreL234 scoreL234 = new ScoreL234(ElevatorStates.L2);

    private static ButtonBoard buttonBoard = new ButtonBoard();

    public JoystickIO() {
    }

    public static void getButtonBindings() {

        if (DriverStation.isJoystickConnected(1)) {
            // if both xbox controllers are connected
            pilotAndOperatorBindings();
        } else if (DriverStation.isJoystickConnected(0)) {
            // if only one xbox controller is connected
            pilotBindings();
        } else if (Robot.isSimulation()) {
            // will show not connected if on
            pilotAndOperatorBindings();
            // simBindings();
        } else {

        }

        setDefaultCommands();
    }

    /**
     * If both controllers are plugged in (pi and op)
     */
    public static void pilotAndOperatorBindings() {
        // gyro reset
        // pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));
        
        // // Checking voltage for all subsystems
        // operator.rightBumper().onTrue(Commands.run(() -> Robot.elevator.setVoltage(-1.5)));
        // operator.b().onTrue(Commands.run(() -> Robot.coralArm.setVoltage(Robot.subsystemVoltageSetter.get())));
        // operator.x().onTrue(Commands.run(() -> Robot.climbArm.setVoltage(4))).onFalse(Commands.run(()->Robot.climbArm.setVoltage(0)));
        // operator.y().onTrue(Commands.run(() -> Robot.climbArm.setVoltage(8))).onFalse(Commands.run(()->Robot.climbArm.setVoltage(0)));

        // operator.y().whileTrue(Commands.run(() -> Robot.algaeRoller.setVoltage(Robot.subsystemVoltageSetter.get()))).onFalse(Commands.run(() -> Robot.algaeRoller.setVoltage(0)));
        // operator.leftBumper().whileTrue(Commands.run(() -> Robot.coralRoller.setVoltage(Robot.subsystemVoltageSetter.get())));
        // operator.leftBumper().whileFalse(Commands.runOnce(() -> Robot.coralRoller.stop()));
        // operator.rightBumper().onTrue(Commands.run(() -> Robot.scoringRoller.setVoltage(Robot.subsystemVoltageSetter.get())));//.onFalse(Commands.run(() -> Robot.scoringRoller.setVoltage(0)));
        //kanhay's keyboard comment (donot remove need for college apps bc i coded a robot)
        // operator.leftBumper().onTrue(Commands.run(() -> Robot.coralRoller.setVoltage(Robot.subsystemVoltageSetter.get())));//.onFalse(Commands.run(() -> Robot.scoringRoller.setVoltage(0)));

        // pilot.a().onTrue(Commands.run(() -> Robot.swerve.setTurnVoltage(3), Robot.swerve));//.onFalse(Commands.run(() -> Robot.scoringRoller.setVoltage(0)));


        // All elevator stages
        // operator.a().onTrue(l1);
        // operator.b().onTrue(l2);
        // operator.x().onTrue(l3);
        // operator.y().onTrue(l4);

        // Climb + Coral Arms
        // operator.a().onTrue(climbStow);
        // operator.b().onTrue(climb);
 

        // Run
        // operator.a().whileTrue(algaeRun);

        // pilot.a().onTrue(new IntakeFloor());
        // pilot.b().onTrue(new Handoff());
        // operator.x().whileTrue(scoreL234);

        // operator.a().onTrue(coralRunIntake);
        operator.x().onTrue(coralL1);
        operator.y().onTrue(coralHandOff);
        operator.a().onTrue(coralPickUp);

        // operator.x().whileTrue(scoringRun);

        // Maintain
        // operator.a().whileTrue(algaeMaintain);
        // operator.b().whileTrue(coralMaintain);
        // operator.x().whileTrue(scoringMaintain);

        // operator.leftBumper().onTrue(intakeSource);
        // operator.rightBumper().onTrue(scoreL234);
        // operator.b().onTrue(Commands.runOnce(() -> Robot.scoringRoller.setVoltage(Robot.subsystemVoltageSetter.get())));
    }

    public static void pilotBindings() {
        // gyro reset
        pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));
        pilot.a().onTrue(new IntakeFloor());
        pilot.b().onTrue(new Handoff());
        pilot.x().whileTrue(scoreL234);
        // Example binding
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
                                () -> {
                                    if (pilot.y().getAsBoolean()) {
                                        return 1.0;
                                    }
                                    return 0.0;
                                }));
    }

}
